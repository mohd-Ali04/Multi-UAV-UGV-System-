#!/usr/bin/env python3
"""Measure the track-envelope loop and a repeat cell from the actual CAD mesh."""
import gzip
import hashlib
from register_io import HASH_POLICY, source_sha256
import json
import math
from pathlib import Path

import numpy as np
import trimesh
import yaml

from native_export import GRADE, read_json, native_matrix_to_si, translation, export_labelled_stl

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
O = REPO / "Simulation/mechanical_design/tins12e/integration/sim_grade"


def loop_segments(circles):
    # Exact support transition normals of the convex outer envelope of circles.
    angles = {0.0, 2*math.pi}
    for a in circles:
        for b in circles:
            vector = np.asarray(a["centre_xz_m"]) - b["centre_xz_m"]
            distance = np.linalg.norm(vector)
            delta = b["radius_m"] - a["radius_m"]
            if distance < 1e-9 or abs(delta) >= distance:
                continue
            phi = math.atan2(vector[1], vector[0])
            offset = math.acos(delta/distance)
            angles.update([(phi-offset)%(2*math.pi), (phi+offset)%(2*math.pi)])
    intervals = []
    values = sorted(angles)
    for start, end in zip(values, values[1:]):
        if end-start < 1e-10:
            continue
        mid = (start+end)/2
        normal = np.array([math.cos(mid), math.sin(mid)])
        winner = int(np.argmax([np.dot(c["centre_xz_m"], normal)+c["radius_m"] for c in circles]))
        if intervals and intervals[-1][2] == winner:
            intervals[-1][1] = end
        else:
            intervals.append([start, end, winner])
    output = []
    for index, (start, end, winner) in enumerate(intervals):
        circle = circles[winner]
        centre, radius = np.array(circle["centre_xz_m"]), circle["radius_m"]
        a = centre + radius*np.array([math.cos(start), math.sin(start)])
        b = centre + radius*np.array([math.cos(end), math.sin(end)])
        output.append(dict(type="arc", centre_xz_m=centre.tolist(), radius_m=radius,
                           start_angle_rad=start, end_angle_rad=end, start_xz_m=a.tolist(), end_xz_m=b.tolist(), length_m=radius*(end-start)))
        following = circles[intervals[(index+1)%len(intervals)][2]]
        target = np.asarray(following["centre_xz_m"]) + following["radius_m"]*np.array([math.cos(end), math.sin(end)])
        distance = np.linalg.norm(target-b)
        if distance > 1e-9:
            output.append(dict(type="straight", start_xz_m=b.tolist(), end_xz_m=target.tolist(), length_m=float(distance)))
    # Forward vehicle motion requires bottom tread motion towards -X.
    output.reverse()
    for segment in output:
        segment["start_xz_m"], segment["end_xz_m"] = segment["end_xz_m"], segment["start_xz_m"]
        if segment["type"] == "arc":
            segment["start_angle_rad"], segment["end_angle_rad"] = segment["end_angle_rad"], segment["start_angle_rad"]
    for left, right in zip(output, output[1:]+output[:1]):
        assert np.linalg.norm(np.array(left["end_xz_m"])-right["start_xz_m"]) < 1e-8
    return output


def main():
    source = O / "cad_extract_utf8/manifest.json"
    manifest = read_json(source)
    surface = manifest["track_surface_geometry"][0]
    definition = next(b for b in manifest["body_definitions"] if b["body_id"] == surface["body_id"])
    occurrences = [o for o in manifest["occurrences"] if surface["body_id"] in o["body_ids"]]
    assert len(occurrences) == 2
    occurrence = max(occurrences, key=lambda o: o["matrix_cm"][7])
    matrix = translation([0, 0, 0.6035]) @ native_matrix_to_si(occurrence["matrix_cm"])
    circles = {}
    for row in surface["cylindrical_faces"]:
        if row["radius_cm"] < 6:
            continue
        centre = (matrix @ np.r_[np.asarray(row["origin_cm"])*0.01, 1])[:3]
        axis = matrix[:3, :3] @ row["axis"]
        assert abs(abs(axis[1])-1) < 1e-8
        key = tuple(np.round(centre[[0, 2]], 7))
        circles[key] = max(circles.get(key, 0), row["radius_cm"]*0.01)
    circles = [dict(centre_xz_m=[float(v) for v in k], radius_m=float(r)) for k, r in sorted(circles.items())]
    assert len(circles) == 4
    segments = loop_segments(circles)
    with gzip.open(source.parent / definition["mesh_file"], "rt", encoding="utf-8") as stream:
        raw = json.load(stream)
    mesh = trimesh.Trimesh(vertices=np.asarray(raw["vertices"]).reshape(-1, 3)*0.01,
                           faces=np.asarray(raw["triangles"]).reshape(-1, 3), process=False)
    mesh.apply_transform(matrix)
    vertices = np.asarray(mesh.vertices)
    support_errors = []
    for angle in np.linspace(0, 2*math.pi, 181):
        n = np.array([math.cos(angle), math.sin(angle)])
        actual = float(np.max(vertices[:, [0, 2]] @ n))
        analytic = max(np.dot(c["centre_xz_m"], n)+c["radius_m"] for c in circles)
        support_errors.append(float(abs(actual-analytic)))
    assert max(support_errors) < 0.002
    bottom = float(vertices[:, 2].min())
    assert abs(bottom) < 1e-7
    flat = np.unique(np.round(vertices[(abs(vertices[:, 2]-bottom)<1e-8)&(abs(vertices[:, 0])<0.4), 0], 6))
    proposals = {round(b-a, 6) for a in flat[:8] for b in flat if 0.03 < b-a < 0.1}
    scored = []
    for pitch in proposals:
        starts = flat[flat+pitch <= flat.max()+1e-8]
        score = float(np.mean([np.min(abs(flat-x-pitch)) < 2e-6 for x in starts]))
        scored.append((score, len(starts), pitch))
    score, matching, pitch = max(scored)
    pitch = float(pitch)
    assert score > 0.95 and matching >= 20
    width = float(np.ptp(vertices[:, 1]))
    assert abs(width-0.15) < 1e-6
    # A period centred on the actual straight-run grouser pattern, not a new part.
    group = flat[(flat > 0.02)&(flat < 0.055)]
    centre_x = float((group.min()+group.max())/2)
    shoe = mesh.slice_plane([centre_x-pitch/2, 0, 0], [1, 0, 0])
    shoe = shoe.slice_plane([centre_x+pitch/2, 0, 0], [-1, 0, 0])
    shoe = shoe.slice_plane([0, 0, bottom+0.06], [0, 0, -1])
    y = float((mesh.bounds[0, 1]+mesh.bounds[1, 1])/2)
    shoe.apply_translation([-centre_x, -y, -bottom])
    shoe.apply_transform(np.diag([-1, 1, -1, 1]))
    shoe_path = PACKAGE / "meshes/visual/track_shoe_visual.stl"
    export_labelled_stl(shoe, shoe_path)
    length = sum(s["length_m"] for s in segments)
    data = dict(schema_version=1, label=GRADE, frame="base_link, X forward/Y left/Z up",
                track_width_m=width, track_outer_envelope_bottom_z_m=bottom,
                shoe_pitch_m=pitch, pitch_basis="Measured 60 mm longitudinal grouser repetition on the supplied continuous rubber-track CAD straight run; NOT verified OEM neutral-axis/chain pitch.",
                pitch_matching_fraction=score, pitch_comparisons=matching,
                path_basis="Outer track-envelope loop from four measured cylindrical-face centres/radii and common external tangents. This is an envelope path for visual animation, not a neutral fibre or pitch-circle model.",
                circles=circles, segments=segments, closed_loop_length_m=length,
                max_sampled_outer_hull_support_error_vs_native_mesh_m=max(support_errors),
                direction="Clockwise in X-Z profile: forward +X vehicle motion gives tread motion -X on the bottom run.",
                shoe_frame="X tangent to tread motion, Y left, Z outward. Cropped geometry extends inward at negative local Z; open crop boundaries are visual only.",
                shoe_mesh=shoe_path.relative_to(PACKAGE).as_posix(), shoe_mesh_triangles=len(shoe.faces),
                shoe_type="REFERENCE_VISUAL_ONLY period cell of a continuous rubber belt; zero additional physical mass, no individual shoe physics bodies or BOM entry",
                sides={"left": {"centre_y_m": abs(y), "track_link": "left_track_proxy_link"},
                       "right": {"centre_y_m": -abs(y), "track_link": "right_track_proxy_link"}},
                animation_policy="URDF uses original complete track solids as static faithful visuals. Web animation may replace those visuals with instanced period cells along this loop, driven by measured track speed. Do not render both sets simultaneously or give cells physical mass. Loop length/pitch need not be an integer; do not infer manufacturing pitch/count.",
                hidden_contact_wheels=[f"{side}_contact_wheel_{i}" for side in ["left", "right"] for i in [1, 2, 3]],
                contact_patch_policy="All six hidden physics cylinders have radius0.13 m and centreZ0.13 m: bottomZ0 coincides with measured track-envelope bottom. Positions/radius are simulation proxies, not OEM road-wheel geometry.",
                source_native_manifest_sha256=source_sha256(source), source_hash_policy=HASH_POLICY)
    output = PACKAGE / "config/track_path.yaml"
    output.write_text(yaml.safe_dump(data, sort_keys=False, width=110), encoding="utf-8", newline="\n")
    (O / "evidence/track-path-export.json").write_text(json.dumps(data, indent=2), encoding="utf-8", newline="\n")
    print(json.dumps(dict(output=str(output), circles=circles, loop_length_m=length, pitch_m=pitch,
                          max_support_error_m=max(support_errors), width_m=width, bottom_z_m=bottom,
                          shoe_triangles=len(shoe.faces))))


if __name__ == "__main__":
    main()
