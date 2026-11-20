#!/usr/bin/env python3
"""Export link-local visuals, primitive collisions and a named-node GLB offline."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh

from native_export import (GRADE, read_json, placed_mesh, merged_visual,
                           export_labelled_stl, bounds_union, translation, DECIMATION_AUDIT)
from collision_primitives import box_from_bounds, horizontal_plate_boxes

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
O = REPO / "Simulation/mechanical_design/tins12e/integration/sim_grade"


def target_triangles(name):
    if name == "base_link":
        return 40000
    if name == "carousel_mount":
        return 18000
    if name in ["tray_a_x_carriage", "tray_b_x_carriage"]:
        return 5000
    if name in ["tray_a", "tray_b"]:
        return 6000
    if "drive_sprocket" in name:
        return 8000
    if "track_proxy" in name:
        return 16000
    if name.startswith(("x_screw", "z_screw")):
        return 800
    if name.startswith("lock_"):
        return 180
    if name.startswith("motor_"):
        return 250
    return 1500


def palette(name):
    if name == "base_link":
        return [0.24, 0.30, 0.22, 1]
    if "track_" in name or "road_wheel" in name or "idler" in name or "carrier" in name:
        return [0.12, 0.13, 0.14, 1]
    if name in ["tray_a", "tray_b"]:
        return [0.53, 0.58, 0.64, 1]
    return [0.45, 0.47, 0.49, 1]


def main():
    draft_path = PACKAGE / "config/link_joint_sim_map.draft.json"
    data = read_json(draft_path)
    assert data["status"] == "DRAFT_MESH_AND_TRACK_EXPORT_PENDING"
    manifest_path = REPO / data["native_cache"]
    cache = manifest_path.parent
    manifest = read_json(manifest_path)
    definitions = {d["body_id"]: d for d in manifest["body_definitions"]}
    bounds_rows = read_json(O / "evidence/crawler-placement-rootproxy-correction.json")["physical_instance_bounds"]
    bounds_lookup = {(b["path"], b["body"]): np.array([b["min_cm"], b["max_cm"]])*0.01 + [0, 0, 0.6035]
                     for b in bounds_rows}
    mount = read_json(O / "evidence/simple-mount-construction.json")
    for part in mount["parts"]:
        occurrence = next(o for o in manifest["occurrences"] if o["component_name"] == part["component"])
        bounds_lookup[(occurrence["path"], part["body"])] = np.array([part["min_cm"], part["max_cm"]])*0.01 + [0, 0, 0.6035]
    assert len(bounds_lookup) == 1526
    links = {link["name"]: link for link in data["links"]}
    grouped = defaultdict(list)
    expected_bounds = defaultdict(list)
    all_meshes = {}
    collisions = defaultdict(list)
    progress_path = O / "evidence/link-mesh-export-progress.json"
    assignments = data["native_body_assignments"]
    for index, row in enumerate(assignments):
        if row["separate_aircraft"] and row["link"] != "uav02_static_aircraft":
            continue
        name = "aircraft_canonical" if row["separate_aircraft"] else row["link"]
        definition = definitions[row["body_id"]]
        progress_path.write_text(json.dumps(dict(status="LOADING_AND_REDUCING_NATIVE_BODIES", assignment_index=index,
                                                 assignments_total=len(assignments), link=name, component=row["component"], body_id=row["body_id"])), encoding="utf-8", newline="\n")
        matrix = np.asarray(row["body_to_link_m"])
        mesh = placed_mesh(cache, definition, matrix)
        grouped[name].append(mesh)
        origin_name = "uav02_dock" if row["separate_aircraft"] else name
        origin = np.asarray(data["zero_frame_origins_in_base_m"][origin_name])
        native_bounds = bounds_lookup[(row["native_path"], row["source_body"])] - origin
        expected_bounds[name].append(native_bounds)
        if row["separate_aircraft"]:
            continue
        # Tracks and running gear are visual bodies; hidden cylinders carry contact.
        # Pins, screws and motor rotors are fixed park visuals in this version.
        visual_only = ("track_proxy" in name or "drive_sprocket" in name or "carrier_wheel" in name
                       or name.startswith(("lock_", "x_screw", "z_screw", "motor_")))
        if visual_only or name == "base_link" or row["component"].startswith(("MOTOR_", "SLK_")):
            continue
        if any(word in row["component"] for word in ["WASHER", "BOLT", "SCREW_M", "_NUT_", "_BUFFER", "ARC_CLAMP_"]):
            continue
        if row["component"] in ["Tray_A_PROVISIONAL", "Tray_B_PROVISIONAL"]:
            collisions[name].extend(horizontal_plate_boxes(mesh, row["native_path"]))
        else:
            collisions[name].append(box_from_bounds(mesh.bounds, row["native_path"]))
    mesh_audit = []
    for name, pieces in grouped.items():
        budget = 20000 if name == "aircraft_canonical" else target_triangles(name)
        progress_path.write_text(json.dumps(dict(status="MERGING_LINK_VISUAL", link=name,
                                                 input_triangles=sum(len(p.faces) for p in pieces), target_triangles=budget)), encoding="utf-8", newline="\n")
        mesh = merged_visual(pieces, budget)
        expected = bounds_union(expected_bounds[name])
        error = float(np.max(abs(mesh.bounds-expected)))
        assert error <= 0.002, ("Visual differs from precise CAD bounds", name, error, mesh.bounds, expected)
        if name == "aircraft_canonical":
            relative = "meshes/aircraft/qav250_visual.stl"
            colour = [0.18, 0.19, 0.20, 1]
        else:
            relative = "meshes/visual/" + name + ".stl"
            colour = palette(name)
            links[name]["visuals"] = [dict(type="mesh", file=relative, xyz_m=[0, 0, 0], rpy_rad=[0, 0, 0], rgba=colour)]
            if name == "base_link":
                links[name]["collisions"] = [box_from_bounds(expected, "Conservative merged crawler body bounds; track/drive/contact links excluded")]
            elif name in collisions:
                links[name]["collisions"] = collisions[name]
        mesh.visual.face_colors = np.round(np.asarray(colour)*255).astype(np.uint8)
        export_labelled_stl(mesh, PACKAGE / relative)
        all_meshes[name] = mesh
        mesh_audit.append(dict(file=relative, link=name, triangles=len(mesh.faces), cad_bounds_m=expected.tolist(),
                               export_bounds_m=mesh.bounds.tolist(), max_bounds_error_m=error,
                               source="Native precise assembled body bounds union in link-local frame; envelope-constrained QEM with retained original extreme faces; no rescale. Bounds check does not qualify small-feature fidelity."))
        print(json.dumps(dict(link=name, triangles=len(mesh.faces), max_bounds_error_m=error)), flush=True)
        # Releasing these lists keeps completed dense component copies out of RAM.
        pieces.clear()
    for name, link in links.items():
        if "proxy_visual" not in link:
            continue
        p = link["proxy_visual"]
        mesh = trimesh.creation.cylinder(radius=p["radius_m"], height=p["length_m"], sections=32)
        mesh.apply_transform(trimesh.transformations.euler_matrix(*p["rpy_rad"]))
        relative = "meshes/visual/" + name + ".stl"
        colour = palette(name)
        mesh.visual.face_colors = np.round(np.asarray(colour)*255).astype(np.uint8)
        export_labelled_stl(mesh, PACKAGE / relative)
        link["visuals"] = [dict(type="mesh", file=relative, xyz_m=[0, 0, 0], rpy_rad=[0, 0, 0], rgba=colour)]
        all_meshes[name] = mesh
        mesh_audit.append(dict(file=relative, link=name, triangles=len(mesh.faces), export_bounds_m=mesh.bounds.tolist(),
                               source="REFERENCE_VISUAL_ONLY; dimension assumptions in running_gear_visual_proxies.json; no native CAD body or physical-part mass"))
    for link in data["static_aircraft_links"]:
        link["visuals"] = [dict(type="mesh", file="meshes/aircraft/qav250_visual.stl", xyz_m=[0, 0, 0], rpy_rad=[0, 0, 0], rgba=[0.18, 0.19, 0.20, 1])]
    scene = trimesh.Scene(base_frame="world")
    joint_by_child = {j["child"]: j for j in data["joints"] + data["static_aircraft_joints"]}
    for link in data["links"] + data["static_aircraft_links"]:
        name = link["name"]
        joint = joint_by_child.get(name)
        parent = joint["parent"] if joint else "world"
        matrix = translation(joint["xyz_m"]) if joint else np.eye(4)
        if name.endswith("_static_aircraft"):
            geometry_name = "qav250_visual"
            if geometry_name not in scene.geometry:
                scene.geometry[geometry_name] = all_meshes["aircraft_canonical"]
        elif name in all_meshes:
            geometry_name = name + "__mesh"
            scene.geometry[geometry_name] = all_meshes[name]
        else:
            geometry_name = None
        arguments = dict(frame_to=name, frame_from=parent, matrix=matrix)
        if geometry_name:
            arguments["geometry"] = geometry_name
        scene.graph.update(**arguments)
    scene.metadata.update(label=GRADE, aircraft_mode="static_visual", frame_convention="X forward/Y left/Z up; metres",
                          simplification="Reference road/idler geometry and unverified mass assumptions documented in package config; no physical qualification.")
    web = PACKAGE / "meshes/web/tins_ugv_carousel.glb"
    web.parent.mkdir(parents=True, exist_ok=True)
    web.write_bytes(scene.export(file_type="glb"))
    collision_data = dict(label=GRADE, geometry={name:link["collisions"] for name,link in links.items()},
                          policy="Primitives only; no raw part mesh collisions. Hidden wheels carry track contact. Fixed station pins/screws/rotors and reference wheel visuals have no contact geometry. Internal self-contact is not a qualified mechanism test.")
    collision_directory = PACKAGE / "meshes/collision"
    collision_directory.mkdir(parents=True, exist_ok=True)
    (collision_directory / "collision_geometry.json").write_text(json.dumps(collision_data, indent=2), encoding="utf-8", newline="\n")
    (collision_directory / "README.md").write_text(GRADE + "\n\nCollision geometry consists of URDF primitives listed in collision_geometry.json.\nNo raw part-level collision meshes are used. Tray notches are retained in the\nplate box decomposition. These approximations need Gazebo contact validation.\n", encoding="utf-8", newline="\n")
    data["mesh_audit"] = mesh_audit
    data["glb_link_nodes"] = [link["name"] for link in data["links"] + data["static_aircraft_links"]]
    vehicle_triangles = sum(len(mesh.faces) for name,mesh in all_meshes.items() if name != "aircraft_canonical")
    aircraft_triangles = len(all_meshes["aircraft_canonical"].faces)
    data["visual_triangle_counts"] = dict(vehicle=vehicle_triangles, one_aircraft=aircraft_triangles,
                                          rendered_with_eight_aircraft=vehicle_triangles+8*aircraft_triangles)
    data["status"] = "VALIDATED_EXPORT_INPUTS"
    # The source register hashes are enforced again by the XML generator.
    target = PACKAGE / "config/link_joint_sim_map.json"
    target.write_text(json.dumps(data, indent=2), encoding="utf-8", newline="\n")
    report = dict(label=GRADE, status="LINK_LOCAL_MESH_EXPORT_COMPLETE_DESCRIPTION_VALIDATION_PENDING",
                  mesh_files=len(mesh_audit), max_CAD_bounds_error_m=max(x.get("max_bounds_error_m",0) for x in mesh_audit),
                  visual_triangle_counts=data["visual_triangle_counts"], glb_nodes=len(data["glb_link_nodes"]),
                  glb_sha256=hashlib.sha256(web.read_bytes()).hexdigest(), collision_primitives=sum(len(x["collisions"]) for x in links.values()),
                  visual_LOD_policy="Envelope-constrained QEM; original extrema faces retained. No bounding-box rescale. Small-feature/surface error is not qualified.",
                  decimation_operations=DECIMATION_AUDIT)
    (O / "evidence/link-mesh-export-report.json").write_text(json.dumps(report, indent=2), encoding="utf-8", newline="\n")
    progress_path.write_text(json.dumps(report), encoding="utf-8", newline="\n")
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
