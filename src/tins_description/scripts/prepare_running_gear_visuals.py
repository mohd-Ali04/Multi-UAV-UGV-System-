#!/usr/bin/env python3
"""Document measured gear axes and missing-gear display proxies; no CAD fitting."""
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from native_export import GRADE, read_json, native_matrix_to_si, translation
from register_io import source_record

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
R = REPO / "Simulation/mechanical_design/tins12e"


def main():
    surface_path = R / "evidence/mechanism-surfaces.json"
    manifest_path = R / "integration/sim_grade/cad_extract_utf8/manifest.json"
    path_file = PACKAGE / "config/track_path.yaml"
    surfaces, manifest = map(read_json, [surface_path, manifest_path])
    track = yaml.safe_load(path_file.read_text(encoding="utf-8"))
    old = next(o for o in surfaces["occurrences"] if o["path"].endswith("+ÂÖ110:1"))
    cylinder = max((s for s in old["surfaces"] if s["type"] == "cylinder"), key=lambda s: s["radius"])
    old_matrix = np.asarray(old["transform"]).reshape(4, 4)
    # Move the measured cylinder axis into definition-local space, then into the
    # current corrected combined assembly. No old-world alignment is reused.
    local_point = np.linalg.inv(old_matrix) @ np.r_[cylinder["origin"], 1]
    local_axis = old_matrix[:3, :3].T @ cylinder["axis"]
    current = next(o for o in manifest["occurrences"] if o["component_name"] == "ÂÖ110")
    matrix = translation([0, 0, 0.6035]) @ native_matrix_to_si(current["matrix_cm"])
    point = (matrix @ np.r_[local_point[:3]*0.01, 1])[:3]
    axis = matrix[:3, :3] @ local_axis
    assert abs(abs(axis[1])-1) < 1e-8
    body = next(b for b in manifest["body_definitions"] if b["body_id"] in current["body_ids"])
    # Its measured wheel width is 30 mm, with this cylinder origin on one side.
    extent_native_m = (np.asarray(body["bounds_cm"]["max"]) - body["bounds_cm"]["min"]) * 0.01
    width = float(np.dot(np.abs(local_axis), extent_native_m))
    assert abs(width-0.03) < 1e-8
    centre = point + axis * width/2
    assert np.max(abs(centre - [0.0845, -0.3245, 0.3265])) < 1e-6
    actual = dict(name="right_upper_carrier_wheel_link", component="ÂÖ110",
                  source_path=current["path"], body_ids=current["body_ids"],
                  parent="base_link", xyz_m=centre.tolist(), axis=[0, 1, 0],
                  radius_m=cylinder["radius"]*0.01, width_m=width,
                  policy="Measured outer cylinder axis for rotating visual link. Source shaft/bearing topology remains incompletely documented; no suspension articulation or hardware qualification inferred.")
    circles = track["circles"]
    lower = sorted([c for c in circles if abs(c["centre_xz_m"][1]-0.095)<1e-7], key=lambda c:c["centre_xz_m"][0])
    rear_upper = next(c for c in circles if abs(c["centre_xz_m"][1]-0.31)<1e-7)
    assert len(lower) == 2
    proxies = []
    for side, sign in [("left", 1), ("right", -1)]:
        for index, x in enumerate(np.linspace(lower[0]["centre_xz_m"][0], lower[1]["centre_xz_m"][0], 6), 1):
            proxies.append(dict(name=f"{side}_road_wheel_{index}", parent="base_link",
                                xyz_m=[float(x), sign*0.325, 0.095], axis=[0, 1, 0],
                                radius_m=0.065, width_m=0.10,
                                dimension_basis="Six lower stations per human input; equal spacing between measured bottom corner arc centres is assumed. Radius65mm fits measured inner track arcs; width100mm assumed. Not OEM wheel dimensions or manufactured placement."))
        proxies.append(dict(name=f"{side}_rear_idler_visual", parent="base_link",
                            xyz_m=[*rear_upper["centre_xz_m"][:1], sign*0.325, rear_upper["centre_xz_m"][1]],
                            axis=[0, 1, 0], radius_m=0.065, width_m=0.10,
                            dimension_basis="Centre/radius inferred as a visual fit to the measured rear-upper inner track arc; actual tensioner/axle mechanism unknown."))
    proxies.append(dict(name="left_upper_carrier_visual", parent="base_link",
                        xyz_m=[centre[0], -centre[1], centre[2]], axis=[0, 1, 0],
                        radius_m=0.055, width_m=0.03,
                        dimension_basis="Mirrored display envelope of the supplied right110mm carrier wheel; a matching left physical part is not supplied/verified."))
    for proxy in proxies:
        proxy.update(label="REFERENCE_VISUAL_ONLY", physical_geometry_mass_kg=0,
                     included_in_BOM=False, collision=False,
                     physics_policy="EXCLUDED_REFERENCE_VISUAL_ONLY; Ubuntu overlay handles visualization without physical zero-mass moving bodies.",
                     actuation="Visual phase from measured track speed; no independent powered axis or suspension motion.")
    assert len(proxies) == 15
    data = dict(label=GRADE, actual_rotating_geometry=[actual], visual_proxies=proxies,
                hidden_contact_wheel_names=track["hidden_contact_wheels"],
                simplification="Two faithful source sprockets and complete source tracks retained. One supplied110mm carrier axis measured. Twelve lower wheels, two rear idlers and the missing left carrier are explicitly generic display proxies, not finished crawler running gear. CA-22P fitting remains deferred.",
                total_proxy_physical_mass_kg=0,
                tread_speed_policy="Use measured hidden contact-wheel angular speed ×0.13m for belt speed. Visual wheel speed uses its recorded radius. Source drive-sprocket pitch radius remains unverified;0.13m is a display speed reference, not a measured pitch radius. Vehicle pose remains Gazebo-measured.",
                source_registers=[source_record(p, REPO) for p in [surface_path, manifest_path, path_file]])
    target = PACKAGE / "config/running_gear_visual_proxies.json"
    target.write_text(json.dumps(data, indent=2), encoding="utf-8", newline="\n")
    print(json.dumps(dict(output=str(target), actual_carrier_axis=actual, reference_visual_proxies=len(proxies), new_physical_part_mass_kg=0)))


if __name__ == "__main__":
    main()
