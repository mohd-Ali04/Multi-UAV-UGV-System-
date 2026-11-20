#!/usr/bin/env python3
"""Report explicit simulation mass assumptions and their derived CG/inertia cases."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from native_export import GRADE, read_json, aggregate_inertials, box_inertial
from register_io import HASH_POLICY, source_sha256

PACKAGE = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=PACKAGE / "config/link_joint_sim_map.json")
    args = parser.parse_args()
    data = read_json(args.map)
    waypoints = yaml.safe_load((PACKAGE / "config/guarded_exchange_waypoints.yaml").read_text(encoding="utf-8"))
    aircraft = yaml.safe_load((PACKAGE / "config/aircraft_qav250.yaml").read_text(encoding="utf-8"))
    links = {link["name"]: link for link in data["links"]}
    children = {}
    for joint in data["joints"]:
        children.setdefault(joint["parent"], []).append(joint)

    def origins(positions):
        values = {"base_link": np.zeros(3)}

        def visit(parent):
            for joint in children.get(parent, []):
                offset = np.asarray(joint["xyz_m"], dtype=float)
                if joint["type"] == "prismatic":
                    offset += np.asarray(joint["axis"]) * positions[joint["name"]]
                assert joint["rpy_rad"] == [0, 0, 0]
                values[joint["child"]] = values[parent] + offset
                visit(joint["child"])

        visit("base_link")
        assert len(values) == len(links)
        return values

    def physical_parts(pose, only=None):
        output = []
        for name, link in links.items():
            if "inertial" not in link or only is not None and name not in only:
                continue
            d = link["inertial"]
            t = d["inertia_at_cg_kg_m2"]
            tensor = np.array([[t["ixx"], t["ixy"], t["ixz"]], [t["ixy"], t["iyy"], t["iyz"]], [t["ixz"], t["iyz"], t["izz"]]])
            output.append(dict(mass_kg=d["mass_kg"], cg_m=pose[name]+d["cg_m"], tensor_kg_m2=tensor))
        return output

    park = origins(waypoints["waypoints"][0]["joint_positions_m"])
    baseline_names = set(data["crawler_reference_links"])
    crawler = aggregate_inertials(physical_parts(park, baseline_names))
    assert abs(crawler["mass_kg"] - 295) < 1e-8
    vehicle = aggregate_inertials(physical_parts(park))
    shifts = (np.array(vehicle["cg_m"]) - crawler["cg_m"]).tolist()
    cases = []
    for waypoint in waypoints["waypoints"]:
        pose = origins(waypoint["joint_positions_m"])
        parts = physical_parts(pose)
        empty = aggregate_inertials(parts)
        for case, indices, mass in [("vehicle_no_aircraft", [], 0),
                                   ("eight_catalogue_reference_aircraft", range(1, 9), aircraft["simulation_reference_mass_kg"]),
                                   ("only_tray_A_catalogue_reference_aircraft", [1, 3, 5, 7], aircraft["simulation_reference_mass_kg"]),
                                   ("only_tray_B_catalogue_reference_aircraft", [2, 4, 6, 8], aircraft["simulation_reference_mass_kg"]),
                                   ("eight_original_allocation_aircraft_LOAD_CASE_ONLY", range(1, 9), 2)]:
            additions = []
            for index in indices:
                # Aircraft centre is an explicit envelope-centre estimate, not a measured CG.
                centre = pose[f"uav{index:02d}_dock"] + [0, 0, aircraft["envelope"]["antenna_reduced_retained_solid_height_m"] / 2]
                size = [*aircraft["envelope"]["propeller_full_sweep_plan_m"], aircraft["envelope"]["antenna_reduced_retained_solid_height_m"]]
                additions.append(box_inertial(mass, centre, size))
            total = aggregate_inertials(parts + additions) if additions else empty
            cases.append(dict(waypoint=waypoint["name"], waypoint_index=waypoint["index"], case=case,
                              **total, aircraft_mass_kg=len(additions)*mass))
    report = dict(label=GRADE, status="SIMULATION_REFERENCE_MASS_PROPERTIES_NOT_MEASURED_OR_QUALIFIED",
                  input_map_sha256=source_sha256(args.map), input_map_hash_policy=HASH_POLICY,
                  vehicle_park=vehicle, crawler_reference_park=crawler,
                  added_vehicle_mount_mechanism_and_lump_kg=vehicle["mass_kg"]-crawler["mass_kg"],
                  estimated_vehicle_CG_shift_from_crawler_reference_m=shifts,
                  motor_brake_reference_lump_kg=4.54, motor_brake_charges=1,
                  separate_aircraft_each_reference_kg=aircraft["simulation_reference_mass_kg"],
                  separate_fleet_reference_kg=4, unchanged_design_aircraft_allocation_kg=16,
                  aircraft_CG_inertia_basis="Uniform swept-envelope box centre/tensor used for load-case comparison only, not flight-model acceptance. Vehicle URDF contains none of this aircraft mass.",
                  assumptions=data["assumptions"], link_inertials=[dict(link=n, **v["inertial"]) for n, v in links.items() if "inertial" in v],
                  waypoint_load_cases=cases,
                  limitations="Exact crawler configuration, missing internal hardware distribution, battery placement and CG limits are unverified. Numbers are model-derived estimates, not a measured robot or structural/stability qualification. Physical mass budget closure remains deferred.")
    target = PACKAGE / "config/mass_inertia_report.json"
    target.write_text(json.dumps(report, indent=2), encoding="utf-8", newline="\n")
    with (PACKAGE / "config/link_inertials.csv").open("w", encoding="utf-8", newline="\n") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["label", "link", "mass_kg", "cg_x_m", "cg_y_m", "cg_z_m", "ixx", "iyy", "izz", "ixy", "ixz", "iyz"])
        for row in report["link_inertials"]:
            writer.writerow([GRADE, row["link"], row["mass_kg"], *row["cg_m"], *[row["inertia_at_cg_kg_m2"][k] for k in ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]]])
    text = f'''{GRADE}

Simulation reference values, not a weighed or physically qualified assembly.

Vehicle without aircraft: **{vehicle['mass_kg']:.6f} kg**. This consists of the
unverified 295 kg crawler/battery catalogue baseline and {vehicle['mass_kg']-295:.6f} kg
of current mounting/mechanism estimates, including the single 4.54 kg motor/brake lump.
Park CG in base_link: {vehicle['cg_m']} m. Estimated shift from the assumed crawler
baseline CG: {shifts} m. See physical_mass_rules.json for material, catalogue,
missing-internal-mass and battery-distribution assumptions.

The eight separate aircraft use a 0.5 kg catalogue reference each (4 kg total),
compared with the unchanged 16 kg design allocation. Their exact supplied-build
mass and CG are unmeasured. They add no mass to the vehicle URDF. Static display
aircraft are massless. The 16 kg alternative remains a load case, not a mass credit.

mass_inertia_report.json contains every physical link tensor and 130 load cases
(26 waypoint states × five occupancy/allocation cases). Aircraft box estimates
are for this load comparison only. No OEM CG-limit, stability, strength, locking,
braking, power-loss or flight check has passed on the basis of this calculation.
'''
    (PACKAGE / "config/mass_inertia_report.md").write_text(text, encoding="utf-8", newline="\n")
    print(json.dumps(dict(status=report["status"], vehicle_park=vehicle,
                          estimated_CG_shift_m=shifts, load_cases=len(cases), physical_links=len(report["link_inertials"]), output=str(target))))


if __name__ == "__main__":
    main()
