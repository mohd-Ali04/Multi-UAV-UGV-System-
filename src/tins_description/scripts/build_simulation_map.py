#!/usr/bin/env python3
"""Flatten the preserved native assembly into an auditable SI simulation input map.

This script reads extracted geometry metadata and the physical registers. It never
edits Fusion. Mesh export/validation is a subsequent step, so this draft map cannot
yet be passed to the URDF generator.
"""
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import re

import numpy as np
import yaml

from native_export import (GRADE, read_json, native_matrix_to_si, translation,
                           body_inertial, aggregate_inertials, box_inertial)
from register_io import source_record

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
R = REPO / "Simulation/mechanical_design/tins12e"
O = R / "integration/sim_grade"


def main():
    manifest_path = O / "cad_extract_utf8/manifest.json"
    manifest = read_json(manifest_path)
    assert manifest["status"] == "COMPLETE_READ_ONLY_CAD_EXTRACTION"
    assert manifest["physical_body_definitions"] == 333 and manifest["physical_body_instances"] == 1524
    assert manifest["max_matrix_error"] < 1e-6
    audit_path = O / "evidence/post-extraction-geometry-audit.json"
    audit = read_json(audit_path)
    assert audit["status"] == "PASS_POST_EXTRACTION_GEOMETRY_AND_RELATION_AUDIT"
    assert audit["max_body_bounds_error_cm"] == 0 and audit["max_matrix_error"] == 0
    physical_path = R / "carousel/double_level_station_locks/link-joint-frame-register.json"
    motion_path = R / "carousel/double_level_motion_control/motion-frame-register.json"
    ledger_path = R / "carousel/double_level_motion_control/physical-mass-ledger.json"
    rules_path = PACKAGE / "config/physical_mass_rules.json"
    crawler_path = R / "registers/link-joint-frame-register.json"
    inventory_path = R / "registers/crawler-part-inventory.json"
    motor_path = R / "carousel/double_level_motion_control/evidence/motor-units-construction.json"
    station_path = PACKAGE / "config/station_frames.yaml"
    waypoint_path = PACKAGE / "config/guarded_exchange_waypoints.yaml"
    running_gear_path = PACKAGE / "config/running_gear_visual_proxies.json"
    running_gear = read_json(running_gear_path)
    physical, rules, crawler, inventory, motor = map(read_json, [physical_path, rules_path, crawler_path, inventory_path, motor_path])
    stations = yaml.safe_load(station_path.read_text(encoding="utf-8"))
    waypoints = yaml.safe_load(waypoint_path.read_text(encoding="utf-8"))
    source_paths = [physical_path, motion_path, ledger_path, rules_path, crawler_path,
                    inventory_path, motor_path, station_path, waypoint_path, manifest_path, audit_path,
                    running_gear_path, PACKAGE / "config/track_path.yaml"]
    links, joints, origins = {}, [], {}

    def add(name, parent=None, xyz=None, kind="fixed", axis=None, limits=None, **metadata):
        assert name not in links, name
        xyz = list(xyz or [0, 0, 0])
        links[name] = dict(name=name, visuals=[], collisions=[], **metadata)
        origins[name] = (np.asarray(origins[parent]) + xyz).tolist() if parent else xyz
        if parent:
            record = dict(name=name + "_fixed_joint", parent=parent, child=name, xyz_m=xyz,
                          rpy_rad=[0, 0, 0], type=kind, **metadata)
            if axis is not None:
                record["axis"] = axis
            if limits is not None:
                record["limits"] = limits
            joints.append(record)

    add("base_link", role="crawler_chassis_and_internal_mass_estimates")
    add("carousel_mount", "base_link", [0, 0, 0.6035], role="fixed_frame_and_simple_mount")
    for record in waypoints["joints"].values():
        lo, hi = record["limits_m"]
        add(record["child"], record["parent"], record["origin_xyz_m"], "prismatic", record["axis"],
            {"lower": lo, "upper": hi, "effort": 1000, "velocity": 0.02},
            limit_status="POSITION_LIMITS_MEASURED; effort/velocity provisional simulation caps, not ratings")
        joints[-1]["name"] = next(k for k, v in waypoints["joints"].items() if v is record)
    for record in crawler["joints"]:
        limits = {"effort": 500, "velocity": 15} if record["type_urdf"] == "continuous" else None
        add(record["child"], record["parent"], record["origin_xyz_m"], record["type_urdf"],
            record.get("axis"), limits, role="source_running_gear_visual",
            limit_status="SIMULATION_CAPS_NOT_VERIFIED_MOTOR_RATINGS",
            physical_source_status=record["status"])
        joints[-1]["name"] = record["name"]
        if "fusion_rotation_value_to_urdf_angle" in record:
            joints[-1]["fusion_rotation_value_to_urdf_angle"] = record["fusion_rotation_value_to_urdf_angle"]
    for record in physical["screw_branches"]:
        add(record["child"], record["parent"], record["origin_xyz_m"],
            physical_type=record["type"], v1_policy="FIXED_SAVED_PARK_VISUAL; no mimic or independent screw actuation")
        joints[-1]["name"] = record["joint"]
    for record in physical["station_lock_frames_and_joints"]:
        add(record["housing_frame"], record["housing_fixed_parent"], record["housing_origin_xyz_m"], role="massless_axis_datum")
        pin = record["pin_joint"]
        add(pin["child"], record["housing_frame"], physical_type=pin["type"],
            v1_policy="FIXED_SAVED_PARK_VISUAL; saved pin displacement is baked into its link-local mesh, no receiver closure")
        joints[-1]["name"] = pin["name"]
        joints[-1]["saved_physical_q_m"] = waypoints["waypoints"][0]["cad_pin_positions_m"][record["id"]]
        joints[-1]["physical_axis"] = pin["axis"]
    owner_alias = {"base": "carousel_mount", "A_x": "tray_a_x_carriage", "B_x": "tray_b_x_carriage"}
    motor_links = {}
    for record in motor["parts"]:
        name = "motor_" + record["id"].lower() + "_rotor_visual"
        parent = owner_alias[record["owner"]]
        world = np.asarray(record["rotation_origin_world_cm"]) * 0.01 + [0, 0, 0.6035]
        add(name, parent, (world - origins[parent]).tolist(), role="motor_rotor_visual_only",
            physical_type="continuous", v1_policy="FIXED_SAVED_PARK_VISUAL; no inertial double counting")
        joints[-1]["name"] = record["rotor_joint"]
        motor_links[record["id"]] = name
    add("motor_brake_reference_lump", "carousel_mount", role="ONE_PARAMETERIZED_4_54_KG_REFERENCE_LUMP",
        mass_parameter="motor_brake_lumped_mass_kg")
    for record in crawler["frames"]:
        if record["name"] not in links:
            add(record["name"], record["parent"], record["xyz_m"], reference_status=record["status"])
    for record in stations["frames"]:
        add(record["name"], record["parent"], record["xyz_m"], role=record["role"])
    hidden_contacts = []
    for side, sign in [("left", 1), ("right", -1)]:
        for index, x in enumerate([-0.35, 0, 0.35], 1):
            name = f"{side}_contact_wheel_{index}"
            add(name, "base_link", [x, sign * 0.325, 0.13], "continuous", [0, 1, 0],
                {"effort": 500, "velocity": 15}, role="HIDDEN_SIMULATION_CONTACT_PROXY_NOT_OEM_WHEEL",
                limit_status="SIMULATION_CAPS_NOT_VERIFIED_MOTOR_RATINGS")
            joints[-1]["name"] = name + "_joint"
            links[name]["collisions"] = [dict(type="cylinder", radius_m=0.13, length_m=0.12,
                                             xyz_m=[0, 0, 0], rpy_rad=[np.pi/2, 0, 0])]
            hidden_contacts.append(name)
    actual_wheel_owners = {}
    for record in running_gear["actual_rotating_geometry"]:
        add(record["name"], record["parent"], record["xyz_m"], "continuous", record["axis"],
            {"effort": 1, "velocity": 50}, role="SOURCE_WHEEL_ROTATING_VISUAL",
            physical_source_status=record["policy"])
        joints[-1]["name"] = record["name"] + "_joint"
        actual_wheel_owners[record["component"]] = record["name"]
    for record in running_gear["visual_proxies"]:
        add(record["name"], record["parent"], record["xyz_m"], "continuous", record["axis"],
            {"effort": 1, "velocity": 50}, role="REFERENCE_VISUAL_ONLY",
            physical_geometry_mass_kg=0, included_in_BOM=False,
            proxy_visual=dict(type="cylinder", radius_m=record["radius_m"], length_m=record["width_m"],
                              xyz_m=[0, 0, 0], rpy_rad=[np.pi/2, 0, 0]),
            dimension_basis=record["dimension_basis"])
        joints[-1]["name"] = record["name"] + "_joint"
    body_defs = {b["body_id"]: b for b in manifest["body_definitions"]}
    rules_by_name = {row["component"]: row for row in rules["component_rules"]}
    definition_volumes = defaultdict(float)
    for body in body_defs.values():
        if not body["properties"].get("unavailable"):
            definition_volumes[body["component"]] += body["properties"]["volume_cm3"]
    crawler_roles = {row["source_name"]: row["role"] for row in inventory["definitions"]}
    inertials, motor_shapes, instances = defaultdict(list), defaultdict(list), []
    crawler_material_mass = 0
    native_to_base = translation([0, 0, 0.6035])
    carousel_prefixes = {"Tray_A_PROVISIONAL:1": "tray_a", "Tray_B_PROVISIONAL:1": "tray_b",
                         "LINK_Tray_A_X_Carriage_REFERENCE:1": "tray_a_x_carriage",
                         "LINK_Tray_B_X_Carriage_REFERENCE:1": "tray_b_x_carriage",
                         "REFERENCE_ExchangeFixedBase_UnratedMount:1": "carousel_mount"}
    for occurrence in manifest["occurrences"]:
        if occurrence["excluded_nonphysical"]:
            assert not occurrence["body_ids"]
            continue
        path, component = occurrence["path"], occurrence["component_name"]
        world = native_to_base @ native_matrix_to_si(occurrence["matrix_cm"])
        aircraft_match = re.search(r"UAV(\d\d)_DockDatum_GEOMETRIC", path)
        root_owner = carousel_prefixes.get(path.split("+")[0])
        for body_id in occurrence["body_ids"]:
            definition = body_defs[body_id]
            mass, method = 0, "SEPARATE_AIRCRAFT_NO_VEHICLE_MASS"
            motor_unit = re.search(r"MOTOR_UNIT_(X_A|X_B|Z_A|Z_B)_", path)
            if aircraft_match:
                owner = "uav" + aircraft_match.group(1) + "_static_aircraft"
            elif path.startswith("TinS12E_Crawler_SimGrade_Copy:"):
                role = crawler_roles[component]
                centre = (world @ np.r_[np.asarray(definition["bounds_cm"]["min"]) * 0.005 + np.asarray(definition["bounds_cm"]["max"]) * 0.005, 1])[:3]
                side = "left" if centre[1] > 0 else "right"
                owner = side + "_drive_sprocket_link" if role == "drive_rotor" else side + "_track_proxy_link" if role == "track_solid_proxy" else "base_link"
                owner = actual_wheel_owners.get(component, owner)
                density = 1200 if role == "track_solid_proxy" else 7850
                mass = definition["properties"]["volume_cm3"] * 1e-6 * density
                method = f"CRAWLER_VOLUME_TIMES_ASSUMED_DENSITY_{density}_KG_M3; exact material unverified"
                crawler_material_mass += mass
            elif motor_unit:
                owner = motor_links[motor_unit.group(1)] if component == "MOTOR_SHARED_EXPOSED_SHAFT_VISUAL" else root_owner
                method = "MOTOR_REFERENCE_VISUAL; mass charged once in parameterized aggregate lump"
                lump_frame = np.linalg.inv(translation(origins["motor_brake_reference_lump"])) @ world
                motor_shapes[motor_unit.group(1)].append((definition, lump_frame))
            elif component in rules_by_name:
                rule = rules_by_name[component]
                owner = rule["physical_link"]
                if rule["method"] == "CATALOGUE_PART_MASS_ONCE":
                    mass = rule["mass_kg"] * definition["properties"]["volume_cm3"] / definition_volumes[component]
                else:
                    mass = definition["properties"]["volume_cm3"] * 1e-6 * rule["density_kg_m3"]
                method = rule["status"]
            elif component in rules["station_package_density_rules"]:
                owner = root_owner
                density = rules["station_package_density_rules"][component]
                mass = definition["properties"]["volume_cm3"] * 1e-6 * density
                method = f"STATION_PACKAGE_CURRENT_VOLUME_TIMES_ASSUMED_DENSITY_{density}_KG_M3"
            else:
                raise ValueError(f"Unassigned physical part: {path}, {body_id}, {component}")
            if not aircraft_match:
                assert owner in links, (component, owner)
                body_to_link = np.linalg.inv(translation(origins[owner])) @ world
                if mass > 0:
                    inertials[owner].append(body_inertial(definition, body_to_link, mass))
            else:
                dock_name = "uav" + aircraft_match.group(1) + "_dock"
                body_to_link = np.linalg.inv(translation(origins[dock_name])) @ world
            instances.append(dict(native_path=path, body_id=body_id, component=component,
                                  source_body=definition["body"], link=owner,
                                  body_to_link_m=body_to_link.tolist(), assigned_vehicle_mass_kg=mass,
                                  mass_basis=method, separate_aircraft=bool(aircraft_match)))
    assert len(instances) == 1524
    assert len({(x["native_path"], x["body_id"]) for x in instances}) == 1524
    contact_mass = 0.05
    for name in hidden_contacts:
        # Small positive simulation proxy inertial, charged inside the dry chassis reference once.
        m, radius, length = contact_mass, 0.13, 0.12
        tensor = np.diag([m*(3*radius*radius+length*length)/12, m*radius*radius/2,
                          m*(3*radius*radius+length*length)/12])
        inertials[name].append(dict(mass_kg=m, cg_m=np.zeros(3), tensor_kg_m2=tensor))
    # Deferred running gear is strictly a massless display reference. Ubuntu's
    # overlay must render these axes outside the physical dynamic bodies.
    missing_mass = rules["crawler"]["published_dry_chassis_reference_kg"] - crawler_material_mass - len(hidden_contacts)*contact_mass
    assert missing_mass > 0, "Retained material estimates exceed published dry reference; resolve before export"
    inertials["base_link"].append(box_inertial(missing_mass, rules["crawler"]["missing_internal_CG_m_assumed"], rules["crawler"]["missing_internal_box_size_m_assumed"]))
    inertials["base_link"].append(box_inertial(85, rules["crawler"]["battery_CG_m_assumed"], rules["crawler"]["battery_box_size_m_assumed"]))
    assert set(motor_shapes) == {"X_A", "X_B", "Z_A", "Z_B"}
    for unit, shapes in motor_shapes.items():
        volume = sum(b["properties"]["volume_cm3"] for b, _ in shapes)
        for body, matrix in shapes:
            mass = 4.54/4 * body["properties"]["volume_cm3"] / volume
            inertials["motor_brake_reference_lump"].append(body_inertial(body, matrix, mass))
    for name, parts in inertials.items():
        links[name]["inertial"] = aggregate_inertials(parts)
    vehicle_mass = sum(link.get("inertial", {}).get("mass_kg", 0) for link in links.values())
    expected = 295 + rules["expected_prior_carousel_without_aircraft_or_motor_lump_kg"] + rules["mount_estimate_kg"] + 4.54
    difference = vehicle_mass - expected
    assert abs(difference) < 1e-6, ("Mass ledger reconciliation failed", vehicle_mass, expected, difference)
    static_links, static_joints = [], []
    for index in range(1, 9):
        name = f"uav{index:02d}_static_aircraft"
        static_links.append(dict(name=name, visuals=[], collisions=[], role="MASSLESS_DISPLAY_ONLY"))
        static_joints.append(dict(name=name + "_fixed_joint", type="fixed", parent=f"uav{index:02d}_dock", child=name, xyz_m=[0, 0, 0], rpy_rad=[0, 0, 0]))
    sources = [source_record(p, REPO) for p in source_paths]
    data = dict(schema_version=1, label=GRADE, status="DRAFT_MESH_AND_TRACK_EXPORT_PENDING",
                native_cache=manifest_path.relative_to(REPO).as_posix(), source_registers=sources,
                units="m/kg/rad/kg*m^2", frames="X forward/Y left/Z up; base_link ground datum",
                links=list(links.values()), joints=joints, zero_frame_origins_in_base_m=origins,
                static_aircraft_links=static_links, static_aircraft_joints=static_joints,
                native_body_assignments=instances, excluded_nonphysical=manifest["excluded_nonphysical"],
                hidden_contact_wheels=hidden_contacts,
                crawler_reference_links=["base_link", "left_drive_sprocket_link", "right_drive_sprocket_link", "left_track_proxy_link", "right_track_proxy_link", *hidden_contacts, *actual_wheel_owners.values(), *[x["name"] for x in running_gear["visual_proxies"]]],
                mesh_audit=[], glb_link_nodes=[],
                reconciliation=dict(all_physical_instances_assigned_once=True,
                    native_physical_instances=1524, vehicle_mass_kg=vehicle_mass,
                    expected_vehicle_mass_kg=expected, difference_kg=difference,
                    crawler_reference_total_kg=295, crawler_material_estimate_kg=crawler_material_mass,
                    crawler_missing_internal_estimate_kg=missing_mass, hidden_contact_proxy_mass_kg=len(hidden_contacts)*contact_mass,
                    reference_visual_mass_kg=0,
                    motor_brake_reference_kg=4.54, motor_brake_reference_charges=1,
                    aircraft_in_vehicle_inertials_kg=0, separate_aircraft_fleet_reference_kg=4,
                    unchanged_aircraft_design_allocation_kg=16),
                assumptions=["SIMULATION_GRADE_NOT_PHYSICALLY_QUALIFIED; no physical mounting/strength/locking/braking/power-loss/flight acceptance.",
                    "Fusion reported the working copy modified after read-only extraction; cause unproven. Follow-up audit found all333 physical body bounds and1084 occurrence matrices unchanged and288 relationships valid. Original source archive hashes unchanged.",
                    "Published 210 kg dry crawler plus 85 kg battery is a parameterized reference, not a verified exact-configuration weight.",
                    "Retained crawler solids: assumed steel 7850 kg/m3 except reinforced-track surrogate 1200 kg/m3. Missing internal mass and battery CG/inertia use explicit uniform box estimates.",
                    "Catalogue component CG/inertia use homogeneous reference shapes, not manufacturer measured tensors.",
                    "One 4.54 kg fixed motor/brake lump uses aggregate park reference geometry; moving Z-motor mass redistribution during X travel is approximated.",
                    "Four slides active. Eight pins, four screws and four motor rotor visuals fixed at saved park; physical register joint types unchanged. No mimic or closed-loop claims.",
                    "Six hidden contact wheels carry physics; static CAD track solids carry visual geometry/inertials only. Contact proxy mass is deducted from crawler baseline once.",
                    "Fifteen missing running-gear visuals are REFERENCE_VISUAL_ONLY: zero physical mass, no inertial/collision/BOM entry. Ubuntu overlay must exclude these branches from dynamics and animate display geometry from measured track speed.",
                    "Simulation effort/velocity caps are provisional software inputs, not verified actuator ratings."])
    target = PACKAGE / "config/link_joint_sim_map.draft.json"
    target.write_text(json.dumps(data, indent=2), encoding="utf-8", newline="\n")
    print(json.dumps(dict(status=data["status"], links=len(links), joints=len(joints), native_instances=len(instances), reconciliation=data["reconciliation"], output=str(target))))


if __name__ == "__main__":
    main()
