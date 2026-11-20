#!/usr/bin/env python3
"""Validate generated tree, source hashes, inertials, physical ledger and CAD bounds.

Writes a new timestamped evidence file on failure; never removes failed evidence.
Use after build_simulation_map.py, export_link_meshes.py and the XML generator.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import traceback
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import xacro
from yourdfpy import URDF
from register_io import verify_source

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
GRADE = "SIMULATION_GRADE_NOT_PHYSICALLY_QUALIFIED"


def check(condition, message):
    if not condition:
        raise ValueError(message)


def parse_vec(text, default):
    return np.asarray([float(x) for x in text.split()] if text else default)


def resolved_mesh(uri):
    prefix = "package://tins_description/"
    check(uri.startswith(prefix), f"Unexpected mesh URI: {uri}")
    path = (PACKAGE / uri[len(prefix):]).resolve()
    check(path.is_relative_to(PACKAGE), f"Mesh escapes package: {path}")
    check(path.is_file(), f"Missing mesh: {path}")
    check(not path.read_bytes()[:80].startswith(b"version https://git-lfs"), f"LFS pointer, not mesh: {path}")
    return path


def validate_variant(path, expected, static):
    root = ET.parse(path).getroot()
    links = root.findall("link")
    joints = root.findall("joint")
    names = [x.attrib["name"] for x in links]
    joint_names = [x.attrib["name"] for x in joints]
    check(len(names) == len(set(names)), "Duplicate link names")
    check(len(joint_names) == len(set(joint_names)), "Duplicate joint names")
    check(len(joints) == len(links) - 1, "Tree must have N-1 joints")
    child_counts = Counter(x.find("child").attrib["link"] for x in joints)
    check(all(value == 1 for value in child_counts.values()), "Link has multiple parents")
    check(set(names) - set(child_counts) == {"base_link"}, "Wrong tree root")
    children = {name: [] for name in names}
    for joint in joints:
        parent, child = [joint.find(x).attrib["link"] for x in ["parent", "child"]]
        check(parent in children and child in children, "Unknown parent/child")
        children[parent].append(child)
        if joint.attrib["type"] != "fixed":
            limits = joint.find("limit")
            check(limits is not None, f"Missing limits: {joint.attrib['name']}")
            values = {key: float(value) for key, value in limits.attrib.items()}
            check(values["effort"] > 0 and values["velocity"] > 0, "Invalid effort/velocity bound")
            if joint.attrib["type"] != "continuous":
                check(values["lower"] < values["upper"], "Invalid position limits")
            axis = parse_vec(joint.find("axis").attrib["xyz"], [0, 0, 0])
            check(abs(np.linalg.norm(axis) - 1) < 1e-9, "Joint axis is not unit length")
    seen = set()

    def visit(name):
        check(name not in seen, "Joint graph has a cycle")
        seen.add(name)
        for child in children[name]:
            visit(child)

    visit("base_link")
    check(seen == set(names), "Disconnected links")
    mass = 0
    checked_inertias = 0
    for link in links:
        inertial = link.find("inertial")
        if inertial is not None:
            m = float(inertial.find("mass").attrib["value"])
            check(np.isfinite(m) and m > 0, "Nonpositive physical mass")
            values = {k: float(v) for k, v in inertial.find("inertia").attrib.items()}
            tensor = np.array([[values["ixx"], values["ixy"], values["ixz"]],
                               [values["ixy"], values["iyy"], values["iyz"]],
                               [values["ixz"], values["iyz"], values["izz"]]])
            check(np.isfinite(tensor).all(), "Nonfinite inertia")
            eig = np.linalg.eigvalsh(tensor)
            check(eig.min() > 0, f"Inertia is not positive definite: {link.attrib['name']}")
            check(eig[2] <= eig[:2].sum() + 1e-8, f"Inertia violates triangle inequality: {link.attrib['name']}")
            mass += m
            checked_inertias += 1
        if "_static_aircraft" in link.attrib["name"]:
            check(static and inertial is None and link.find("collision") is None,
                  "Display aircraft must not add vehicle mass/collisions")
    check(abs(mass - expected["vehicle_mass_kg"]) < 1e-7, "Vehicle mass differs from reconciled ledger")
    for mesh in root.findall(".//mesh"):
        resolved_mesh(mesh.attrib["filename"])
        check(np.array_equal(parse_vec(mesh.get("scale"), [1, 1, 1]), [1, 1, 1]), "Mesh scale must be 1")
    xml = ET.tostring(root, encoding="unicode")
    check("CAD_KEEP_OUT_" not in xml, "CAD keep-out included in robot description")
    # Independent parser validates joint semantics and resolves the actual mesh assets.
    robot = URDF.load(str(path), filename_handler=lambda fname: str(resolved_mesh(fname)),
                      load_meshes=True, build_scene_graph=True, load_collision_meshes=True)
    check(len(robot.robot.links) == len(links), "Independent URDF parser link count mismatch")
    return {"file": path.relative_to(PACKAGE).as_posix(), "links": len(links),
            "joints": len(joints), "inertials": checked_inertias, "mass_kg": mass}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=PACKAGE / "validation_report.json")
    args = parser.parse_args()
    report = {"label": GRADE, "status": "RUNNING", "checked_utc": datetime.now(timezone.utc).isoformat(),
              "runtime_scope": "Offline generated-description validation; no Gazebo, physical, flight or manufacturing acceptance."}
    try:
        map_path = PACKAGE / "config/link_joint_sim_map.json"
        data = json.loads(map_path.read_text(encoding="utf-8"))
        check(data["status"] == "VALIDATED_EXPORT_INPUTS", "Simulation map is unfinished")
        for source in data["source_registers"]:
            verify_source(source, REPO)
        for link in data["links"]:
            if link.get("role") == "REFERENCE_VISUAL_ONLY":
                check(not link.get("inertial") and not link.get("collisions"),
                      "Reference-only gear must have zero physical mass and no contact")
        check(data["label"] == GRADE, "Missing simulation-grade label")
        check(len(data["excluded_nonphysical"]) == 2, "Expected both CAD keep-outs excluded")
        check(data["reconciliation"]["all_physical_instances_assigned_once"], "Native physical instance coverage incomplete")
        check(data["reconciliation"]["motor_brake_reference_charges"] == 1, "Motor/brake reference mass double counted")
        check(data["visual_triangle_counts"]["rendered_with_eight_aircraft"] <= 310000,
              "Rendered visual triangle count exceeds approximately300k handoff target")
        bounds_results = []
        for record in data["mesh_audit"]:
            path = PACKAGE / record["file"]
            check(path.is_file(), f"Missing exported asset: {path}")
            mesh = trimesh.load_mesh(path, process=False)
            check(np.isfinite(mesh.vertices).all() and len(mesh.faces) > 0, f"Invalid mesh: {path}")
            if "cad_bounds_m" in record:
                error = float(np.max(np.abs(mesh.bounds - np.asarray(record["cad_bounds_m"]))))
                check(error <= 0.002, f"Mesh exceeds CAD bounds tolerance: {path}: {error}m")
                bounds_results.append({"file": record["file"], "max_bounds_error_m": error})
        variants = [validate_variant(PACKAGE / "urdf" / name, data["reconciliation"], static)
                    for name, static in [("tins_ugv_carousel.urdf", False),
                                         ("tins_ugv_carousel_static_visual.urdf", True)]]
        # A changed lump parameter must affect the mass by exactly the parameter delta.
        doc = xacro.process_file(str(PACKAGE / "urdf/tins_ugv_carousel.urdf.xacro"),
                                 mappings={"aircraft_mode": "none", "motor_brake_lumped_mass_kg": "5.54"})
        masses = [float(x.attrib["value"]) for x in ET.fromstring(doc.toxml()).findall(".//inertial/mass")]
        check(abs(sum(masses) - data["reconciliation"]["vehicle_mass_kg"] - 1) < 1e-7,
              "Lumped-mass parameter does not change total by exactly one kilogram")
        glb = trimesh.load(PACKAGE / "meshes/web/tins_ugv_carousel.glb", force="scene", process=False)
        expected_nodes = set(data["glb_link_nodes"])
        check(set(glb.graph.nodes) - {"world"} == expected_nodes, "GLB node names differ from registered links")
        check(not any("CAD_KEEP_OUT_" in n for n in glb.graph.nodes), "CAD keep-out in GLB")
        report.update(status="PASS_OFFLINE_DESCRIPTION_VALIDATION", variants=variants,
                      mesh_bounds=bounds_results, glb_nodes=len(expected_nodes),
                      motor_lump_parameter_delta_kg=1.0,
                      source_registers=data["source_registers"],
                      reconciliation=data["reconciliation"],
                      assumptions=data["assumptions"])
    except Exception:
        report.update(status="FAIL_OFFLINE_DESCRIPTION_VALIDATION", error=traceback.format_exc())
        failure = PACKAGE / ("validation_failure_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
        failure.write_text(json.dumps(report, indent=2), encoding="utf-8", newline="\n")
        print(json.dumps({"status": report["status"], "evidence": str(failure), "error": report["error"]}))
        raise SystemExit(1)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8", newline="\n")
    print(json.dumps({"status": report["status"], "output": str(args.output), "variants": variants,
                      "meshes_checked": len(bounds_results), "glb_nodes": len(expected_nodes)}))


if __name__ == "__main__":
    main()
