#!/usr/bin/env python3
"""Generate robot XML from the audited simulation map; never edit generated XML.

The map records source-register hashes and reconciled physical link inertials.
This command has no Fusion API or construction operations.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import xacro
from register_io import HASH_POLICY, source_sha256, verify_source

GRADE = "SIMULATION_GRADE_NOT_PHYSICALLY_QUALIFIED"
NS = "http://www.ros.org/wiki/xacro"
ET.register_namespace("xacro", NS)
PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]


def numbers(values):
    return " ".join(format(float(value), ".12g") for value in values)


def element(parent, tag, **attributes):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attributes.items()})


def origin(parent, xyz=None, rpy=None):
    return element(parent, "origin", xyz=numbers(xyz or [0, 0, 0]),
                   rpy=numbers(rpy or [0, 0, 0]))


def geometry(parent, record):
    geo = element(parent, "geometry")
    shape = record["type"]
    if shape == "mesh":
        element(geo, "mesh", filename="package://tins_description/" + record["file"],
                scale="1 1 1")
    elif shape == "box":
        element(geo, "box", size=numbers(record["size_m"]))
    elif shape == "cylinder":
        element(geo, "cylinder", radius=record["radius_m"], length=record["length_m"])
    elif shape == "sphere":
        element(geo, "sphere", radius=record["radius_m"])
    else:
        raise ValueError(f"Unknown collision/visual shape: {shape}")


def make_link(parent, record):
    link = element(parent, "link", name=record["name"])
    if record.get("inertial"):
        data = record["inertial"]
        inertial = element(link, "inertial")
        origin(inertial, data["cg_m"])
        parameter = record.get("mass_parameter")
        if parameter:
            element(inertial, "mass", value="${" + parameter + "}")
        else:
            element(inertial, "mass", value=data["mass_kg"])
        tensor = data["inertia_at_cg_kg_m2"]
        values = {}
        for key, value in tensor.items():
            values[key] = ("${" + f"{value:.15g} * {parameter} / {data['mass_kg']:.15g}" + "}"
                           if parameter else format(value, ".15g"))
        element(inertial, "inertia", **values)
    for i, data in enumerate(record.get("visuals", [])):
        visual = element(link, "visual", name=f"{record['name']}_visual_{i}")
        origin(visual, data.get("xyz_m"), data.get("rpy_rad"))
        geometry(visual, data)
        if "rgba" in data:
            material = element(visual, "material", name=f"{record['name']}_material_{i}")
            element(material, "color", rgba=numbers(data["rgba"]))
    for i, data in enumerate(record.get("collisions", [])):
        collision = element(link, "collision", name=f"{record['name']}_collision_{i}")
        origin(collision, data.get("xyz_m"), data.get("rpy_rad"))
        geometry(collision, data)
    return link


def make_joint(parent, record):
    joint = element(parent, "joint", name=record["name"], type=record["type"])
    element(joint, "parent", link=record["parent"])
    element(joint, "child", link=record["child"])
    origin(joint, record["xyz_m"], record.get("rpy_rad"))
    if record["type"] != "fixed":
        element(joint, "axis", xyz=numbers(record["axis"]))
        limits = record["limits"]
        attributes = {"effort": limits["effort"], "velocity": limits["velocity"]}
        if record["type"] != "continuous":
            attributes.update(lower=limits["lower"], upper=limits["upper"])
        element(joint, "limit", **attributes)
    return joint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, default=PACKAGE / "config/link_joint_sim_map.json")
    args = parser.parse_args()
    data = json.loads(args.map.read_text(encoding="utf-8"))
    assert data["label"] == GRADE
    assert data["status"] == "VALIDATED_EXPORT_INPUTS", "Refuse unfinished simulation map"
    for source in data["source_registers"]:
        verify_source(source, REPO)
    names = [record["name"] for record in data["links"]]
    assert len(names) == len(set(names))
    root = ET.Element("robot", name="tins_ugv_carousel")
    root.append(ET.Comment(" GENERATED. " + GRADE + ". No physical qualification. "))
    for key, value in {"aircraft_mode": "none", "motor_brake_lumped_mass_kg": "4.54"}.items():
        element(root, "{" + NS + "}arg", name=key, default=value)
        element(root, "{" + NS + "}property", name=key, value="$(arg " + key + ")")
    for link in data["links"]:
        make_link(root, link)
    for joint in data["joints"]:
        make_joint(root, joint)
    conditional = element(root, "{" + NS + "}if", value="${aircraft_mode == 'static_visual'}")
    for link in data["static_aircraft_links"]:
        assert not link.get("inertial"), "Display aircraft must not add vehicle inertials"
        assert not link.get("collisions"), "Display aircraft must not affect physics"
        make_link(conditional, link)
    for joint in data["static_aircraft_joints"]:
        make_joint(conditional, joint)
    target = PACKAGE / "urdf"
    target.mkdir(exist_ok=True)
    ET.indent(root, space="  ")
    xacro_path = target / "tins_ugv_carousel.urdf.xacro"
    xacro_path.write_text('<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(root, encoding="unicode"),
                          encoding="utf-8", newline="\n")
    outputs = [xacro_path]
    for mode, name in [("none", "tins_ugv_carousel.urdf"),
                       ("static_visual", "tins_ugv_carousel_static_visual.urdf")]:
        # process_file adds an absolute input-path banner, breaking fresh-clone
        # reproducibility. Parse/process the identical text without that banner.
        xacro.init_stacks(None)
        doc = xacro.parse(xacro_path.read_text(encoding="utf-8"))
        xacro.process_doc(doc, mappings={"aircraft_mode": mode})
        path = target / name
        path.write_text(doc.toprettyxml(indent="  "), encoding="utf-8", newline="\n")
        outputs.append(path)
    provenance = {"label": GRADE, "generator": Path(__file__).name,
                  "map_sha256": source_sha256(args.map), "map_hash_policy": HASH_POLICY,
                  "outputs": [{"file": str(p.relative_to(PACKAGE)).replace("\\", "/"),
                               "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
                              for p in outputs]}
    (target / "generation-provenance.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8", newline="\n")
    print(json.dumps(provenance))


if __name__ == "__main__":
    main()
