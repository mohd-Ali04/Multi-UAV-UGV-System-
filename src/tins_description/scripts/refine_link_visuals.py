"""Reduce preserved dense link meshes; keep physical map/collisions unchanged.

Run after export_link_meshes.py. The unaccepted dense export is preserved before
this documented backend correction. This operation does not access Fusion.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
import trimesh

from native_export import GRADE, read_json, merged_visual, export_labelled_stl, translation, DECIMATION_AUDIT
from export_link_meshes import target_triangles, palette

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
O = REPO / "Simulation/mechanical_design/tins12e/integration/sim_grade"


def main():
    preserved = O / "evidence/dense-mesh-trial-before-lod-correction"
    assert (preserved / "preservation.json").is_file()
    output = O / "evidence/link-visual-lod-correction.json"
    assert not output.exists(), "Use a new attempt path; never overwrite validation evidence."
    data = read_json(PACKAGE / "config/link_joint_sim_map.json")
    meshes, audit = {}, []
    for record in data["mesh_audit"]:
        name = record["link"]
        path = PACKAGE / record["file"]
        # Read the preserved dense trial so partial outputs cannot become inputs.
        source = preserved / path.relative_to(REPO)
        mesh = trimesh.load_mesh(source, process=False)
        target = 20000 if name == "aircraft_canonical" else target_triangles(name)
        before = len(mesh.faces)
        progress = dict(status="REFINING_LINK_VISUAL", link=name, source_triangles=before, target_triangles=target)
        (O / "evidence/visual-lod-progress.json").write_text(json.dumps(progress), encoding="utf-8", newline="\n")
        if before > target:
            mesh = merged_visual([mesh], target)
        assert len(mesh.faces) <= target + 12, ("LOD target not reached", name, len(mesh.faces), target)
        error = float(np.max(abs(mesh.bounds - np.asarray(record.get("cad_bounds_m", record["export_bounds_m"])))))
        assert error <= .002, ("Visual CAD bounds tolerance exceeded", name, error)
        export_labelled_stl(mesh, path)
        colour = [.18, .19, .20, 1] if name == "aircraft_canonical" else palette(name)
        mesh.visual.face_colors = np.round(np.asarray(colour)*255).astype(np.uint8)
        meshes[name] = mesh
        record.update(triangles=len(mesh.faces), export_bounds_m=mesh.bounds.tolist(),
                      visual_LOD="MeshLab QEM using original vertex positions with retained original extreme faces. No mesh rescaling. Small-feature surface error unqualified.")
        if "cad_bounds_m" in record:
            record["max_bounds_error_m"] = error
        audit.append(dict(link=name, before_triangles=before, triangles=len(mesh.faces), max_bounds_error_m=error))
        print(json.dumps(audit[-1]), flush=True)
    vehicle = sum(len(mesh.faces) for n,mesh in meshes.items() if n != "aircraft_canonical")
    aircraft = len(meshes["aircraft_canonical"].faces)
    total = vehicle + aircraft*8
    assert total <= 310000, ("Approximately300k rendered target exceeded", total)
    scene = trimesh.Scene(base_frame="world")
    joints = {j["child"]:j for j in data["joints"]+data["static_aircraft_joints"]}
    for link in data["links"]+data["static_aircraft_links"]:
        name = link["name"]
        joint = joints.get(name)
        parent = joint["parent"] if joint else "world"
        matrix = translation(joint["xyz_m"]) if joint else np.eye(4)
        if name.endswith("_static_aircraft"):
            geometry = "qav250_visual"
            scene.geometry[geometry] = meshes["aircraft_canonical"]
        elif name in meshes:
            geometry = name + "__mesh"
            scene.geometry[geometry] = meshes[name]
        else:
            geometry = None
        options = dict(frame_to=name, frame_from=parent, matrix=matrix)
        if geometry:
            options["geometry"] = geometry
        scene.graph.update(**options)
    scene.metadata.update(label=GRADE, aircraft_mode="static_visual", units="metres", frame_convention="X forward/Y left/Z up",
                          visual_LOD="Original-vertex QEM, no bbox rescale. Reference wheels explicitly massless; no physical qualification.")
    glb = PACKAGE / "meshes/web/tins_ugv_carousel.glb"
    glb.write_bytes(scene.export(file_type="glb"))
    data["visual_triangle_counts"] = dict(vehicle=vehicle, one_aircraft=aircraft, rendered_with_eight_aircraft=total)
    data["visual_LOD_policy"] = "MeshLab QEM original-vertex placement and retained extreme faces; source dense trial preserved. Bounds acceptance is not fine-feature or physical qualification."
    (PACKAGE / "config/link_joint_sim_map.json").write_text(json.dumps(data, indent=2), encoding="utf-8", newline="\n")
    report = dict(label=GRADE, status="PASS_VISUAL_LOD_COUNT_AND_BOUNDS", links=audit,
                  visual_triangle_counts=data["visual_triangle_counts"], glb_link_nodes=len(data["glb_link_nodes"]),
                  glb_sha256=hashlib.sha256(glb.read_bytes()).hexdigest(), operations=DECIMATION_AUDIT,
                  physical_mass_or_collision_changes=False)
    output.write_text(json.dumps(report, indent=2), encoding="utf-8", newline="\n")
    (O / "evidence/visual-lod-progress.json").write_text(json.dumps({k:v for k,v in report.items() if k not in ["links","operations"]}), encoding="utf-8", newline="\n")
    print(json.dumps({k:v for k,v in report.items() if k not in ["links","operations"]}), flush=True)


if __name__ == "__main__":
    main()
