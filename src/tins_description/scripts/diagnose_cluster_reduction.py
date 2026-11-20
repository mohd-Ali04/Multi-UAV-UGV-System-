"""Measure dense-cylinder clustering before a documented LOD correction."""
import json
from pathlib import Path
import numpy as np
import trimesh

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
mesh = trimesh.load_mesh(PACKAGE / "meshes/visual/motor_x_a_rotor_visual.stl")
records = []
for spacing in [.0001, .00025, .0005]:
    key = np.floor(mesh.vertices/spacing + .5).astype(np.int64)
    _, representatives, mapping = np.unique(key, axis=0, return_index=True, return_inverse=True)
    vertices = mesh.vertices[representatives]
    faces = mapping[mesh.faces]
    candidate = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    candidate.update_faces(candidate.nondegenerate_faces())
    candidate.update_faces(candidate.unique_faces())
    candidate.remove_unreferenced_vertices()
    reduced = candidate.simplify_quadric_decimation(face_count=244)
    records.append(dict(spacing_m=spacing, clustered_triangles=len(candidate.faces), reduced_triangles=len(reduced.faces),
                        max_cluster_point_displacement_m=float(np.linalg.norm(mesh.vertices-vertices[mapping], axis=1).max())))
out = REPO / "Simulation/mechanical_design/tins12e/integration/sim_grade/evidence/cluster-reduction-diagnosis.json"
assert not out.exists()
out.write_text(json.dumps(dict(source="motor_x_a_rotor_visual.stl", results=records), indent=2), encoding="utf-8", newline="\n")
print(json.dumps(records))
