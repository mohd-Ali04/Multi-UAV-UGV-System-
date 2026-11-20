"""Read-only LOD diagnostics; keep exported trial and source CAD unchanged."""
import json
from pathlib import Path
import numpy as np
import trimesh

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
output = REPO / "Simulation/mechanical_design/tins12e/integration/sim_grade/evidence/visual-reduction-diagnosis.json"
assert not output.exists()
path = PACKAGE / "meshes/visual/motor_x_a_rotor_visual.stl"
raw = trimesh.load_mesh(path, process=False)
welded = raw.copy()
welded.merge_vertices()
cleaned = welded.copy()
cleaned.update_faces(cleaned.unique_faces())
cleaned.update_faces(cleaned.nondegenerate_faces())
cleaned.remove_unreferenced_vertices()
trimesh.repair.fix_normals(cleaned, multibody=True)
results = []
for name, mesh in [("welded", welded), ("cleaned", cleaned)]:
    for scale in [1, 1000]:
        candidate = mesh.copy()
        candidate.apply_scale(scale)
        reduced = candidate.simplify_quadric_decimation(face_count=244)
        results.append(dict(input=name, coordinate_scale=scale, vertices=len(mesh.vertices),
                            triangles=len(mesh.faces), output_triangles=len(reduced.faces),
                            winding_consistent=mesh.is_winding_consistent, watertight=mesh.is_watertight))
report = dict(label="SIMULATION_GRADE_NOT_PHYSICALLY_QUALIFIED", file=path.relative_to(PACKAGE).as_posix(),
              target_triangles=250, results=results)
output.write_text(json.dumps(report, indent=2), encoding="utf-8", newline="\n")
print(json.dumps(report))
