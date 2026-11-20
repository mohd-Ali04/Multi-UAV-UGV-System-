"""Preserve and diagnose the first offline export failure without rerunning it."""
import gzip
import json
from pathlib import Path
import shutil

import numpy as np
import trimesh

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
O = REPO / "Simulation/mechanical_design/tins12e/integration/sim_grade"
out = O / "evidence/mesh-decimation-attempt1-diagnosis"
assert not out.exists()
out.mkdir()
for path in [PACKAGE / "scripts/native_export.py", PACKAGE / "scripts/export_link_meshes.py",
             O / "evidence/link-mesh-export-progress.json"]:
    shutil.copy2(path, out / path.name)
path = O / "cad_extract_utf8/bodies/C0308_B00.json.gz"
with gzip.open(path, "rt", encoding="utf-8") as stream:
    raw = json.load(stream)
mesh = trimesh.Trimesh(vertices=np.asarray(raw["vertices"]).reshape(-1, 3)*.01,
                       faces=np.asarray(raw["triangles"]).reshape(-1, 3), process=False)
bounds = mesh.bounds.copy()
mesh.merge_vertices()
reduced = mesh.simplify_quadric_decimation(face_count=49994)
delta = np.maximum(np.maximum(bounds[0]-reduced.vertices, reduced.vertices-bounds[1]), 0)
report = dict(source="C0308_B00", source_triangles=len(mesh.faces), source_bounds_m=bounds.tolist(),
              reduced_bounds_m=reduced.bounds.tolist(), max_QEM_outside_envelope_m=float(delta.max()),
              affected_vertices=int(np.any(delta>0, axis=1).sum()),
              diagnosis="Unconstrained QEM edge-collapse placement overshoots native envelope; source extreme faces prevent shrinkage but not expansion.")
(out / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8", newline="\n")
print(json.dumps(report))
