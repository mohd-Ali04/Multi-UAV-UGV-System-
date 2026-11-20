"""Offline native-cache helpers. Lengths in metres and SI inertials throughout."""
from functools import lru_cache
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh
import ijson

GRADE = "SIMULATION_GRADE_NOT_PHYSICALLY_QUALIFIED"
DECIMATION_AUDIT = []


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def native_matrix_to_si(values):
    matrix = np.asarray(values, dtype=float).reshape(4, 4).copy()
    matrix[:3, 3] *= 0.01
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10):
        raise ValueError("Invalid native homogeneous matrix")
    if not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-8):
        raise ValueError("Native matrix has non-rigid scale/shear")
    return matrix


def translation(values):
    matrix = np.eye(4)
    matrix[:3, 3] = values
    return matrix


@lru_cache(maxsize=12)
def load_native_body(path):
    path = Path(path)
    cache = path.parent.parent / "predecimated" / (path.stem.replace(".json", "") + ".npz")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if cache.exists():
        with np.load(cache, allow_pickle=False) as data:
            if str(data["source_sha256"]) != digest:
                raise ValueError("Native reduced-mesh cache source changed")
            return trimesh.Trimesh(vertices=data["vertices"], faces=data["faces"], process=False)
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        header = stream.read(8192)
    position = header.find('"vertices":')
    if position < 0:
        raise ValueError("Native mesh header unexpectedly long or malformed")
    record = json.loads(header[:position] + '"vertices":null}')
    if record["label"] != GRADE or record["units"] != "cm":
        raise ValueError("Missing native mesh grade or unexpected units")
    with gzip.open(path, "rb") as stream:
        vertices = np.fromiter(ijson.items(stream, "vertices.item", use_float=True), dtype=np.float64).reshape(-1, 3) * 0.01
    with gzip.open(path, "rb") as stream:
        faces = np.fromiter(ijson.items(stream, "triangles.item", use_float=True), dtype=np.int32).reshape(-1, 3)
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    raw_faces = len(mesh.faces)
    if raw_faces > 50000:
        mesh = merged_visual([mesh], 50000)
    cache.parent.mkdir(exist_ok=True)
    np.savez_compressed(cache, vertices=mesh.vertices, faces=mesh.faces, source_sha256=digest,
                        label=GRADE, raw_triangles=raw_faces)
    return mesh


def placed_mesh(cache_directory, body_definition, body_to_link):
    mesh = load_native_body(str(cache_directory / body_definition["mesh_file"])).copy()
    mesh.apply_transform(body_to_link)
    return mesh


def merged_visual(meshes, target_triangles):
    """Decimate, retaining original extreme faces; never rescale to hide bbox loss."""
    merged = trimesh.util.concatenate(meshes)
    if len(merged.faces) <= target_triangles:
        return merged
    source_bounds = merged.bounds.copy()
    # Preserve actual original faces containing all six coordinate extrema.
    vertices = merged.vertices
    referenced = np.unique(merged.faces)
    active_vertices = vertices[referenced]
    extrema = set(referenced[np.r_[np.argmin(active_vertices, axis=0), np.argmax(active_vertices, axis=0)]].tolist())
    face_indices = sorted({int(np.flatnonzero(np.any(merged.faces == v, axis=1))[0]) for v in extrema})
    extreme_faces = merged.submesh([face_indices], append=True, repair=False)
    import pymeshlab
    cleaned = merged.copy()
    cleaned.merge_vertices()
    cleaned.update_faces(cleaned.unique_faces())
    cleaned.update_faces(cleaned.nondegenerate_faces())
    cleaned.remove_unreferenced_vertices()
    # The previous backend stalled on dense CAD cylinders. MeshLab handles
    # those planar/curved quadrics. Original-vertex placement prevents spikes
    # and envelope expansion without rescaling or coordinate projection.
    reducer = pymeshlab.MeshSet()
    reducer.add_mesh(pymeshlab.Mesh(vertex_matrix=cleaned.vertices, face_matrix=cleaned.faces))
    reducer.meshing_decimation_quadric_edge_collapse(
        targetfacenum=max(12, target_triangles-len(extreme_faces.faces)),
        preservenormal=True, optimalplacement=False, planarquadric=True, autoclean=True)
    result_mesh = reducer.current_mesh()
    reduced = trimesh.Trimesh(vertices=result_mesh.vertex_matrix().copy(),
                              faces=result_mesh.face_matrix().copy(), process=False)
    DECIMATION_AUDIT.append(dict(backend="pymeshlab_original_vertex_QEM",
                                input_triangles=len(merged.faces), target_triangles=target_triangles,
                                reduced_triangles=len(reduced.faces), original_extreme_faces=len(extreme_faces.faces)))
    result = trimesh.util.concatenate([reduced, extreme_faces])
    if np.max(np.abs(result.bounds - source_bounds)) > 1e-9:
        raise ValueError("Source extreme faces not retained during decimation")
    return result


def export_labelled_stl(mesh, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = bytearray(mesh.export(file_type="stl"))
    payload[:80] = GRADE.encode("ascii").ljust(80, b" ")
    path.write_bytes(payload)


def central_box_tensor_per_kg(bounds):
    x, y, z = np.diff(np.asarray(bounds), axis=0)[0]
    return np.diag([y*y+z*z, x*x+z*z, x*x+y*y]) / 12


def body_inertial(definition, body_to_link, assigned_mass):
    """Use shape inertia about its own CG, then rotate into the physical link."""
    props = definition["properties"]
    if props.get("unavailable"):
        raise ValueError(f"Missing physical shape properties: {definition['body_id']}")
    if not np.isfinite(assigned_mass) or assigned_mass <= 0:
        raise ValueError("Assigned physical mass must be positive")
    centre = np.asarray(props["cg_native_cm"]) * 0.01
    shape = np.asarray(props["central_inertia_per_kg_m2"])
    shape = (shape + shape.T) / 2
    eig = np.linalg.eigvalsh(shape)
    if not np.isfinite(shape).all() or eig.min() <= 0:
        raise ValueError(f"Native normalized shape inertia is invalid: {definition['body_id']}: {eig}")
    rotation = body_to_link[:3, :3]
    centre_link = rotation @ centre + body_to_link[:3, 3]
    tensor_link = assigned_mass * rotation @ shape @ rotation.T
    return {"mass_kg": float(assigned_mass), "cg_m": centre_link,
            "tensor_kg_m2": tensor_link}


def aggregate_inertials(parts):
    total = sum(p["mass_kg"] for p in parts)
    if total <= 0:
        raise ValueError("Physical link has no assigned mass")
    centre = sum(p["mass_kg"] * np.asarray(p["cg_m"]) for p in parts) / total
    tensor = np.zeros((3, 3))
    for part in parts:
        offset = np.asarray(part["cg_m"]) - centre
        tensor += part["tensor_kg_m2"] + part["mass_kg"] * (np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset))
    tensor = (tensor + tensor.T) / 2
    eigenvalues = np.linalg.eigvalsh(tensor)
    if eigenvalues.min() <= 0 or eigenvalues[2] > eigenvalues[:2].sum() + 1e-8:
        raise ValueError("Merged physical inertia is not realizable")
    return {"mass_kg": float(total), "cg_m": centre.tolist(),
            "inertia_at_cg_kg_m2": dict(ixx=float(tensor[0, 0]), iyy=float(tensor[1, 1]),
                                      izz=float(tensor[2, 2]), ixy=float(tensor[0, 1]),
                                      ixz=float(tensor[0, 2]), iyz=float(tensor[1, 2]))}


def box_inertial(mass, centre, size):
    return {"mass_kg": float(mass), "cg_m": np.asarray(centre),
            "tensor_kg_m2": float(mass) * central_box_tensor_per_kg([np.zeros(3), np.asarray(size)])}


def bounds_union(bounds):
    array = np.asarray(bounds)
    return np.stack([array[:, 0].min(axis=0), array[:, 1].max(axis=0)])
