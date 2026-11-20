"""Conservative primitive selection; keep concave horizontal tray notches open."""
import numpy as np
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union


def box_from_bounds(bounds, source):
    lo, hi = np.asarray(bounds)
    return {"type": "box", "size_m": np.maximum(hi - lo, 1e-5).tolist(),
            "xyz_m": ((lo + hi) / 2).tolist(), "rpy_rad": [0, 0, 0],
            "source": source,
            "simplification": "Conservative axis-aligned primitive for one rigid structural part; not raw CAD mesh."}


def horizontal_plate_boxes(mesh, source):
    """Decompose the top-face polygon into vertical slabs, preserving its notches.

    Only used on the horizontal thin tray plate. Rigid support hardware is handled
    separately. Rounded mesh edge facets remain within native tessellation error.
    """
    vertices, faces = np.asarray(mesh.vertices), np.asarray(mesh.faces)
    lo, hi = mesh.bounds
    if hi[2] - lo[2] > 0.01:
        raise ValueError("Horizontal tray decomposition received a thick/non-horizontal part")
    top = vertices[faces]
    mask = np.all(np.abs(top[:, :, 2] - hi[2]) < 1e-7, axis=1)
    polygons = [Polygon(triangle[:, :2]) for triangle in top[mask]
                if abs(np.linalg.det(np.array([triangle[1, :2] - triangle[0, :2], triangle[2, :2] - triangle[0, :2]]))) > 1e-14]
    footprint = unary_union(polygons)
    if footprint.is_empty or not footprint.is_valid:
        raise ValueError("Tray top-face footprint unavailable/invalid")
    regions = list(footprint.geoms) if hasattr(footprint, "geoms") else [footprint]
    xs = sorted({round(x, 9) for polygon in regions for x, _ in polygon.exterior.coords})
    for polygon in regions:
        for ring in polygon.interiors:
            xs += [round(x, 9) for x, _ in ring.coords]
    xs = sorted(set(xs))
    boxes = []
    for start, end in zip(xs, xs[1:]):
        if end - start < 1e-8:
            continue
        x = (start + end) / 2
        cut = footprint.intersection(LineString([(x, lo[1] - 1), (x, hi[1] + 1)]))
        segments = list(cut.geoms) if hasattr(cut, "geoms") else [cut]
        for segment in segments:
            if segment.is_empty or segment.geom_type != "LineString":
                continue
            _, ymin, _, ymax = segment.bounds
            if ymax - ymin < 1e-8:
                continue
            candidate = np.array([[start, ymin, lo[2]], [end, ymax, hi[2]]])
            # Adjacent slabs with identical Y limits merge without filling a notch.
            previous = next((b for b in boxes if abs(b[1, 0] - start) < 1e-8
                             and abs(b[0, 1] - ymin) < 1e-8 and abs(b[1, 1] - ymax) < 1e-8), None)
            if previous is not None:
                previous[1, 0] = end
            else:
                boxes.append(candidate)
    if not boxes:
        raise ValueError("No tray collision slabs generated")
    output = [box_from_bounds(b, source) for b in boxes]
    for record in output:
        record["simplification"] = "Horizontal native top-face footprint decomposed into boxes; concave tray notches and holes retained as open spaces. Edge facets approximated."
    return output
