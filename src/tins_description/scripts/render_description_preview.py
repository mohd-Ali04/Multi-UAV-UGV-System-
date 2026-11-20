"""Render actual URDF meshes for visual QA; orthographic, no invented geometry."""
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from yourdfpy import URDF
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
O = REPO / "Simulation/mechanical_design/tins12e/integration/sim_grade"


def main():
    path = O / "evidence/previews/urdf_park_and_exchange.png"
    assert not path.exists(), "Preserve earlier preview."
    path.parent.mkdir(exist_ok=True)
    waypoints = yaml.safe_load((PACKAGE / "config/guarded_exchange_waypoints.yaml").read_text(encoding="utf-8"))["waypoints"]
    widest = next(w for w in waypoints if abs(w["joint_positions_m"]["tray_a_x_joint"]-.405)<1e-8 and abs(w["joint_positions_m"]["tray_b_x_joint"]+.405)<1e-8)
    poses = [waypoints[0], widest]
    def resolve(fname):
        prefix = "package://tins_description/"
        assert fname.startswith(prefix)
        return str(PACKAGE / fname[len(prefix):])
    robot = URDF.load(str(PACKAGE / "urdf/tins_ugv_carousel_static_visual.urdf"), filename_handler=resolve,
                      load_meshes=True, build_scene_graph=True)
    eye = np.array([2.2, -3, 2.2], dtype=float)
    eye /= np.linalg.norm(eye)
    right = np.cross([0, 0, 1], eye)
    right /= np.linalg.norm(right)
    up = np.cross(eye, right)
    basis = np.stack([right, up, eye], axis=1)
    light = np.array([.3, -.5, 1.2])
    light /= np.linalg.norm(light)
    rendered = []
    for pose in poses:
        robot.update_cfg(pose["joint_positions_m"])
        triangles, colours = [], []
        for node in robot.scene.graph.nodes_geometry:
            matrix, name = robot.scene.graph[node]
            mesh = robot.scene.geometry[name]
            vertices = mesh.vertices @ matrix[:3, :3].T + matrix[:3, 3]
            world = vertices[mesh.faces]
            normal = np.cross(world[:, 1]-world[:, 0], world[:, 2]-world[:, 0])
            normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-20)
            shade = .34+.66*np.abs(normal @ light)
            rgba = mesh.visual.face_colors
            colour = np.clip(rgba[:, :3]*shade[:, None], 0, 255).astype(np.uint8)
            triangles.append(world)
            colours.append(colour)
        world = np.concatenate(triangles)
        bounds = np.stack([world.min(axis=(0, 1)), world.max(axis=(0, 1))])
        projected = world @ basis
        rendered.append(dict(world=world, projected=projected, colours=np.concatenate(colours), bounds=bounds))
    canvas = Image.new("RGB", (1800, 1000), "#eff2f6")
    draw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 24)
        small = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 19)
        title = ImageFont.truetype("C:/Windows/Fonts/arialbd.ttf", 29)
    except OSError:
        font = small = title = ImageFont.load_default()
    draw.text((35, 23), "TinS-12E + guarded two-level carousel | actual URDF mesh preview", font=title, fill="#152437")
    draw.text((35, 67), "SIMULATION_GRADE_NOT_PHYSICALLY_QUALIFIED", font=small, fill="#9a4e16")
    ranges = [np.ptp(r["projected"][:, :, :2].reshape(-1, 2), axis=0) for r in rendered]
    scale = min(780/max(r[0] for r in ranges), 650/max(r[1] for r in ranges))
    for index, (pose, r) in enumerate(zip(poses, rendered)):
        left = index*900
        draw.rounded_rectangle((left+20, 110, left+880, 895), radius=14, fill="white", outline="#cdd5df", width=2)
        label = "Saved park: A upper / B lower" if index == 0 else "Recorded outward pose before height exchange"
        draw.text((left+42, 128), label, font=font, fill="#152437")
        projected = r["projected"]
        centre = (projected[:, :, :2].max(axis=(0, 1))+projected[:, :, :2].min(axis=(0, 1)))/2
        screen = (projected[:, :, :2]-centre)*[scale, -scale]+[left+450, 495]
        order = np.argsort(projected[:, :, 2].mean(axis=1), kind="stable")
        for face in order:
            points = [tuple(x) for x in screen[face]]
            draw.polygon(points, fill=tuple(r["colours"][face]))
        extent = np.diff(r["bounds"], axis=0)[0]*1000
        draw.text((left+42, 820), f"Visible geometry X x Y x Z: {extent[0]:.1f} x {extent[1]:.1f} x {extent[2]:.1f} mm", font=small, fill="#152437")
        draw.text((left+42, 850), "Same scale. Z=0 is the ground datum. Aircraft are display-only.", font=small, fill="#526074")
    draw.text((35, 917), "Props shown at saved CAD angles; full rotor sweep is recorded in aircraft_qav250.yaml.", font=small, fill="#344356")
    draw.text((35, 947), "Reference wheels are massless. Physical guards, retention, operating footprint and structural/flight qualification remain open.", font=small, fill="#344356")
    canvas.save(path)
    report = dict(label="SIMULATION_GRADE_NOT_PHYSICALLY_QUALIFIED", file=path.relative_to(REPO).as_posix(),
                  renderer="Orthographic painter projection of actual generated URDF visual triangles, same scale in both panels.",
                  poses=[dict(name=p["name"], joint_positions_m=p["joint_positions_m"], geometry_bounds_m=r["bounds"].tolist()) for p,r in zip(poses,rendered)],
                  limitations="Visual QA only; not photorealistic, a clearance test, flight validation or approved transport/operating-envelope drawing.")
    (path.parent / "preview-manifest.json").write_text(json.dumps(report, indent=2), encoding="utf-8", newline="\n")
    print(json.dumps(dict(file=str(path), poses=[p["name"] for p in poses])))


if __name__ == "__main__":
    main()
