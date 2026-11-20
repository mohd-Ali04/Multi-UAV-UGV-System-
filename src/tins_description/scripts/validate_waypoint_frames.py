"""Check generated URDF frame motion against the measured waypoint/frame data."""
from datetime import datetime, timezone
import json
from pathlib import Path
import numpy as np
import trimesh
from yourdfpy import URDF
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
GRADE = "SIMULATION_GRADE_NOT_PHYSICALLY_QUALIFIED"


def main():
    waypoints = yaml.safe_load((PACKAGE / "config/guarded_exchange_waypoints.yaml").read_text(encoding="utf-8"))
    stations = yaml.safe_load((PACKAGE / "config/station_frames.yaml").read_text(encoding="utf-8"))
    frames = {x["name"]:x for x in stations["frames"]}
    track = yaml.safe_load((PACKAGE / "config/track_path.yaml").read_text(encoding="utf-8"))
    robot = URDF.load(str(PACKAGE / "urdf/tins_ugv_carousel.urdf"), load_meshes=False,
                      load_collision_meshes=False, build_scene_graph=True)
    maximum, count, poses = 0., 0, []
    for waypoint in waypoints["waypoints"]:
        robot.update_cfg(waypoint["joint_positions_m"])
        expected = {}
        for side in ["A", "B"]:
            matrix = np.eye(4)
            matrix[:3, 3] = np.asarray(waypoint["tray_origins_in_carousel_mount_m"][side])+[0, 0, .6035]
            expected["tray_"+side.lower()] = matrix

        def frame(name):
            if name not in expected:
                record = frames[name]
                local = trimesh.transformations.euler_matrix(*record["rpy_rad"])
                local[:3, 3] = record["xyz_m"]
                expected[name] = frame(record["parent"]) @ local
            return expected[name]

        error = 0.
        for name in frames:
            actual = robot.get_transform(name, "base_link")
            error = max(error, float(np.max(abs(actual-frame(name)))))
            count += 1
        assert error < 1e-10, ("URDF frame disagrees with measured waypoint", waypoint["name"], error)
        maximum = max(maximum, error)
        poses.append(dict(waypoint=waypoint["name"], frames_checked=len(frames), max_matrix_error=error))
    robot.update_cfg(waypoints["waypoints"][0]["joint_positions_m"])
    contact = []
    for name in track["hidden_contact_wheels"]:
        centre = robot.get_transform(name, "base_link")[:3, 3]
        bottom = float(centre[2]-.13)
        error = abs(bottom-track["track_outer_envelope_bottom_z_m"])
        assert error < .001
        contact.append(dict(link=name, centre_m=centre.tolist(), radius_m=.13, bottom_m=bottom, envelope_bottom_error_m=error))
    result = dict(label=GRADE, status="PASS_URDF_WAYPOINT_FRAME_AND_CONTACT_DATUM_CHECK",
                  checked_utc=datetime.now(timezone.utc).isoformat(), poses=poses, frame_matrices_checked=count,
                  max_matrix_error=maximum, hidden_contacts=contact,
                  limitation="Offline forward kinematics against measured CAD waypoint/frame data. No Gazebo contact, guarded-controller, physical lock or flight acceptance.")
    output = PACKAGE / "waypoint_frame_validation.json"
    output.write_text(json.dumps(result, indent=2), encoding="utf-8", newline="\n")
    print(json.dumps(dict(status=result["status"], frames_checked=count, max_matrix_error=maximum, hidden_contacts=len(contact))))


if __name__ == "__main__":
    main()
