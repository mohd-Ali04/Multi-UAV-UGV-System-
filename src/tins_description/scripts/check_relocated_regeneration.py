"""Check LF/CRLF and path-independent regeneration in a new local directory.

This does not claim the required fresh clone from GitHub; that remains separate.
The new test directory is preserved and never deleted by this script.
"""
from datetime import datetime, timezone
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
O = REPO / "Simulation/mechanical_design/tins12e/integration/sim_grade"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=REPO.parent / "_verification/urdf_relocation_2026-10-20")
    parser.add_argument("--run-id", default="attempt1")
    args = parser.parse_args()
    scratch = args.destination.resolve()
    assert scratch.is_relative_to(REPO.parent), "Test destination must remain inside this workspace."
    assert args.run_id.replace("_", "").replace("-", "").isalnum()
    assert not scratch.exists(), "Use a new destination; preserve previous runs."
    scratch.mkdir(parents=True)
    manifest = json.loads((PACKAGE / "config/link_joint_sim_map.json").read_text(encoding="utf-8"))
    for path in PACKAGE.rglob("*"):
        if not path.is_file() or "__pycache__" in path.parts:
            continue
        dest = scratch / path.relative_to(REPO)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, dest)
    for source in manifest["source_registers"]:
        path = REPO / source["file"]
        dest = scratch / source["file"]
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, dest)
    stable = [*PACKAGE.glob("urdf/*"), PACKAGE / "config/mass_inertia_report.json",
              PACKAGE / "config/mass_inertia_report.md", PACKAGE / "config/link_inertials.csv"]
    stable = [p for p in stable if p.is_file()]
    expected = {p.relative_to(REPO).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in stable}
    results = []
    for mode in ["LF_inputs", "CRLF_inputs"]:
        if mode == "CRLF_inputs":
            inputs = [scratch / x["file"] for x in manifest["source_registers"]]
            inputs.append(scratch / "src/tins_description/config/link_joint_sim_map.json")
            for path in inputs:
                # Deliberate test input conversion simulates a Windows checkout.
                path.write_bytes(path.read_bytes().replace(b"\r\n", b"\n").replace(b"\n", b"\r\n"))
        logs = []
        for name in ["generate_urdf_from_register.py", "report_mass_properties.py", "validate_description.py", "validate_waypoint_frames.py"]:
            script = scratch / "src/tins_description/scripts" / name
            completed = subprocess.run([sys.executable, str(script)], cwd=scratch, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            log = O / "evidence" / ("relocated-"+args.run_id+"-"+mode+"-"+script.stem+".log")
            assert not log.exists()
            log.write_text(completed.stdout.decode("utf-8", errors="replace"), encoding="utf-8", newline="\n")
            logs.append(dict(script=name, exit_code=completed.returncode, log=log.relative_to(REPO).as_posix()))
            assert completed.returncode == 0, (name, mode, str(log))
        observed = {name:hashlib.sha256((scratch / name).read_bytes()).hexdigest() for name in expected}
        assert observed == expected, ("Regeneration output hashes changed", mode)
        assert all(b"\r\n" not in (scratch / name).read_bytes() for name in expected), "Generated outputs are not LF"
        results.append(dict(mode=mode, stable_output_hashes=observed, logs=logs))
    report = dict(status="PASS_RELOCATED_LF_CRLF_REGENERATION", checked_utc=datetime.now(timezone.utc).isoformat(),
                  label="SIMULATION_GRADE_NOT_PHYSICALLY_QUALIFIED", destination=str(scratch), cases=results,
                  stable_files=len(expected), runtime_evidence_excluded=["validation_report.json", "waypoint_frame_validation.json"],
                  reason="Runtime validation evidence contains execution timestamps; generated XML/provenance and mass reports are required to be byte-identical.",
                  limitation="Relocated local file copy, not a GitHub fresh-clone check. Gate A remains pending until branch/tag push and remote fresh clone validation.")
    output = O / "evidence" / ("relocated-regeneration-check-"+args.run_id+".json")
    assert not output.exists()
    output.write_text(json.dumps(report, indent=2), encoding="utf-8", newline="\n")
    print(json.dumps(dict(status=report["status"], stable_files=len(expected), input_modes=2, destination=str(scratch))))


if __name__ == "__main__":
    main()
