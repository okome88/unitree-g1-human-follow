"""Physics regression suite. Writes measured JSON, fails on unmet criteria."""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace
from .sim import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime", type=Path, default=Path("~/g1-official-demo/official-runtime"))
    parser.add_argument("--output-dir", type=Path, default=Path("results/validation"))
    args = parser.parse_args()
    failures = []
    for scenario in ("straight", "turn", "stop", "loss", "crossing"):
        path = args.output_dir / f"{scenario}.json"
        code = run(SimpleNamespace(runtime=args.runtime, seconds=30., speed=.4,
                                   seed=42, scenario=scenario, viewer=False, output=path))
        report = json.loads(path.read_text())
        checks = {
            "completed": code == 0 and report["simulated_seconds"] >= 29.99,
            "follow_distance": report["distance_final_m"] is not None and .85 <= report["distance_final_m"] <= 2.5,
            "moved": report["displacement_m"] > (2 if scenario == "stop" else 5),
            "target_clearance": report["distance_min_m"] is not None and report["distance_min_m"] >= .85,
            "loss_stops": report["lost_command_abs_max"] == 0 and
                          (scenario != "loss" or report["stop_reasons"].get("target_lost", 0) >= 140),
            "obstacle_clearance": report["obstacle_clearance_min_m"] is None or report["obstacle_clearance_min_m"] > 0,
        }
        report["acceptance"] = checks
        path.write_text(json.dumps(report, indent=2) + "\n")
        failures.extend(f"{scenario}: {key}" for key, passed in checks.items() if not passed)
    print("PASS: five physics scenarios" if not failures else "FAIL: " + ", ".join(failures))
    raise SystemExit(bool(failures))


if __name__ == "__main__":
    main()
