"""Run explicit conditions (not duplicate seeds of a deterministic scenario).

Run from the repository root: python scripts/benchmark_carry.py
"""
from dataclasses import replace
from datetime import datetime
import json
from pathlib import Path
from cooperative_carry.config import load_config
from cooperative_carry.cli import run_episode


def main():
    root = Path("results") / ("benchmark-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f"))
    cases = [("straight_0p5kg", load_config("configs/carry_g1.toml")),
             ("lateral", load_config("configs/carry_curve.toml")),
             ("stop", load_config("configs/carry_stop.toml"))]
    for mass in (1.0, 2.0):
        cfg = load_config("configs/carry_g1.toml")
        cfg.payload.mass = mass
        cases.append((f"straight_{mass}kg", cfg))
    cfg = load_config("configs/carry_g1.toml")
    cases.append(("physics_1ms", replace(cfg, timestep=.001)))
    summaries = {name: run_episode(cfg, root / name) for name, cfg in cases}
    (root / "comparison.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")
    print(json.dumps(summaries, indent=2))
    print(root)


if __name__ == "__main__":
    main()
