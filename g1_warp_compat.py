"""Run the G1 experiment using Warp's GPU solver fallback."""

from pathlib import Path
import runpy

import warp as wp

wp.config.enable_mathdx_solver = False
print("Warp: using GPU Cholesky fallback (enable_mathdx_solver=False)", flush=True)
runpy.run_path(str(Path(__file__).with_name("g1_locomotion.py")), run_name="__main__")
