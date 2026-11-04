#!/usr/bin/env python3
"""Compile the integer replacement for the retired periodic LUT correction."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as tmp:
    binary=Path(tmp)/"math"
    subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror","-fsanitize=address,undefined","-I"+str(root/"include"),str(root/"tests/calibration_math.c"),"-o",str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
print("Calibration map integer interpolation: PASS")
