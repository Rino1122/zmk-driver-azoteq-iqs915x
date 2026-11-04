#!/usr/bin/env python3
from pathlib import Path
import subprocess,tempfile
r=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as tmp:
    binary=Path(tmp)/"processing"
    subprocess.run(["cc","-std=c11","-Wall","-Wextra","-Werror","-fsanitize=address,undefined","-I"+str(r/"include"),str(r/"tests/processing_math.c"),"-o",str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
