#!/usr/bin/env python3
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='iqs-contacts-') as tmp:
 binary=Path(tmp)/'test'
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
  '-I',str(root/'include'),'-I',str(root/'drivers/input'),str(root/'tests/contact_frames.c'),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
