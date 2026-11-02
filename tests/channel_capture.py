from pathlib import Path
import subprocess,tempfile
r=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'test'
 subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(r/'include'),str(r/'tests/channel_capture.c'),'-o',str(p)],check=True)
 subprocess.run([str(p)],check=True,timeout=10)
