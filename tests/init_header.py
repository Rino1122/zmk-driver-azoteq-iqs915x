#!/usr/bin/env python3
"""Validate export parsing and prove edits reach the compiled C byte array."""
import importlib.util
import re
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("generator", ROOT / "scripts/generate_init_array.py")
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)
SOURCE = (ROOT / "drivers/input/IQS9150_init.h").read_text()


class InitHeaderTests(unittest.TestCase):
    def test_compiled_bytes_follow_export(self):
        entries = generator.parse_header(SOURCE)
        first_name, first_value = entries[0]
        edited = re.sub(r"(^#define " + first_name + r"\s+)0x[0-9A-Fa-f]+",
                        lambda m: m[1] + f"0x{first_value ^ 0x10:02X}", SOURCE, count=1, flags=re.M)
        self.assertNotEqual(edited, SOURCE)
        for source in (SOURCE, edited):
            entries = generator.parse_header(source)
            self.assertEqual(len(entries), 1174)
            with tempfile.TemporaryDirectory() as temporary:
                directory = Path(temporary)
                (directory / "IQS9150_init.h").write_text(source)
                (directory / "iqs915x_init_array.h").write_text(generator.generate(source))
                (directory / "test.c").write_text(
                    '#include <stdio.h>\n#include "iqs915x_init_array.h"\n'
                    'int main(void) { return fwrite(iqs915x_init_data, 1, sizeof(iqs915x_init_data), stdout) != 1174; }\n')
                subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", str(directory / "test.c"), "-o", str(directory / "test")], check=True)
                result = subprocess.run([str(directory / "test")], check=True, capture_output=True)
                self.assertEqual(result.stdout, bytes(value for _, value in entries))

    def test_reject_corrupt_exports(self):
        first = re.search(r"^#define ALP_COMPENSATION_RX0_0\s+0x[0-9A-Fa-f]+", SOURCE, re.M)[0]
        value = first.split()[-1]
        cases = [
            SOURCE.replace(first, ""),
            SOURCE.replace("0x115C - 0x1175", "0x115D - 0x1176"),
            SOURCE.replace("0x115C - 0x1175", "0x115C - 0x1174"),
            SOURCE.replace(first, first.replace(value, "0x100")),
            SOURCE.replace(first, first.replace(value, "wrong")),
            SOURCE.replace("ALP_COMPENSATION_RX0_1", "ALP_COMPENSATION_RX0_0"),
            SOURCE.replace("0x2000 - 0x2005", "0x2001 - 0x2006"),
        ]
        for source in cases:
            with self.subTest(source=source[:80]), self.assertRaises(ValueError):
                generator.parse_header(source)


if __name__ == "__main__":
    unittest.main()
