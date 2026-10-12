#!/usr/bin/env python3
"""Build the driver's C array from Azoteq's header; no DTS or manual copying."""

import argparse
from pathlib import Path
import re

EXPECTED_ADDRESSES = list(range(0x115C, 0x15EC)) + list(range(0x2000, 0x2006))


def parse_header(source):
    """Validate section addresses and bytes before emitting C macro references."""
    entries = []
    addresses = []
    names = set()
    section = None
    address = end = 0
    for line in source.splitlines():
        position = re.search(r"Memory Map Position (0x[0-9A-Fa-f]+) - (0x[0-9A-Fa-f]+)", line)
        if position:
            if section is not None and address != end + 1:
                raise ValueError(f"incomplete section {section}")
            address, end = (int(value, 16) for value in position.groups())
            section = position.group(0)
            if end < address:
                raise ValueError(f"invalid section {section}")
            continue
        define = re.match(r"\s*#define\s+(\w+)\s+(.+?)\s*$", line)
        if not define:
            continue
        name, literal = define.groups()
        if section is None or address > end:
            raise ValueError(f"unexpected byte definition {name}")
        if not re.fullmatch(r"0x[0-9A-Fa-f]+", literal):
            raise ValueError(f"invalid byte definition {name}: {literal}")
        value = int(literal, 16)
        if value > 255 or name in names:
            raise ValueError(f"out-of-range or duplicate byte definition {name}")
        entries.append((name, value))
        addresses.append(address)
        names.add(name)
        address += 1
    if section is None or address != end + 1 or addresses != EXPECTED_ADDRESSES:
        raise ValueError("expected complete 1174-byte map: 0x115C..0x15EB + 0x2000..0x2005")
    return entries


def generate(source):
    entries = parse_header(source)
    lines = ["/* Generated at build time. Values come directly from IQS9150_init.h. */",
             "#ifndef IQS915X_INIT_ARRAY_H", "#define IQS915X_INIT_ARRAY_H",
             "#include <stdint.h>", '#include "IQS9150_init.h"',
             "static const uint8_t iqs915x_init_data[] = {"]
    lines.extend(f"    {name}," for name, _ in entries)
    lines.extend(["};", "#endif", ""])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", "-o", type=Path, required=True)
    args = parser.parse_args()
    try:
        output = generate(args.input.read_text())
    except ValueError as error:
        parser.exit(1, f"IQS9150 initialization header: {error}\n")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output)


if __name__ == "__main__":
    main()
