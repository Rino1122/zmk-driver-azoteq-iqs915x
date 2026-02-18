#!/usr/bin/env python3
"""
IQS9150/IQS9151 初期化ヘッダーファイル → DTS バイト配列変換スクリプト

Azoteq GUIが出力する IQS9150_init.h を読み込み、
shield overlayに貼り付けられるDTSスニペットを標準出力に生成する。

使い方:
    python3 scripts/convert_init_header.py <path_to_IQS9150_init.h>

出力例:
    azoteq,init-data = [
        /* ALP ATI Compensation (0x115C, 26 bytes) */
        00 00 00 00 00 00 00 00 ...
        ...
    ];
"""

import re
import sys
import os

# ヘッダーファイル内のセクション定義
# (セクション名, 開始アドレス, バイト数)
# Memory Map Positionコメントから導出
SECTIONS = [
    ("ALP ATI Compensation",          0x115C, 26),
    ("I2C Slave Address",             0x1176, 2),
    ("Settings Version Numbers",      0x1178, 2),
    ("ATI Multipliers / Dividers",    0x117A, 28),
    ("ATI Settings",                  0x1196, 12),
    ("Sampling Periods and Timing",   0x11A2, 26),
    ("System Settings",               0x11BC, 6),
    ("ALP Settings",                  0x11C2, 10),
    ("Thresholds and Debounce",       0x11CC, 8),
    ("ALP Count and LTA Betas",       0x11D4, 4),
    ("Hardware Settings",             0x11D8, 10),
    ("Trackpad Settings",             0x11E2, 20),
    ("Gesture Settings",              0x11F6, 34),
    ("Rx/Tx Mapping",                 0x1218, 46),
    ("TP Channel Disables",           0x1246, 88),
    ("TP Snap Enable",                0x129E, 88),
    ("Touch Threshold Adjustments",   0x12F6, 506),
    ("Virtual Buttons 0",             0x14F0, 34),
    ("Virtual Buttons 1",             0x1512, 48),
    ("Virtual Buttons 2",             0x1542, 48),
    ("Virtual Sliders 0",             0x1572, 52),
    ("Virtual Sliders 1",             0x15A6, 30),
    ("Virtual Wheels",                0x15C4, 40),
    ("Eng Settings",                  0x2000, 6),
]

# 期待される合計バイト数
EXPECTED_MAIN_SIZE = 1168  # 0x115C - 0x15EB
EXPECTED_ENG_SIZE = 6      # 0x2000 - 0x2005
EXPECTED_TOTAL = EXPECTED_MAIN_SIZE + EXPECTED_ENG_SIZE


def parse_header(filepath: str) -> list[int]:
    """ヘッダーファイルから #define の値を出現順に抽出する。

    Returns:
        16進値のリスト（各要素は0-255の整数）
    """
    values = []
    define_pattern = re.compile(
        r'^\s*#define\s+\w+\s+(0x[0-9A-Fa-f]+)\s*$'
    )

    with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            line = line.rstrip('\r\n')
            match = define_pattern.match(line)
            if match:
                val = int(match.group(1), 16)
                if val > 0xFF:
                    print(f"Warning: value {match.group(1)} exceeds 0xFF, "
                          f"truncating to lower byte", file=sys.stderr)
                    val = val & 0xFF
                values.append(val)

    return values


def validate_values(values: list[int]) -> bool:
    """抽出されたバイト数を検証する。"""
    if len(values) != EXPECTED_TOTAL:
        print(f"Error: Expected {EXPECTED_TOTAL} bytes, "
              f"but extracted {len(values)} bytes.", file=sys.stderr)
        print(f"  Expected: {EXPECTED_MAIN_SIZE} (main) + "
              f"{EXPECTED_ENG_SIZE} (eng) = {EXPECTED_TOTAL}",
              file=sys.stderr)
        return False
    return True


def format_dts_output(values: list[int]) -> str:
    """バイト値をDTSのuint8-array形式に変換する。"""
    lines = []
    lines.append("/* Auto-generated from Azoteq GUI header file */")
    lines.append("/* Paste this property into your shield overlay */")
    lines.append("azoteq,init-data = [")

    offset = 0
    for name, addr, size in SECTIONS:
        # セクションヘッダーコメント
        lines.append(f"    /* {name} (0x{addr:04X}, {size} bytes) */")

        # バイトデータを1行16バイトずつ整形
        section_data = values[offset:offset + size]
        for i in range(0, len(section_data), 16):
            chunk = section_data[i:i + 16]
            hex_str = ' '.join(f'{b:02X}' for b in chunk)
            lines.append(f"    {hex_str}")

        offset += size

    lines.append("];")

    return '\n'.join(lines)


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {os.path.basename(sys.argv[0])} <IQS9150_init.h>",
              file=sys.stderr)
        sys.exit(1)

    filepath = sys.argv[1]
    if not os.path.isfile(filepath):
        print(f"Error: File not found: {filepath}", file=sys.stderr)
        sys.exit(1)

    # ヘッダーファイルを解析
    values = parse_header(filepath)
    print(f"Extracted {len(values)} byte values from {filepath}",
          file=sys.stderr)

    # バイト数を検証
    if not validate_values(values):
        # セクションごとの期待バイト数を表示して診断を助ける
        total_expected = sum(s[2] for s in SECTIONS)
        print(f"\nSection breakdown (total expected: {total_expected}):",
              file=sys.stderr)
        for name, addr, size in SECTIONS:
            print(f"  {name}: {size} bytes (0x{addr:04X})", file=sys.stderr)
        sys.exit(1)

    # DTS形式で出力
    output = format_dts_output(values)
    print(output)

    print(f"\nDone: {len(values)} bytes formatted as DTS uint8-array.",
          file=sys.stderr)


if __name__ == '__main__':
    main()
