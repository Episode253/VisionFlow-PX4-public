#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import re
from pathlib import Path

def convert_uri(text: str):

    pattern = re.compile(
        r"(<uri>\s*)"
        r"file://[^<]*?/models/"
        r"([^/\s<]+)/"
        r"([^<\s]+)"
        r"(\s*</uri>)",
        re.IGNORECASE,
    )

    count = 0

    def repl(match):
        nonlocal count
        count += 1

        prefix = match.group(1)
        model_name = match.group(2)
        relative_path = match.group(3)
        suffix = match.group(4)

        return f"{prefix}model://{model_name}/{relative_path}{suffix}"

    new_text = pattern.sub(repl, text)
    return new_text, count

def find_remaining_file_uris(text: str, max_items: int = 20):
    pattern = re.compile(r"<uri>\s*file://[^<]+</uri>", re.IGNORECASE)
    return pattern.findall(text)[:max_items]

def main():
    parser = argparse.ArgumentParser(
        description="Convert Gazebo SDF <uri> file://.../models/... paths to model://... paths."
    )

    parser.add_argument(
        "input_file",
        help="SDF / world / model file path. This file will be overwritten directly.",
    )

    args = parser.parse_args()

    input_path = Path(args.input_file)

    if not input_path.exists():
        raise FileNotFoundError(f"Input file not found: {input_path}")

    text = input_path.read_text(encoding="utf-8")
    new_text, count = convert_uri(text)

    input_path.write_text(new_text, encoding="utf-8")

    print(f"[OK] Replaced {count} URI(s).")
    print(f"[OK] Overwritten file: {input_path}")

    remaining = find_remaining_file_uris(new_text)

    if remaining:
        print()
        print("[WARN] Some file:// URI(s) still remain. First examples:")
        for item in remaining:
            print(item)
    else:
        print("[OK] No file:// URI remains in <uri> tags.")

if __name__ == "__main__":
    main()
