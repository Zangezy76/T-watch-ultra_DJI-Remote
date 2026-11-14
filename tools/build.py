#!/usr/bin/env python3
"""Build the T-Watch sketch with the same pinned dependencies as CI.

Requires Python 3, Git and Arduino CLI. Build files live in .arduino-build/;
the user's regular Arduino libraries are not changed.
"""

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess


ROOT = Path(__file__).resolve().parent.parent
SKETCH = ROOT / "arduino" / "DJI_Remote_T-Watch"
CORE = "esp32:esp32@3.3.12"
FQBN = "esp32:esp32:twatch_ultra"
BOARD_INDEX = "https://espressif.github.io/arduino-esp32/package_esp32_index.json"
NIMBLE = "NimBLE-Arduino@2.5.0"

# Keep these commits in one place: CI runs this script too.
LIBRARIES = (
    ("LilyGoLib", "Xinyuan-LilyGO/LilyGoLib", "c3f182870f3300c848c2b6a10976ba0e9ec9fbe7"),
    ("TinyGPSPlus", "mikalhart/TinyGPSPlus", "0a205759da22a1c54c5f5285480fe6132592a4e2"),
    ("NFC-RFAL-fork", "lewisxhe/NFC-RFAL-fork", "7bde4587ea44c36c178a43d083d7a7345c1e4cfe"),
    ("ST25R3916-fork", "lewisxhe/ST25R3916-fork", "0c8e00f49d12881d1cff6d6f0879b90ce7ed4033"),
    ("XPowersLib", "lewisxhe/XPowersLib", "85213b21aba8e81dffcc015a8da0804743e0da2f"),
)
THIRDPARTY_REPO = "Xinyuan-LilyGO/LilyGoLib-ThirdParty"
THIRDPARTY_REF = "c8e6de3b4684df60101569cfe8ead8adbbb79643"
THIRDPARTY_LIBRARIES = ("SensorLib", "RadioLib", "lvgl")


def run(*args, capture=False):
    print("+", " ".join(str(arg) for arg in args), flush=True)
    return subprocess.run(args, check=True, text=True, capture_output=capture)


def pinned_clone(name, repo, commit, destination):
    if destination.exists():
        if not (destination / ".git").exists():
            raise RuntimeError(f"{destination} exists but is not a Git checkout")
        head = run("git", "-C", str(destination), "rev-parse", "HEAD", capture=True).stdout.strip()
        if head != commit:
            raise RuntimeError(f"{destination} is at {head}, expected {commit}; remove the cached directory to retry")
        return
    run("git", "clone", "--quiet", "--filter=blob:none", f"https://github.com/{repo}.git", str(destination))
    run("git", "-C", str(destination), "checkout", "--quiet", "--detach", commit)
    print(f"Pinned {name} at {commit[:7]}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mac", default=os.environ.get("CAMERA_MAC_ADDR"), help="camera BLE MAC address")
    parser.add_argument("--cli", default=shutil.which("arduino-cli"), help="path to arduino-cli executable")
    parser.add_argument("--cache-dir", type=Path, default=ROOT / ".arduino-build")
    args = parser.parse_args()

    if not args.cli:
        parser.error("arduino-cli is not on PATH; install it or pass --cli")
    if not args.mac or not re.fullmatch(r"[0-9A-Fa-f]{2}(?::[0-9A-Fa-f]{2}){5}", args.mac):
        parser.error("pass a camera MAC such as --mac AA:BB:CC:DD:EE:FF")

    cache = args.cache_dir.resolve()
    libraries = cache / "sketchbook" / "libraries"
    libraries.mkdir(parents=True, exist_ok=True)
    config = cache / "arduino-cli.yaml"
    config.write_text(
        "directories:\n"
        f"  user: {json.dumps(str(cache / 'sketchbook').replace(os.sep, '/'))}\n"
        "board_manager:\n"
        f"  additional_urls:\n    - {BOARD_INDEX}\n",
        encoding="utf-8",
    )
    cli = (args.cli, "--config-file", str(config))

    installed = json.loads(run(*cli, "core", "list", "--json", capture=True).stdout).get("platforms", [])
    if not any(p.get("id") == "esp32:esp32" and p.get("installed_version") == "3.3.12" for p in installed):
        run(*cli, "core", "update-index")
        run(*cli, "core", "install", CORE)

    for name, repo, commit in LIBRARIES:
        pinned_clone(name, repo, commit, libraries / name)

    thirdparty = cache / "thirdparty"
    pinned_clone("LilyGoLib-ThirdParty", THIRDPARTY_REPO, THIRDPARTY_REF, thirdparty)
    for name in THIRDPARTY_LIBRARIES:
        destination = libraries / name
        marker = destination / ".pinned-ref"
        if destination.exists():
            if not marker.exists() or marker.read_text(encoding="utf-8").strip() != THIRDPARTY_REF:
                raise RuntimeError(f"{destination} is not the pinned copy; remove it from the cache to retry")
        else:
            shutil.copytree(thirdparty / name, destination)
            marker.write_text(THIRDPARTY_REF + "\n", encoding="utf-8")

    nimble_properties = libraries / "NimBLE-Arduino" / "library.properties"
    if not nimble_properties.exists() or "version=2.5.0" not in nimble_properties.read_text(encoding="utf-8"):
        run(*cli, "lib", "update-index")
        run(*cli, "lib", "install", NIMBLE)

    mac = args.mac.upper()
    run(
        *cli, "compile", "--fqbn", FQBN, "--warnings", "default", "--clean",
        "--build-path", str(cache / "build"),
        "--build-property", f'compiler.cpp.extra_flags=-DCAMERA_MAC_ADDR="{mac}"',
        str(SKETCH),
    )
    print(f"Build complete: {cache / 'build'}")


if __name__ == "__main__":
    main()
