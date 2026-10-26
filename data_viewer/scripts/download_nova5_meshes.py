#!/usr/bin/env python3
"""Download the Nova 5 STL files referenced by Dobot's ROS Xacro."""

from pathlib import Path
from urllib.request import Request, urlopen


BASE_URL = (
    "https://raw.githubusercontent.com/Dobot-Arm/TCP-IP-ROS-6AXis/"
    "feature/v4-optimization/dobot_description/meshes/nova5"
)
FILES = (
    "base_link.STL",
    "Link1.STL",
    "Link2.STL",
    "Link3.STL",
    "Link4.STL",
    "Link5.STL",
    "Link6.STL",
)
LICENSE_URL = "https://raw.githubusercontent.com/Dobot-Arm/TCP-IP-ROS-6AXis/feature/v4-optimization/LICENSE"
OUTPUT_DIR = Path(__file__).resolve().parents[1] / "public" / "models" / "nova5"


def download(name: str) -> None:
    request = Request(f"{BASE_URL}/{name}", headers={"User-Agent": "mycobot-data-viewer"})
    with urlopen(request, timeout=60) as response:
        content = response.read()
    if len(content) < 84:
        raise RuntimeError(f"Downloaded file is too small to be an STL: {name}")
    target = OUTPUT_DIR / name
    temporary = target.with_suffix(target.suffix + ".part")
    temporary.write_bytes(content)
    temporary.replace(target)
    print(f"{name}: {len(content):,} bytes")


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for name in FILES:
        download(name)
    request = Request(LICENSE_URL, headers={"User-Agent": "mycobot-data-viewer"})
    with urlopen(request, timeout=30) as response:
        (OUTPUT_DIR / "LICENSE").write_bytes(response.read())
    print(f"Saved Nova 5 meshes to {OUTPUT_DIR}")
    print("Source: Dobot-Arm/TCP-IP-ROS-6AXis, feature/v4-optimization (MIT; LICENSE saved beside meshes)")


if __name__ == "__main__":
    main()
