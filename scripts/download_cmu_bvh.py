#!/usr/bin/env python3
"""Download CMU motion capture data in BVH format.

Downloads selected BVH files from the una-dinosauria/cmu-mocap GitHub
repository. Files are saved to data/bvh/cmu/<subject>/<file>.bvh.

Usage:
    python3 scripts/download_cmu_bvh.py                    # Download defaults
    python3 scripts/download_cmu_bvh.py --all              # Download all recommended
    python3 scripts/download_cmu_bvh.py --subject 002      # Download all from subject 002
    python3 scripts/download_cmu_bvh.py --file 002/02_01   # Download specific file
"""

import argparse
import sys
from pathlib import Path
from urllib.request import urlretrieve
from urllib.error import URLError

# Base URL for raw BVH files
BASE_URL = "https://raw.githubusercontent.com/una-dinosauria/cmu-mocap/master/data"

# Recommended files for testing different motion types
RECOMMENDED_FILES = {
    "walking": [
        ("002", "02_01"),  # Walking
        ("002", "02_03"),  # Slow walk
        ("007", "07_01"),  # Walking
    ],
    "arm_reaching": [
        ("049", "49_02"),  # Arm movements
        ("015", "15_04"),  # Reaching
    ],
    "full_body": [
        ("086", "86_02"),  # Complex motion
        ("086", "86_01"),  # Running
    ],
    "minimal": [
        ("002", "02_01"),  # Walking (smallest file)
    ],
}

# Default download set
DEFAULT_FILES = RECOMMENDED_FILES["minimal"] + RECOMMENDED_FILES["arm_reaching"][:1]


def download_bvh(subject: str, motion_id: str, output_dir: Path) -> bool:
    """Download a single BVH file.

    Args:
        subject: Subject folder (e.g., "002").
        motion_id: Motion ID (e.g., "02_01").
        output_dir: Root output directory.

    Returns:
        True if download succeeded.
    """
    url = f"{BASE_URL}/{subject}/{motion_id}.bvh"
    dest_dir = output_dir / subject
    dest_dir.mkdir(parents=True, exist_ok=True)
    dest_file = dest_dir / f"{motion_id}.bvh"

    if dest_file.exists():
        print(f"  [skip] {dest_file} (already exists)")
        return True

    try:
        print(f"  [download] {url}")
        urlretrieve(url, dest_file)
        size_kb = dest_file.stat().st_size / 1024
        print(f"  [ok] {dest_file} ({size_kb:.0f} KB)")
        return True
    except URLError as e:
        print(f"  [error] {url}: {e}")
        return False
    except Exception as e:
        print(f"  [error] {url}: {e}")
        return False


def validate_bvh(file_path: Path) -> bool:
    """Check if a BVH file is parseable."""
    try:
        import bvhio
        root = bvhio.readAsHierarchy(str(file_path))
        joint_count = len([j for j in _collect_names(root)])
        bvh = bvhio.readAsBvh(str(file_path))
        print(f"  [valid] {file_path.name}: {bvh.FrameCount} frames, {joint_count} joints")
        return True
    except ImportError:
        print("  [warn] bvhio not installed, skipping validation")
        return True
    except Exception as e:
        print(f"  [invalid] {file_path.name}: {e}")
        return False


def _collect_names(joint) -> list:
    names = [joint.Name]
    for child in joint.Children:
        names.extend(_collect_names(child))
    return names


def main():
    parser = argparse.ArgumentParser(description="Download CMU BVH motion data")
    parser.add_argument(
        "--output-dir", type=str,
        default="data/bvh/cmu",
        help="Output directory (default: data/bvh/cmu)",
    )
    parser.add_argument(
        "--all", action="store_true",
        help="Download all recommended files",
    )
    parser.add_argument(
        "--category", type=str,
        choices=list(RECOMMENDED_FILES.keys()),
        help="Download files from a specific category",
    )
    parser.add_argument(
        "--file", type=str,
        help="Download specific file (e.g., 002/02_01)",
    )
    parser.add_argument(
        "--no-validate", action="store_true",
        help="Skip BVH validation after download",
    )

    args = parser.parse_args()

    # Resolve output directory relative to project root
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    output_dir = project_root / args.output_dir

    print(f"CMU BVH Download")
    print(f"Output: {output_dir}")
    print()

    # Determine files to download
    files_to_download = []

    if args.file:
        parts = args.file.split("/")
        if len(parts) == 2:
            files_to_download.append((parts[0], parts[1]))
        else:
            print(f"Error: --file format should be 'SUBJECT/MOTION_ID' (e.g., '002/02_01')")
            sys.exit(1)
    elif args.all:
        for category_files in RECOMMENDED_FILES.values():
            for f in category_files:
                if f not in files_to_download:
                    files_to_download.append(f)
    elif args.category:
        files_to_download = RECOMMENDED_FILES[args.category]
    else:
        files_to_download = DEFAULT_FILES

    # Download
    success_count = 0
    total = len(files_to_download)

    print(f"Downloading {total} files...")
    for subject, motion_id in files_to_download:
        if download_bvh(subject, motion_id, output_dir):
            success_count += 1

    print(f"\nDownloaded: {success_count}/{total}")

    # Validate
    if not args.no_validate:
        print("\nValidating...")
        for subject, motion_id in files_to_download:
            bvh_file = output_dir / subject / f"{motion_id}.bvh"
            if bvh_file.exists():
                validate_bvh(bvh_file)

    print("\nDone.")


if __name__ == "__main__":
    main()
