#!/usr/bin/env python3
"""Download and verify the YOLO11n-OBB checkpoint used by visual-only.

The node never downloads a model implicitly.  This script is the explicit
network step, writes outside the repository cache by default and refuses to
overwrite an existing mismatching file.
"""

import argparse
import hashlib
from pathlib import Path
import sys
import urllib.error
import urllib.request


URL = (
    'https://github.com/ultralytics/assets/releases/download/v8.3.0/'
    'yolo11n-obb.pt'
)
SHA256 = 'b62898ebf38940ca4df323863e45ee9d84a1a46d5d11ebdde529fb33aa9f3a32'
SIZE = 5795654
DEFAULT_OUTPUT = '~/.cache/ual_planner/models/yolo11n-obb.pt'


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def verified(path, expected=SHA256):
    path = Path(path)
    return path.is_file() and path.stat().st_size == SIZE and digest(path) == expected


def download(url, output, expected=SHA256):
    output = Path(output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        if verified(output, expected):
            print(f'already verified: {output}')
            return output
        raise SystemExit(
            f'refusing to overwrite existing file with mismatching SHA-256: {output}'
        )
    part = output.with_name(output.name + '.part')
    offset = part.stat().st_size if part.exists() else 0
    request = urllib.request.Request(url, headers={'User-Agent': 'UAL-planner/1.0'})
    if offset:
        request.add_header('Range', f'bytes={offset}-')
    try:
        with urllib.request.urlopen(request, timeout=60) as response:
            mode = 'ab' if offset and response.status == 206 else 'wb'
            if mode == 'wb':
                offset = 0
            with part.open(mode) as handle:
                while True:
                    chunk = response.read(1024 * 1024)
                    if not chunk:
                        break
                    handle.write(chunk)
    except (urllib.error.URLError, OSError) as error:
        raise SystemExit(f'download failed: {error}; partial file retained at {part}')
    if not verified(part, expected):
        raise SystemExit(
            f'SHA-256/size verification failed for {part}; partial file retained'
        )
    part.replace(output)
    print(f'downloaded and verified: {output}')
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default=URL)
    parser.add_argument('--sha256', default=SHA256)
    parser.add_argument('--output', default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    download(args.url, args.output, args.sha256)


if __name__ == '__main__':
    main()
