#!/usr/bin/env python3
"""Fetch the selected public weights, not the complete multi-model archive."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import io
import json
from pathlib import Path
import struct
import urllib.request
import zipfile
import zlib

import yaml


class RemoteZip(io.RawIOBase):
    """Seekable HTTP range reader; reject servers silently returning the full file."""

    def __init__(self, url, size, chunk_bytes=4 * 2**20, workers=6):
        self.url, self.size, self.position = url, size, 0
        self.chunk_bytes, self.workers = chunk_bytes, workers

    def seekable(self):
        return True

    def tell(self):
        return self.position

    def seek(self, offset, whence=0):
        if whence not in (0, 1, 2):
            raise ValueError('invalid whence')
        position = offset + (0 if whence == 0 else self.position if whence == 1 else self.size)
        if position < 0:
            raise ValueError('negative seek')
        self.position = position
        return position

    def read(self, size=-1):
        size = self.size - self.position if size < 0 else min(size, self.size - self.position)
        if size <= 0:
            return b''
        start = self.position
        if size > self.chunk_bytes:
            spans = [(offset, min(self.chunk_bytes, start + size - offset))
                     for offset in range(start, start + size, self.chunk_bytes)]
            with ThreadPoolExecutor(max_workers=self.workers) as pool:
                data = b''.join(pool.map(lambda span: self.read_range(*span), spans))
        else:
            data = self.read_range(start, size)
        self.position += size
        return data

    def read_range(self, start, size):
        request = urllib.request.Request(self.url, headers={'Range': f'bytes={start}-{start + size - 1}'})
        with urllib.request.urlopen(request, timeout=60) as response:
            expected = f'bytes {start}-{start + size - 1}/{self.size}'
            if response.status != 206 or response.headers.get('Content-Range') != expected:
                raise RuntimeError('HTTP range mismatch; refusing full archive download')
            data = response.read(size + 1)
        if len(data) != size:
            raise RuntimeError('HTTP range size mismatch')
        if size > 2**20:
            print(f'Received {size} archive bytes at {start}', flush=True)
        return data


def read_member(remote, info):
    remote.seek(info.header_offset)
    header = remote.read(30)
    fields = struct.unpack('<4s5H3L2H', header)
    if fields[0] != b'PK\x03\x04' or fields[2] & 1:
        raise ValueError('invalid or encrypted ZIP member')
    remote.seek(fields[-2] + fields[-1], 1)
    compressed = remote.read(info.compress_size)
    if info.compress_type == zipfile.ZIP_DEFLATED:
        data = zlib.decompress(compressed, -15)
    elif info.compress_type == zipfile.ZIP_STORED:
        data = compressed
    else:
        raise ValueError('unsupported ZIP compression')
    if len(data) != info.file_size or zlib.crc32(data) != info.CRC:
        raise ValueError('ZIP member size/CRC mismatch')
    return data


def member_destination(member):
    path = Path(member)
    return f'{path.parent.name}_metadata.json' if path.name == 'metadata.json' else path.name


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='config/ship_detection_trial.yaml')
    parser.add_argument('--resume', action='store_true', help='Reuse only files matching recorded SHA-256; never overwrite')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    config_path = (root / args.config).resolve()
    config = yaml.safe_load(config_path.read_text())
    destination = root / config['weights_dir']
    record_path = destination / 'download_provenance.json'
    if args.resume:
        provenance = json.loads(record_path.read_text())
        if provenance['config'] != str(config_path):
            raise ValueError('Resume configuration differs')
        for entry in provenance['files']:
            if hashlib.sha256((destination / entry['file']).read_bytes()).hexdigest() != entry['sha256']:
                raise ValueError(f"Existing file hash mismatch: {entry['file']}")
    else:
        destination.mkdir(parents=True, exist_ok=False)
        provenance = {'config': str(config_path), 'files': [], 'status': 'incomplete'}

    def save(name, data, source, crc=None):
        if Path(name).name != name:
            raise ValueError('only flat destination filenames are supported')
        old = next((item for item in provenance['files'] if item['file'] == name), None)
        if old:
            if old['sha256'] != hashlib.sha256(data).hexdigest() or old['source'] != source:
                raise ValueError(f'Resumed source/content mismatch: {name}')
            return
        with (destination / name).open('xb') as handle:
            handle.write(data)
        provenance['files'].append({'file': name, 'bytes': len(data),
                                    'sha256': hashlib.sha256(data).hexdigest(),
                                    'source': source, 'zip_crc32': crc})
        print(f'Downloaded {name}: {len(data)} bytes', flush=True)
        record_path.write_text(json.dumps(provenance, indent=2) + '\n')

    for model in config['models']:
        if 'url' in model:
            if any(item['file'] == model['file'] and item['source'] == model['url'] for item in provenance['files']):
                continue
            with urllib.request.urlopen(model['url'], timeout=60) as response:
                save(model['file'], response.read(), model['url'])
    with urllib.request.urlopen(config['archive']['record_url'], timeout=60) as response:
        record = json.load(response)
    archive = next(item for item in record['files'] if item['key'] == config['archive']['file'])
    provenance['archive'] = archive
    provenance['archive_license'] = record['metadata'].get('license')
    provenance['integrity_note'] = 'Selected members checked by ZIP CRC32 and locally hashed; whole-archive MD5 not checked.'
    remote = RemoteZip(archive['links']['self'], archive['size'])
    with zipfile.ZipFile(remote) as reader:
        members = [(model['archive_member'], model['file']) for model in config['models'] if 'archive_member' in model]
        members += [(name, member_destination(name)) for name in config['archive']['metadata_members']]
        for member, filename in members:
            source = archive['links']['self'] + '#' + member
            if any(item['file'] == filename and item['source'] == source for item in provenance['files']):
                continue
            info = reader.getinfo(member)
            save(filename, read_member(remote, info), source, f'{info.CRC:08x}')
    provenance['status'] = 'complete'
    record_path.write_text(json.dumps(provenance, indent=2) + '\n')


if __name__ == '__main__':
    main()
