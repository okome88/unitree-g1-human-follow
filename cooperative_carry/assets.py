"""Explicit, pinned downloads. Normal simulation runs never access the network."""
from concurrent.futures import ThreadPoolExecutor
from hashlib import sha1
from importlib.resources import files
from pathlib import Path
import json
from urllib.request import urlopen


def fetch_assets(destination):
    manifest = json.loads(files("cooperative_carry").joinpath("asset_manifest.json").read_text())
    prefix = "src/assets/robots/unitree_g1/xmls/"
    destination = Path(destination).resolve()
    def fetch(entry):
        relative = entry["path"].removeprefix(prefix)
        target = (destination / relative).resolve()
        if not target.is_relative_to(destination):
            raise ValueError("Asset path outside destination")
        def valid(data):
            return sha1(f"blob {len(data)}\0".encode() + data).hexdigest() == entry["sha"]
        if target.is_file() and valid(target.read_bytes()):
            return
        url = f"https://raw.githubusercontent.com/{manifest['repository']}/{manifest['revision']}/{entry['path']}"
        with urlopen(url, timeout=60) as response:
            data = response.read()
        if not valid(data):
            raise ValueError(f"Asset checksum mismatch: {relative}")
        target.parent.mkdir(parents=True, exist_ok=True)
        temp = target.with_suffix(target.suffix + ".tmp")
        temp.write_bytes(data)
        temp.replace(target)
    with ThreadPoolExecutor(max_workers=4) as pool:
        list(pool.map(fetch, manifest["files"]))
    (destination / "source.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return destination
