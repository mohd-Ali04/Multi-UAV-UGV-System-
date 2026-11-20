"""Portable text-register hashing. Binary asset hashes remain byte-for-byte."""
import hashlib
from pathlib import Path

HASH_POLICY = "sha256-text-crlf-to-lf-v1"


def source_sha256(path):
    return hashlib.sha256(Path(path).read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def source_record(path, repo):
    return dict(file=Path(path).relative_to(repo).as_posix(),
                sha256=source_sha256(path), hash_policy=HASH_POLICY)


def verify_source(record, repo):
    if record.get("hash_policy") != HASH_POLICY:
        raise ValueError("Missing/unsupported source hash policy: " + record["file"])
    if source_sha256(Path(repo) / record["file"]) != record["sha256"]:
        raise ValueError("Changed generation source: " + record["file"])
