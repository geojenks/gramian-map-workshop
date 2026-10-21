"""Shared helpers for the import/export tools: file hashes, git commits and
key-path reads, so every number the workshop uses names the file, the commit
and the key it came from."""
import hashlib
import json
import os
import subprocess


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def git_commit(repo):
    try:
        out = subprocess.run(["git", "-C", repo, "rev-parse", "HEAD"],
                             capture_output=True, text=True, check=True)
        return out.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def git_commit_for(repo, relpath):
    """The last commit that touched `relpath` (stable across unrelated
    commits, unlike HEAD)."""
    try:
        out = subprocess.run(["git", "-C", repo, "log", "-1", "--format=%H",
                              "--", relpath],
                             capture_output=True, text=True, check=True)
        return out.stdout.strip() or None
    except (OSError, subprocess.CalledProcessError):
        return None


def git_dirty(repo, relpath):
    """True if `relpath` differs from the commit (so the commit alone does
    not pin it)."""
    try:
        out = subprocess.run(["git", "-C", repo, "status", "--porcelain",
                              "--", relpath],
                             capture_output=True, text=True, check=True)
        return bool(out.stdout.strip())
    except (OSError, subprocess.CalledProcessError):
        return None


def read_key(path, keypath):
    """Value at a '/'-separated key path in a JSON file."""
    with open(path) as f:
        node = json.load(f)
    for k in keypath.strip("/").split("/"):
        node = node[k]
    return node


def source_entry(repo, relpath, keypath=None):
    full = os.path.join(repo, relpath)
    e = {"commit": git_commit_for(repo, relpath), "path": relpath,
         "sha256": sha256(full), "dirty": git_dirty(repo, relpath)}
    if keypath is not None:
        e["key"] = keypath
    return e
