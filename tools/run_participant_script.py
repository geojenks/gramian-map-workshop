"""Run the commands of docs/PARTICIPANT_SCRIPT.md, as written, in a clone.

    python tools/run_participant_script.py --clone <a fresh clone> --log <new file>

Reads the page, takes every line that starts with "python " from its
```bash blocks, in order, and runs each one from the top folder of the
clone with the Python that runs this tool. It writes what each command
printed, its exit code and its time to the log, in the form the other run
logs use ("=== name", "$ command", output, "--- name: exit N in S s").

Lines in other blocks (the install commands, which download) are not run.
The log lists them first, so it is plain what was left out. The path of the
clone and of the Python are written as <clone> and <python>.

Standard library only. It never writes over a log that is there. A command
that fails does not stop the run: the page says which ones fail today.
"""
import argparse
import io
import os
import re
import shlex
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PAGE = os.path.join(ROOT, "docs", "PARTICIPANT_SCRIPT.md")


def blocks(text):
    """[(language, [lines])] for every fenced block of the page, in order."""
    out, lang, cur = [], None, []
    for line in text.split("\n"):
        m = re.match(r"^```(\w*)\s*$", line)
        if m and lang is None:
            lang, cur = m.group(1), []
        elif m:
            out.append((lang, cur))
            lang = None
        elif lang is not None:
            cur.append(line.rstrip())
    return out


def commands(text):
    """(run, left_out): the python lines of the bash blocks, and every other
    command line of a bash or powershell block."""
    run, left = [], []
    for lang, lines in blocks(text):
        if lang not in ("bash", "powershell"):
            continue
        for line in lines:
            if not line.strip():
                continue
            if lang == "bash" and line.startswith("python "):
                run.append(line)
            else:
                left.append(line)
    return run, left


def name_of(cmd, seen):
    """A short name for a command: its script, numbered when it repeats."""
    parts = shlex.split(cmd)
    base = os.path.splitext(os.path.basename(parts[1]))[0] if len(parts) > 1 else "python"
    seen[base] = seen.get(base, 0) + 1
    return base if seen[base] == 1 else "%s_%d" % (base, seen[base])


def clean(text, clone):
    for a in (clone, clone.replace("\\", "/"), clone.replace("/", "\\")):
        text = text.replace(a, "<clone>")
    exe = sys.executable
    for a in (exe, exe.replace("\\", "/")):
        text = text.replace(a, "<python>")
    return text


def read_log(path):
    """The steps of a log this tool wrote: [{name, cmd, out, exit, s}]."""
    steps, cur = [], None
    with io.open(path, encoding="utf-8") as f:
        for line in f.read().split("\n"):
            m = re.match(r"^--- (\S+): exit (-?\d+) in (\d+) s$", line)
            if line.startswith("=== "):
                cur = {"name": line[4:], "cmd": None, "out": []}
            elif cur is not None and cur["cmd"] is None and line.startswith("$ "):
                cur["cmd"] = line[2:]
            elif cur is not None and m and m.group(1) == cur["name"]:
                cur["exit"], cur["s"] = int(m.group(2)), int(m.group(3))
                steps.append(cur)
                cur = None
            elif cur is not None:
                cur["out"].append(line)
    return steps


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--clone", required=True, help="top folder of a fresh clone")
    ap.add_argument("--log", required=True, help="the log to write (never written over)")
    ap.add_argument("--page", default=PAGE)
    a = ap.parse_args(argv)
    clone = os.path.abspath(a.clone)
    with io.open(a.page, encoding="utf-8") as f:
        run, left = commands(f.read())
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    failed = 0
    with io.open(a.log, "x", encoding="utf-8", newline="\n") as log:
        log.write("Not run (install commands and anything that is not a python line):\n")
        for line in left:
            log.write("  " + line + "\n")
        seen = {}
        for cmd in run:
            name = name_of(cmd, seen)
            args = [sys.executable] + shlex.split(cmd)[1:]
            t = time.time()
            r = subprocess.run(args, cwd=clone, env=env, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT)
            out = r.stdout.decode("utf-8", "replace").replace("\r\n", "\n")
            if out and not out.endswith("\n"):
                out += "\n"
            log.write("=== %s\n$ %s\n%s--- %s: exit %d in %d s\n"
                      % (name, cmd, clean(out, clone), name, r.returncode, time.time() - t))
            log.flush()
            failed += r.returncode != 0
            print("%-28s exit %d" % (name, r.returncode))
    print("%d commands run, %d with a non-zero exit; log %s" % (len(run), failed, a.log))


if __name__ == "__main__":
    main()
