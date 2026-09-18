#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import importlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


# utils
def norm_path(p: str) -> str:
    s = (p or "").replace("\\", "/")
    while "//" in s:
        s = s.replace("//", "/")
    return s


def modules_suffix(p: str) -> str:
    s = norm_path(p).lstrip("/")
    idx = s.find("modules/")
    if idx != -1:
        return s[idx:]
    slash = s.rfind("/")
    return s[slash + 1:] if slash != -1 else s


def load_branch_info(path: Path) -> Dict[str, Dict[int, int]]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    out = {}
    for k, arr in data.items():
        suf = modules_suffix(k)
        mp = {}
        if isinstance(arr, list):
            for item in arr:
                try:
                    bn = int(item.get("branch_number"))
                    ln = int(item.get("line_number"))
                except Exception:
                    continue
                mp[bn] = ln
        out[suf] = mp
    return out


def ensure_module_from_proto(proto_path: Path, include_dirs=None):
    module_name = f"{proto_path.stem}_pb2"
    try:
        return importlib.import_module(module_name)
    except ImportError:
        pass

    protoc = shutil.which("protoc")
    if not protoc:
        raise RuntimeError("'protoc' not found")

    tmpdir = Path(tempfile.mkdtemp(prefix="covproto_"))
    incs = [str(proto_path.parent)]
    if include_dirs:
        incs += [str(Path(d)) for d in include_dirs]

    cmd = [protoc]
    for inc in incs:
        cmd.extend(["-I", inc])
    cmd.extend([f"--python_out={tmpdir}", str(proto_path)])

    subprocess.check_call(cmd)
    sys.path.insert(0, str(tmpdir))
    return importlib.import_module(module_name)

def load_failure_map(path: Path) -> Dict[str, str]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if isinstance(data, dict):
        return {str(k): str(v) for k, v in data.items()}

    if isinstance(data, list):
        out = {}
        for it in data:
            ts = it.get("timestamp")
            lab = it.get("label") or it.get("type")
            if ts and lab:
                out[str(ts)] = str(lab)
        return out

    return {}

class PBCoverage:
    def __init__(self):
        self.data: Dict[Tuple[str, int], Dict[str, Any]] = {}

    def add(self, file_suffix, branch_number, hits, fn):
        key = (file_suffix, branch_number)
        prev = self.data.get(key)
        if prev is None:
            self.data[key] = {"hits": hits, "function_name": fn or ""}
        else:
            prev["hits"] += int(hits)
            if not prev.get("function_name") and fn:
                prev["function_name"] = fn


def sanitize_to_instrumentation_data_no_regex(txt: str) -> str:
    allowed = {"execution_start_time", "execution_end_time"}
    lines = txt.splitlines()
    out = []
    i = 0
    while i < len(lines):
        line = lines[i].rstrip()
        stripped = line.lstrip()

        if stripped.startswith("files") and "{" in stripped:
            out.append("files {")
            brace = line.count("{") - line.count("}")
            i += 1
            while i < len(lines) and brace > 0:
                ln = lines[i].rstrip()
                out.append(ln)
                brace += ln.count("{") - ln.count("}")
                i += 1
            continue

        if ":" in line and '"' in line:
            key = line.split(":")[0].strip()
            if key in allowed:
                first = line.find('"')
                last = line.rfind('"')
                if first != -1 and last > first:
                    val = line[first + 1:last]
                    out.append(f'{key}: "{val}"')
        i += 1

    return "\n".join(out) + "\n"


def _parse_branch_id(bid: str, cur_file):
    if not bid:
        return cur_file, None
    us = bid.rfind("_")
    if us != -1:
        tail = bid[us + 1:]
        if tail.isdigit():
            bn = int(tail)
            file_part = bid[:us]
            return modules_suffix(file_part) if file_part else cur_file, bn
    return cur_file, None


def parse_with_proto(pb_path: Path, coverage_pb2):
    from google.protobuf import text_format

    msg = coverage_pb2.InstrumentationData()
    txt = pb_path.read_text("utf-8", errors="ignore")

    try:
        parser = text_format.Parser(allow_unknown_fields=True)
        parser.Merge(txt, msg)
    except Exception:
        sanitized = sanitize_to_instrumentation_data_no_regex(txt)
        text_format.Merge(sanitized, msg)

    cov = PBCoverage()
    for f in msg.files:
        file_sfx = modules_suffix(f.filename)
        for bc in f.branch_coverage:
            eff_file, bn = _parse_branch_id(bc.branch_id, file_sfx)
            if bn is None:
                continue
            eff_file = os.path.relpath(eff_file, "modules/planning")
            if bc.total_hits > 0:
                cov.add(eff_file, bn, bc.total_hits, bc.function_name)
    return cov


def find_best_dir(root: Path, token: str) -> Optional[Path]:
    best = None
    best_score = (-1, -1)
    for p in root.rglob("*"):
        if p.is_dir() and token in p.name:
            score = (len(list(p.glob("*.pb.txt"))), len(p.parts))
            if score > best_score:
                best_score = score
                best = p
    return best


def map_pb_files(d: Path) -> Dict[int, Path]:
    out = {}
    for f in d.rglob("*.pb.txt"):
        base = f.name.split(".", 1)[0]
        if base.isdigit():
            out[int(base)] = f
    return out


# Pattern grouping + scoring
def group_by_identical_pattern(
    seq_by_branch: Dict[Tuple[str, int], List[int]]
) -> Dict[Tuple[int, ...], List[Tuple[str, int]]]:
    groups = {}
    for br, seq in seq_by_branch.items():
        pat = tuple(seq)
        groups.setdefault(pat, []).append(br)
    return groups


def score_group_pattern(pattern: Tuple[int, ...], ff_idx: int, ftype: str):
    seq = list(pattern)
    L = len(seq)

    def w(i):
        return 1.0 / (1.0 + abs(i - ff_idx))

    def transitions(arr):
        return sum(1 for i in range(1, len(arr)) if arr[i] != arr[i - 1])

    # spike
    if ftype == "spike":
        best = 0.0
        for i in range(1, L - 1):
            if seq[i - 1] == seq[i + 1] != seq[i]:
                best = max(best, w(i))
        return best
        # tot = 0.0
        # m = 0
        # for i in range(1, L - 1):
        #     m = m + w(i)
        #     if seq[i - 1] == seq[i + 1] != seq[i]:
        #         tot = tot + w(i)
        # score = tot / m if m > 0 else 0
        # return score

    # persistent
    if ftype == "persistent":
        ts = [i for i in range(1, L) if seq[i] != seq[i - 1]]
        if len(ts) != 1:
            return 0.0
        t = ts[0]
        return w(t) * (L - t) / L

    # oscillation
    if ftype == "oscillation":
        return transitions(seq) / max(1, L - 1)

    return 0.0


# Main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="./sample/faulty_frame_sample.csv" ,help="Input CSV with faulty_frame and timestamp columns.")
    ap.add_argument("--root", default="./sample/coverage_sample", help="Root directory containing coverage for each cases.")
    ap.add_argument("--branch_info", default="./Data/branch_info.json", help="Branch info JSON file.")   
    ap.add_argument("--coverage_proto", default="./Data/coverage.proto", help="Coverage proto file.")
    ap.add_argument("--failure_map", default="./sample/fault_type_sample.csv", help="Mapping of timestamp and fault type (csv file mapping timestamp to type label).")
    ap.add_argument("--proto_include", nargs="*", default=[])
    ap.add_argument("--out", default="./result", help="Output directory for group CSV and JSON files.")
    args = ap.parse_args()

    failure_map = {}
    with open(args.failure_map, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            ts = str(row["timestamp"])
            type_ = str(row["classification"])
            failure_map[ts] = type_
#    failure_map = load_failure_map(Path(args.failure_map))
    binfo = load_branch_info(Path(args.branch_info))
    coverage_pb2 = ensure_module_from_proto(Path(args.coverage_proto), args.proto_include)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    with open(args.csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)

        for row in reader:
            ts = (row.get("timestamp") or "").strip()
            ff_raw = (row.get("faulty_frame") or "").strip()

            if not ff_raw.isdigit():
                print(f"[SKIP] {ts}: invalid faulty_frame='{ff_raw}'")
                continue

            ff = int(ff_raw)

            if ts not in failure_map:
                print(f"[SKIP] {ts}: no failure type")
                continue

            ftype = failure_map[ts]
            print(f"\n[CASE] {ts} type={ftype}")

            d = find_best_dir(Path(args.root), ts)
            if not d:
                continue

            mp = map_pb_files(d)
            frames = [i for i in mp if ff - 30 <= i <= ff + 30]
            frames.sort()
            if ff not in frames:
                continue

            pos = {fid: i for i, fid in enumerate(frames)}
            ff_idx = pos[ff]

            coverage = {}
            meta = {}

            for fid in frames:
                cov = parse_with_proto(mp[fid], coverage_pb2)
                coverage[fid] = set(cov.data.keys())
                for k, v in cov.data.items():
                    meta.setdefault(k, v)

            all_branches = set().union(*coverage.values())

            seq_by_branch = {
                br: [1 if br in coverage[fid] else 0 for fid in frames]
                for br in all_branches
            }

            groups = group_by_identical_pattern(seq_by_branch)

            rows = []
            members = {}
            gid = 1

            for pat, brs in groups.items():
                score = score_group_pattern(pat, ff_idx, ftype)
                rows.append({
                    "group_id": gid,
                    "score": score,
                    "pattern": "".join(map(str, pat)),
                    "member_count": len(brs)
                })
                members[gid] = [{
                    "file": b[0],
                    "branch": b[1],
                    "line": (binfo.get(b[0]) or {}).get(b[1]),
                    "function": meta.get(b, {}).get("function_name", "")
                } for b in brs]
                gid += 1

            rows.sort(key=lambda r: (-r["score"], r["member_count"]))

            with open(out_dir / f"{ts}_groups.csv", "w", newline="") as wf:
                w = csv.DictWriter(wf, fieldnames=rows[0].keys())
                w.writeheader()
                w.writerows(rows)

            with open(out_dir / f"{ts}_group_members.json", "w") as jf:
                json.dump(members, jf, indent=2)

            print(f"[OUT] {ts} done")


if __name__ == "__main__":
    main()
