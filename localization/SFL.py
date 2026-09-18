#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import importlib
import json
import os
import re
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

from google.protobuf import text_format
from google.protobuf.message import Message
from google.protobuf.descriptor import FieldDescriptor


# utils

def extend_sys_path(paths: List[str]):
    for p in paths or []:
        pth = os.path.expanduser(p)
        if pth and pth not in sys.path:
            sys.path.append(pth)


def normalize_ts(ts: str) -> Optional[str]:
    ts = (ts or "").strip()
    if not ts:
        return None

    if re.fullmatch(r"\d{8}_\d{6}", ts):
        return ts

    digits = "".join(ch for ch in ts if ch.isdigit())
    if len(digits) >= 14:
        digits = digits[:14]
        return digits[:8] + "_" + digits[8:14]

    for fmt in ("%Y-%m-%d %H:%M:%S", "%Y-%m-%d %H:%M:%S.%f"):
        try:
            dt = datetime.strptime(ts, fmt)
            return dt.strftime("%Y%m%d_%H%M%S")
        except Exception:
            pass

    return None


def find_replay_dir_prefix(root: Path, ts_norm: str) -> Optional[Path]:
    if not ts_norm:
        return None

    # fast path: direct children
    candidates = [p for p in root.iterdir() if p.is_dir() and p.name.startswith(ts_norm)]

    # fallback: search all
    if not candidates:
        candidates = [p for p in root.rglob("*") if p.is_dir() and p.name.startswith(ts_norm)]

    if not candidates:
        return None

    def score(p: Path) -> Tuple[int, int]:
        has_pb = 1 if any(p.glob("*.pb.txt")) else 0
        depth = len(p.parts)
        return (has_pb, depth)

    candidates.sort(key=score, reverse=True)
    return candidates[0]


# sorting

def _leading_int(s: str) -> Optional[int]:
    num = []
    for ch in s:
        if ch.isdigit():
            num.append(ch)
        else:
            break
    if not num:
        return None
    try:
        return int("".join(num))
    except Exception:
        return None


def frame_number_from_path(p: Path) -> Optional[int]:
    base = p.name.split(".", 1)[0]
    if base.isdigit():
        return int(base)
    n = _leading_int(base)
    if n is not None:
        return n
    digits = "".join(ch for ch in base if ch.isdigit())
    return int(digits) if digits else None


def collect_frame_files_sorted(frames_dir: Path) -> Tuple[List[Path], List[Optional[int]]]:
    files = list(frames_dir.glob("*.pb.txt"))

    def sort_key(p: Path):
        n = frame_number_from_path(p)
        return (0, n) if n is not None else (1, p.name)

    files.sort(key=sort_key)
    numbers = [frame_number_from_path(p) for p in files]
    return files, numbers


# proto getters

def get_enum_name(fd: FieldDescriptor, number: int) -> str:
    if fd.enum_type:
        ev = fd.enum_type.values_by_number.get(number)
        if ev:
            return ev.name
    return str(number)


def get_planning_status(msg: Message) -> Optional[Message]:
    if not hasattr(msg, "planning_status"):
        return None
    try:
        if not msg.HasField("planning_status"):
            return None
    except Exception:
        return None
    return getattr(msg, "planning_status")


def get_pull_over_flag_and_position(ps: Message) -> Tuple[Optional[bool], Optional[Tuple[float, float, Optional[float]]]]:
    if not ps or not hasattr(ps, "pull_over"):
        return (None, None)
    try:
        if not ps.HasField("pull_over"):
            return (None, None)
    except Exception:
        return (None, None)

    po = getattr(ps, "pull_over")
    plan = None
    if hasattr(po, "plan_pull_over_path"):
        try:
            plan = bool(getattr(po, "plan_pull_over_path"))
        except Exception:
            plan = None

    pos = None
    if hasattr(po, "position"):
        try:
            if po.HasField("position"):
                p = getattr(po, "position")
                x = getattr(p, "x", None)
                y = getattr(p, "y", None)
                th = None
                if hasattr(p, "theta"):
                    th = getattr(p, "theta")
                elif hasattr(p, "heading"):
                    th = getattr(p, "heading")
                if x is not None and y is not None:
                    pos = (float(x), float(y), (float(th) if th is not None else None))
        except Exception:
            pos = None

    return (plan, pos)


def get_path_opt_flag(msg: Message) -> Optional[bool]:
    val = getattr(msg, "path_opt_failure", None)
    if isinstance(val, bool):
        return val
    if val is None:
        return None
    return bool(val)


def is_rerouting(msg: Message) -> bool:
    status = getattr(msg, "planning_status", None)
    if not status:
        return False
    rerouting = getattr(status, "rerouting", None)
    try:
        if getattr(rerouting, "last_rerouting_time", 0) > 0:
            return True
    except Exception:
        pass
    return False


def get_main_stop_reason_string(msg: Message) -> Optional[str]:
    if not hasattr(msg, "traj"):
        return None
    tj = getattr(msg, "traj")
    if not tj or not hasattr(tj, "decision"):
        return None
    dec = getattr(tj, "decision")
    if not dec or not hasattr(dec, "main_decision"):
        return None
    md = getattr(dec, "main_decision")
    if not md or not hasattr(md, "stop"):
        return None
    stop = getattr(md, "stop")

    if hasattr(stop, "reason"):
        val = getattr(stop, "reason")
        if isinstance(val, str) and val:
            return val
        fd = stop.DESCRIPTOR.fields_by_name.get("reason")
        if fd and fd.type == FieldDescriptor.TYPE_ENUM:
            try:
                return get_enum_name(fd, int(val))
            except Exception:
                return None
    return None


# detection helpers

def almost_equal_pos(a: Tuple[float, float, Optional[float]], b: Tuple[float, float, Optional[float]], tol=1e-3) -> bool:
    ax, ay, ath = a
    bx, by, bth = b
    if abs(ax - bx) > tol or abs(ay - by) > tol:
        return False
    if (ath is None) != (bth is None):
        return False
    if ath is not None and bth is not None and abs(ath - bth) > tol:
        return False
    return True


def detect_path_optimization_failure(path_flags: List[Optional[bool]]) -> Dict[str, Optional[int]]:
    first_path = next((i for i, v in enumerate(path_flags) if v is True), None)
    return {"first_path_failure_frame": first_path}


def detect_pull_over_change(plan_flags: List[Optional[bool]], positions: List[Optional[Tuple[float, float, Optional[float]]]]) -> Dict[str, Optional[int]]:
    indices = [i for i, f in enumerate(plan_flags) if f is True]
    if not indices:
        return {"first_frame": None, "last_change_frame": None}

    first_idx = indices[0]
    seq = [(i, positions[i]) for i in indices]

    final_pos = None
    for _, pos in reversed(seq):
        if pos is not None:
            final_pos = pos
            break

    if final_pos is None:
        return {"first_frame": first_idx, "last_change_frame": None}

    first_non_none = None
    for _, pos in seq:
        if pos is not None:
            first_non_none = pos
            break

    if first_non_none is not None:
        mutates = any((p is not None and not almost_equal_pos(p, first_non_none)) for _, p in seq)
        if not mutates:
            return {"first_frame": first_idx, "last_change_frame": None}

    last_change_idx = None
    for i, pos in seq:
        if pos is not None and almost_equal_pos(pos, final_pos):
            last_change_idx = i
            break

    return {"first_frame": first_idx, "last_change_frame": last_change_idx}


def map_single_index(idx: Optional[int], filenames: List[str]) -> Dict[str, str]:
    if idx is None or idx < 0 or idx >= len(filenames):
        return {"filenames": ""}
    return {"filenames": filenames[idx]}

def parse_stop_by_id(reason: str) -> Optional[str]:
    m = re.search(r'stop\s+by\s+([^\s]+)', reason, flags=re.IGNORECASE)
    return m.group(1) if m else None

def detect_main_stop(
    oracle: Optional[str],
    reason_last: Optional[str],
    stop_ids_seq: List[List[str]]
) -> Optional[int]:
    if not reason_last:
        return None

    target_id = parse_stop_by_id(reason_last)
    if not target_id:
        return None

    for i, ids in enumerate(reversed(stop_ids_seq)):
        if not ids:
            continue
        if target_id in ids:
            continue
        pos = len(stop_ids_seq) - i
        return pos
    
    return None



# per-case analysis

def decisions_with_stop_ids(msg: Message) -> List[str]:
    ids: List[str] = []
    if not hasattr(msg, "traj"): return ids
    tj = getattr(msg, "traj")
    if not tj or not hasattr(tj, "decision"): return ids
    dec = getattr(tj, "decision")

    if hasattr(dec, "object_decision"):
        od = getattr(dec, "object_decision")
        if hasattr(od, "decision"):
            rep = getattr(od, "decision")
            try:
                for d in rep:
                    idv = getattr(d, "id", "") if hasattr(d, "id") else ""
                    if hasattr(d, "object_decision"):
                        od2 = getattr(d, "object_decision")
                        for d2 in od2:
                            if hasattr(d2, "stop") and d2.HasField("stop"):
                                if idv: ids.append(str(idv))
            except TypeError:
                pass
        try:
            for d in od:
                if not hasattr(d, "DESCRIPTOR"): break
                idv = getattr(d, "id", "") if hasattr(d, "id") else ""
                if hasattr(d, "object_decision"):
                    od2 = getattr(d, "object_decision")
                    if hasattr(od2, "stop") and od2.HasField("stop"):
                        if idv: ids.append(str(idv))
        except TypeError:
            pass

    def scan(node: Message):
        for fd, val in node.ListFields():
            if fd.type == FieldDescriptor.TYPE_MESSAGE:
                if fd.label == FieldDescriptor.LABEL_REPEATED:
                    for subv in val:
                        if hasattr(subv, "DESCRIPTOR"):
                            idv = getattr(subv, "id", None)
                            if isinstance(idv, str) and idv:
                                if hasattr(subv, "object_decision"):
                                    od2 = getattr(subv, "object_decision")
                                    if hasattr(od2, "stop") and od2.HasField("stop"):
                                        ids.append(idv)
                            scan(subv)
                else:
                    if hasattr(val, "DESCRIPTOR"): scan(val)

    scan(dec)
    seen = set()
    out = []
    for x in ids:
        if x not in seen:
            out.append(x)
            seen.add(x)
    return out

def analyze_case(frames_dir: Path, MsgType, oracle: Optional[str]) -> Dict[str, Any]:
    files, _numbers = collect_frame_files_sorted(frames_dir)
    if not files:
        return {"frames_count": 0, "error": f"No .pb.txt in {frames_dir}"}

    names = [p.name for p in files]

    path_fail_seq: List[Optional[bool]] = []
    main_stop_reason_seq: List[Optional[str]] = []
    plan_flags_seq: List[Optional[bool]] = []
    stop_ids_seq: List[List[str]] = []
    po_positions_seq: List[Optional[Tuple[float, float, Optional[float]]]] = []
    file_empty_seq: List[bool] = []
    rerouting_files: List[str] = []

    for fp in files:
        txt = fp.read_text(encoding="utf-8", errors="ignore")
        is_empty = (txt.strip() == "")
        file_empty_seq.append(is_empty)

        msg = MsgType()
        try:
            if not is_empty:
                text_format.Merge(txt, msg)
        except Exception:
            is_empty = True
            file_empty_seq[-1] = True

        if not is_empty and is_rerouting(msg):
            rerouting_files.append(fp.name)

        ps = get_planning_status(msg) if not is_empty else None
        path_fail_seq.append(get_path_opt_flag(msg) if not is_empty else None)
        main_stop_reason_seq.append(get_main_stop_reason_string(msg) if not is_empty else None)
        stop_ids_seq.append(decisions_with_stop_ids(msg) if not is_empty else [])

        if ps:
            plan, pos = get_pull_over_flag_and_position(ps)
        else:
            plan, pos = (None, None)
        plan_flags_seq.append(plan)
        po_positions_seq.append(pos)

    opt = detect_path_optimization_failure(path_fail_seq)

    last_readable_idx = None
    for i in range(len(files) - 1, -1, -1):
        if not file_empty_seq[i]:
            last_readable_idx = i
            break

    last_reason = None
    if last_readable_idx is not None:
        last_reason = main_stop_reason_seq[last_readable_idx]
        if not last_reason:
            for i in range(last_readable_idx - 1, -1, -1):
                if not file_empty_seq[i] and main_stop_reason_seq[i]:
                    last_reason = main_stop_reason_seq[i]
                    break

    main_pos = detect_main_stop(oracle, last_reason, stop_ids_seq)

    po = detect_pull_over_change(plan_flags_seq, po_positions_seq)

    return {
        "frames_count": len(files),
        "events": {
            "rerouting": rerouting_files, 
            "path_optimization_failure": {
                "first_path_failure_frame": opt.get("first_path_failure_frame"),
                "first_path_map": map_single_index(opt.get("first_path_failure_frame"), names),
            },
            "main_stop_decision_for_immobile": {
                "position": main_pos,
                "filenames": map_single_index(main_pos, names).get("filenames", ""),
                "last_main_decision_reason": last_reason,
            },
            "pull_over_position_change": {
                "first_frame": po.get("first_frame"),
                "last_change_frame": po.get("last_change_frame"),
                "first_map": map_single_index(po.get("first_frame"), names),
                "last_map": map_single_index(po.get("last_change_frame"), names),
            },
        },
    }


# selection/filter 
def load_solution_items(json_path: Path) -> List[Dict[str, Any]]:
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if isinstance(data, dict):
        return [data]
    if isinstance(data, list):
        return data
    raise ValueError("solution_replay.json must be dict or list")


def load_target_timestamps(list_path: Path) -> Set[str]:
    with open(list_path, "r", encoding="utf-8") as f:
        target = json.load(f)

    keep = set()
    for item in target:
        keep.add(str(item.get("timestamp")).strip())
    return keep


# extract faulty_frame + classify

_STOP_OBS_RE = re.compile(r"^\d+(?:_\d+)?$")

def extract_faulty_frames(results_json: List[Dict[str, Any]], target_ts: Set[str]) -> Tuple[Dict[str, str], Dict[str, str], List[str]]:
    faulty_frame: Dict[str, str] = {}
    classify: Dict[str, str] = {}
    exceptions: List[str] = []

    for item in results_json:
        ts = item.get("timestamp")
        if ts not in target_ts:
            continue

        if item.get("error"):
            exceptions.append(ts)
            continue

        frame = (item.get("analysis") or {}).get("events") or {}

        rerouting = frame.get("rerouting") or []
        stop_blk = frame.get("main_stop_decision_for_immobile") or {}
        stop_reason = stop_blk.get("last_main_decision_reason")
        

        if stop_reason:
            stop_obs = stop_reason.split()[-1]
            if _STOP_OBS_RE.match(stop_obs):
                fn = (stop_blk.get("filenames") or "").split(".", 1)[0]
                if fn:
                    faulty_frame[ts] = fn
                    classify[ts] = "obstacle"
                    continue

        if rerouting:
            fn = str(rerouting[0]).split(".", 1)[0]
            if fn:
                faulty_frame[ts] = fn
                exp_s = (item.get("expected_start") or "")
                exp_e = (item.get("expected_end") or "")
                if "STOP" in exp_s.upper() or "STOP" in exp_e.upper():
                    classify[ts] = "reroute_stop_sign"
                else:
                    classify[ts] = "reroute_opt_fail"
                continue

        opt_fail = frame.get("path_optimization_failure") or {}
        first_opt_fail = ((opt_fail.get("first_path_map") or {}).get("filenames") or "").split(".", 1)[0]
        if first_opt_fail:
            faulty_frame[ts] = first_opt_fail
            classify[ts] = "opt_fail"
            continue

        pull_over = frame.get("pull_over_position_change") or {}
        if pull_over.get("first_frame") is not None:
            first_frame = pull_over.get("first_frame")
            last_change = pull_over.get("last_change_frame")
            if last_change is not None and last_change != first_frame:
                fn = ((pull_over.get("last_map") or {}).get("filenames") or "").split(".", 1)[0]
                if fn:
                    faulty_frame[ts] = fn
                    classify[ts] = "pull_over_pos_change"
                    continue
            else:
                fn = ((pull_over.get("first_map") or {}).get("filenames") or "").split(".", 1)[0]
                if fn:
                    faulty_frame[ts] = fn
                    classify[ts] = "pull_over_intersection"
                    continue

        exceptions.append(ts)

    return faulty_frame, classify, exceptions


def write_two_csvs(prefix_path: Path, faulty_frame: Dict[str, str], classify: Dict[str, str]):
    ff_path = prefix_path.with_suffix("")
    ff_csv = Path(str(ff_path) + "_faulty_frames.csv")
    cls_csv = Path(str(ff_path) + "_faulty_frame_classification.csv")

    with open(ff_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "faulty_frame"])
        for ts, fr in faulty_frame.items():
            w.writerow([ts, fr])

    with open(cls_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["timestamp", "classification"])
        for ts, c in classify.items():
            w.writerow([ts, c])

    print(f"Faulty frames saved to: {ff_csv}")
    print(f"Faulty frame classifications saved to: {cls_csv}")


# main

def main():
    ap = argparse.ArgumentParser()

    ap.add_argument("--solution_json", required=True, help="Input solution_replay.json (list/dict of items).")
    ap.add_argument("--replay_root", default=str(Path.home() / "apollo" / "data" / "replay"))
    ap.add_argument("--top_module", default="modules.common.instrumentation_logger.proto.replay_debug_pb2")
    ap.add_argument("--top_message", default="ReplayInfo")
    ap.add_argument("--extra_sys_path", nargs="*", default=[])

    ap.add_argument("--out", required=True, help="Output analyzed JSON path.")
    args = ap.parse_args()

    extend_sys_path(args.extra_sys_path)

    mod = importlib.import_module(args.top_module)
    MsgType = getattr(mod, args.top_message)

    root = Path(os.path.expanduser(args.replay_root))
    items = load_solution_items(Path(args.solution_json))

    target_ts = load_target_timestamps(Path(args.solution_json))
    print(f"[INFO] target timestamps: {len(target_ts)}")

    results_json: List[Dict[str, Any]] = []
    processed = 0

    for it in items:
        ts_raw = (it.get("timestamp") or "").strip()
        if ts_raw not in target_ts:
            continue

        expected_start = it.get("expected_start")
        expected_end = it.get("expected_end")
        oracle = it.get("oracle")

        entry: Dict[str, Any] = {
            "timestamp": ts_raw,
            "expected_start": expected_start,
            "expected_end": expected_end,
            "oracle": oracle,
        }

        ts_norm = normalize_ts(ts_raw)
        if not ts_norm:
            entry["error"] = "BAD_TIMESTAMP_FORMAT"
            results_json.append(entry)
            continue

        replay_dir = find_replay_dir_prefix(root, ts_norm)
        if not replay_dir:
            entry["error"] = f"REPLAY_DIR_NOT_FOUND under {root} for prefix {ts_norm}"
            results_json.append(entry)
            continue

        try:
            analysis = analyze_case(replay_dir, MsgType, oracle)
            entry["frames_dir"] = str(replay_dir)
            entry["analysis"] = analysis
            if analysis.get("error"):
                entry["error"] = analysis.get("error")
            results_json.append(entry)
            processed += 1
        except Exception as e:
            entry["frames_dir"] = str(replay_dir)
            entry["error"] = f"ANALYSIS_ERROR: {e}"
            results_json.append(entry)

    out_path = Path(os.path.expanduser(args.out))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(json.dumps(results_json, ensure_ascii=False, indent=2))
    print(f"[INFO] analyzed JSON written: {out_path}")
    print(f"[INFO] processed items: {processed}")

    faulty_frame, classify, exceptions = extract_faulty_frames(results_json, target_ts)
    print(f"[INFO] extracted faulty frames: {len(faulty_frame)}")
    print(f"[INFO] exceptions: {len(exceptions)}")

    write_two_csvs(out_path, faulty_frame, classify)


if __name__ == "__main__":
    main()
