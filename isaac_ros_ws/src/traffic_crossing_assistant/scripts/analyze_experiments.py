#!/usr/bin/env python3

"""
Analyze Experiment Sessions for Taiwan Traffic Safety System

Inputs per session directory:
  - logs/messages.jsonl
  - logs/latency_summary.json
  - logs/tegrastats.log (optional)
  - clips/*.mp4

Outputs:
  - summary.md (Markdown report)
  - latency_stats.csv
  - decision_counts.csv
  - tegrastats_summary.csv (if available)

Usage:
  python3 analyze_experiments.py --root /home/sophie/visionaid-1/experiment_data --session latest
  python3 analyze_experiments.py --root /home/sophie/visionaid-1/experiment_data --session session_20250803_170122
"""

import argparse
import csv
import glob
import json
import os
import re
from datetime import datetime
from typing import Dict, List, Tuple

import numpy as np


def load_latency_summary(path: str) -> Dict:
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return json.load(f)


def robust_glob(path_pattern: str) -> List[str]:
    try:
        return sorted(glob.glob(path_pattern))
    except Exception:
        return []


def parse_messages(messages_path: str) -> Dict[str, List[Dict]]:
    events: Dict[str, List[Dict]] = {
        'image': [], 'detection': [], 'decision': [], 'traffic_light_state': [],
        'traffic_light_confidence': [], 'time_to_collision': [],
        'vehicle_threat_status': [], 'immediate_crossing_danger': [],
        'motion_compensation_quality': [], 'reasoning': []
    }
    if not os.path.exists(messages_path):
        return events
    with open(messages_path, 'r') as f:
        for line in f:
            try:
                rec = json.loads(line)
            except Exception:
                continue
            t = rec.get('type')
            if t in events:
                events[t].append(rec)
    return events


def summarize_latencies(latency_summary: Dict) -> Dict[str, Dict[str, float]]:
    out = {}
    keys = [
        'yolov8_inference_latency_ms',
        'end_to_end_decision_latency_ms',
        'detection_to_vehicle_threat_latency_ms',
        'detection_to_ttc_latency_ms',
    ]
    for k in keys:
        stats = latency_summary.get(k, {})
        out[k] = {
            'count': stats.get('count', 0),
            'mean_ms': stats.get('mean_ms', 0.0),
            'min_ms': stats.get('min_ms', 0.0),
            'max_ms': stats.get('max_ms', 0.0),
            'std_ms': stats.get('std_ms', 0.0),
        }
    return out


def write_csv(path: str, header: List[str], rows: List[List]):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for row in rows:
            writer.writerow(row)


def summarize_tegrastats(path: str) -> Dict[str, float]:
    # Heuristic parser for Jetson tegrastats
    # We extract temperatures and memory usage; exact format can vary by Jetson version
    if not os.path.exists(path):
        return {}
    temps = []
    ram_used = []
    gpu_mem = []
    with open(path, 'r') as f:
        for line in f:
            # Examples:
            # RAM 2016/7782MB (lfb 148x4MB) SWAP 0/3891MB CPU [3%@1400,off,off,off] EMC_FREQ 0%@1600 GR3D_FREQ 0%@1100
            # GPU@38C CPU@39C AO@39C thermal@37.72C Tdiode@39.25C VDD_IN 3812mW ... GR3D 13%@522
            m_temp = re.findall(r'(GPU|CPU|thermal|Tdiode)@([0-9]+\.?[0-9]*)C', line)
            if m_temp:
                for _, val in m_temp:
                    try:
                        temps.append(float(val))
                    except Exception:
                        pass
            m_ram = re.search(r'RAM\s+(\d+)/(\d+)MB', line)
            if m_ram:
                try:
                    used = float(m_ram.group(1))
                    total = float(m_ram.group(2))
                    ram_used.append(100.0 * used / max(1.0, total))
                except Exception:
                    pass
            m_gpu = re.search(r'GR3D\s+(\d+)%@', line)
            if m_gpu:
                try:
                    gpu_mem.append(float(m_gpu.group(1)))
                except Exception:
                    pass
    def stats(arr: List[float]) -> Tuple[float, float, float]:
        if not arr:
            return (0.0, 0.0, 0.0)
        a = np.array(arr, dtype=np.float64)
        return (float(a.mean()), float(a.min()), float(a.max()))
    t_mean, t_min, t_max = stats(temps)
    r_mean, r_min, r_max = stats(ram_used)
    g_mean, g_min, g_max = stats(gpu_mem)
    return {
        'temp_mean_C': t_mean, 'temp_min_C': t_min, 'temp_max_C': t_max,
        'ram_used_mean_pct': r_mean, 'ram_used_min_pct': r_min, 'ram_used_max_pct': r_max,
        'gpu_activity_mean_pct': g_mean, 'gpu_activity_min_pct': g_min, 'gpu_activity_max_pct': g_max,
    }


def generate_markdown(session_dir: str, latency_stats: Dict[str, Dict[str, float]],
                       decision_counts: Dict[str, int], tegra_summary: Dict[str, float],
                       num_clips: int) -> str:
    def fmt(stats: Dict[str, float]) -> str:
        if not stats or stats.get('count', 0) == 0:
            return 'n/a'
        return f"n={stats['count']}, mean={stats['mean_ms']:.1f} ms, min={stats['min_ms']:.1f}, max={stats['max_ms']:.1f}, std={stats['std_ms']:.1f}"
    lines = []
    lines.append(f"# Experiment Summary: {os.path.basename(session_dir)}")
    lines.append("")
    lines.append("## Latency")
    lines.append(f"- YOLOv8 inference: {fmt(latency_stats.get('yolov8_inference_latency_ms', {}))}")
    lines.append(f"- Detection -> Vehicle threat: {fmt(latency_stats.get('detection_to_vehicle_threat_latency_ms', {}))}")
    lines.append(f"- Detection -> TTC: {fmt(latency_stats.get('detection_to_ttc_latency_ms', {}))}")
    lines.append(f"- End-to-end decision: {fmt(latency_stats.get('end_to_end_decision_latency_ms', {}))}")
    lines.append("")
    lines.append("## Decisions")
    if decision_counts:
        total = sum(decision_counts.values())
        for k, v in sorted(decision_counts.items(), key=lambda x: -x[1]):
            pct = (100.0 * v / total) if total else 0.0
            lines.append(f"- {k}: {v} ({pct:.1f}%)")
    else:
        lines.append("- No decisions recorded")
    lines.append("")
    lines.append("## Power/Thermal/Utilization")
    if tegra_summary:
        lines.append(f"- Temperature mean/min/max (C): {tegra_summary['temp_mean_C']:.1f} / {tegra_summary['temp_min_C']:.1f} / {tegra_summary['temp_max_C']:.1f}")
        lines.append(f"- RAM used mean/min/max (%): {tegra_summary['ram_used_mean_pct']:.1f} / {tegra_summary['ram_used_min_pct']:.1f} / {tegra_summary['ram_used_max_pct']:.1f}")
        lines.append(f"- GPU activity mean/min/max (%): {tegra_summary['gpu_activity_mean_pct']:.1f} / {tegra_summary['gpu_activity_min_pct']:.1f} / {tegra_summary['gpu_activity_max_pct']:.1f}")
    else:
        lines.append("- tegrastats not available")
    lines.append("")
    lines.append("## Artifacts")
    lines.append(f"- Clips recorded: {num_clips}")
    lines.append("- Raw logs: logs/messages.jsonl, logs/latency_summary.json")
    lines.append("")
    return "\n".join(lines)


def analyze_session(session_dir: str) -> None:
    logs_dir = os.path.join(session_dir, 'logs')
    latency_summary = load_latency_summary(os.path.join(logs_dir, 'latency_summary.json'))
    msgs = parse_messages(os.path.join(logs_dir, 'messages.jsonl'))
    tegra_summary = summarize_tegrastats(os.path.join(logs_dir, 'tegrastats.log'))

    # Latency CSV
    latency_stats = summarize_latencies(latency_summary)
    latency_rows = []
    for key, stats in latency_stats.items():
        latency_rows.append([
            key,
            stats.get('count', 0),
            stats.get('mean_ms', 0.0),
            stats.get('min_ms', 0.0),
            stats.get('max_ms', 0.0),
            stats.get('std_ms', 0.0),
        ])
    write_csv(os.path.join(session_dir, 'latency_stats.csv'),
              ['metric', 'count', 'mean_ms', 'min_ms', 'max_ms', 'std_ms'], latency_rows)

    # Decision counts CSV
    decision_counts = latency_summary.get('decision_counts', {})
    rows = [[k, v] for k, v in sorted(decision_counts.items(), key=lambda x: -x[1])]
    write_csv(os.path.join(session_dir, 'decision_counts.csv'), ['decision', 'count'], rows)

    # tegrastats CSV
    if tegra_summary:
        write_csv(os.path.join(session_dir, 'tegrastats_summary.csv'),
                  list(tegra_summary.keys()), [list(tegra_summary.values())])

    # Markdown summary
    md = generate_markdown(session_dir, latency_stats, decision_counts, tegra_summary,
                           num_clips=len(robust_glob(os.path.join(session_dir, 'clips', '*.mp4'))))
    with open(os.path.join(session_dir, 'summary.md'), 'w') as f:
        f.write(md)

    print(f"Wrote analysis to: {session_dir}")


def resolve_session(root: str, session: str) -> str:
    if session == 'latest':
        sessions = [p for p in robust_glob(os.path.join(root, 'session_*')) if os.path.isdir(p)]
        if not sessions:
            raise FileNotFoundError('No sessions found')
        sessions.sort()
        return sessions[-1]
    cand = os.path.join(root, session)
    if not os.path.isdir(cand):
        raise FileNotFoundError(f'Session not found: {cand}')
    return cand


def main(argv=None):
    parser = argparse.ArgumentParser(description='Analyze experiment sessions and generate reports')
    parser.add_argument('--root', type=str, default='/home/sophie/visionaid-1/experiment_data')
    parser.add_argument('--session', type=str, default='latest', help='Session folder name or "latest"')
    args = parser.parse_args(argv)

    session_dir = resolve_session(args.root, args.session)
    analyze_session(session_dir)


if __name__ == '__main__':
    main() 