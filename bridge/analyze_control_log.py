#!/usr/bin/env python3
"""分析 baxter_precision_teleop.py 生成的 CSV 控制诊断日志。"""

import argparse
import csv
import json
import math
import statistics
from collections import Counter
from typing import Any, Dict, Iterable, List, Optional, Sequence


IK_EVENTS = {
    "MOVE",
    "IK_REJECT",
    "IK_ERROR",
    "IK_INVALID",
    "IK_BRANCH_REJECT",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_log", help="控制程序通过 --csv-log 写出的 CSV 文件")
    parser.add_argument("--json-output", default="")
    parser.add_argument("--expected-loop-hz", type=float, default=50.0)
    parser.add_argument("--max-ik-result-age-ms", type=float, default=250.0)
    parser.add_argument("--max-tracking-error-m", type=float, default=0.05)
    args = parser.parse_args()
    if args.expected_loop_hz <= 0.0:
        parser.error("--expected-loop-hz 必须为正数")
    if args.max_ik_result_age_ms <= 0.0:
        parser.error("--max-ik-result-age-ms 必须为正数")
    if args.max_tracking_error_m <= 0.0:
        parser.error("--max-tracking-error-m 必须为正数")
    return args


def finite_float(value: Any) -> Optional[float]:
    if value in (None, ""):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def percentile(values: Sequence[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    index = int(math.ceil(fraction * len(ordered))) - 1
    return ordered[max(0, min(index, len(ordered) - 1))]


def metric_summary(values: Iterable[float]) -> Dict[str, Optional[float]]:
    samples = list(values)
    return {
        "count": len(samples),
        "median": statistics.median(samples) if samples else None,
        "p95": percentile(samples, 0.95),
        "p99": percentile(samples, 0.99),
        "max": max(samples) if samples else None,
    }


def numeric_values(rows: List[Dict[str, str]], field: str) -> List[float]:
    result = []
    for row in rows:
        value = finite_float(row.get(field))
        if value is not None:
            result.append(value)
    return result


def analyze_rows(
    rows: List[Dict[str, str]],
    expected_loop_hz: float,
    maximum_ik_age_ms: float,
    maximum_tracking_error_m: float,
) -> Dict[str, Any]:
    ik_rows = [row for row in rows if row.get("event") in IK_EVENTS]
    move_rows = [row for row in ik_rows if row.get("event") == "MOVE"]
    event_counts = Counter(row.get("event", "") for row in rows)
    backend_counts = Counter(
        row.get("ik_backend", "unknown") or "unknown" for row in ik_rows
    )

    sequence_lag = []
    for row in ik_rows:
        latest = finite_float(row.get("seq"))
        request = finite_float(row.get("ik_request_seq"))
        if latest is not None and request is not None:
            sequence_lag.append(max(0.0, latest - request))

    metrics = {
        "packet_age_ms": metric_summary(numeric_values(ik_rows, "packet_age_ms")),
        "loop_dt_ms": metric_summary(numeric_values(ik_rows, "loop_dt_ms")),
        "ik_time_ms": metric_summary(numeric_values(ik_rows, "ik_time_ms")),
        "ik_result_age_ms": metric_summary(
            numeric_values(ik_rows, "ik_result_age_ms")
        ),
        "ik_sequence_lag": metric_summary(sequence_lag),
        "tracking_error_m": metric_summary(
            numeric_values(move_rows, "tracking_error_m")
        ),
        "joint_tracking_error_rad": metric_summary(
            numeric_values(move_rows, "joint_tracking_error_rad")
        ),
    }

    warnings = []
    loop_p95 = metrics["loop_dt_ms"]["p95"]
    expected_period_ms = 1000.0 / expected_loop_hz
    if loop_p95 is not None and loop_p95 > expected_period_ms * 1.5:
        warnings.append(
            "控制循环 P95 明显慢于目标周期：{:.1f} ms > {:.1f} ms".format(
                loop_p95,
                expected_period_ms * 1.5,
            )
        )
    ik_age_p95 = metrics["ik_result_age_ms"]["p95"]
    if ik_age_p95 is not None and ik_age_p95 > maximum_ik_age_ms:
        warnings.append(
            "IK 结果年龄 P95 超过验收上限：{:.1f} ms > {:.1f} ms".format(
                ik_age_p95,
                maximum_ik_age_ms,
            )
        )
    tracking_p95 = metrics["tracking_error_m"]["p95"]
    if tracking_p95 is not None and tracking_p95 > maximum_tracking_error_m:
        warnings.append(
            "末端跟踪误差 P95 超过阈值：{:.3f} m > {:.3f} m".format(
                tracking_p95,
                maximum_tracking_error_m,
            )
        )
    if event_counts["IK_ERROR"]:
        warnings.append("日志中存在 IK worker/service 异常")
    if event_counts["IK_REJECT"]:
        warnings.append("日志中存在过期或跨 Grip session 的 IK 结果")

    return {
        "rows": len(rows),
        "ik_results": len(ik_rows),
        "accepted_moves": len(move_rows),
        "ik_acceptance_rate": (
            len(move_rows) / len(ik_rows) if ik_rows else 0.0
        ),
        "event_counts": dict(sorted(event_counts.items())),
        "backend_counts": dict(sorted(backend_counts.items())),
        "metrics": metrics,
        "warnings": warnings,
    }


def print_report(report: Dict[str, Any]) -> None:
    print("控制日志汇总")
    print("  rows: {}".format(report["rows"]))
    print(
        "  IK accepted: {}/{} ({:.1%})".format(
            report["accepted_moves"],
            report["ik_results"],
            report["ik_acceptance_rate"],
        )
    )
    print("  events: {}".format(report["event_counts"]))
    print("  backends: {}".format(report["backend_counts"]))
    for name, summary in report["metrics"].items():
        print(
            "  {}: median={} p95={} p99={} max={} n={}".format(
                name,
                summary["median"],
                summary["p95"],
                summary["p99"],
                summary["max"],
                summary["count"],
            )
        )
    if report["warnings"]:
        print("诊断提示：")
        for warning in report["warnings"]:
            print("  - {}".format(warning))


def main() -> int:
    args = parse_args()
    with open(args.csv_log, newline="", encoding="utf-8") as source:
        rows = list(csv.DictReader(source))
    report = analyze_rows(
        rows,
        args.expected_loop_hz,
        args.max_ik_result_age_ms,
        args.max_tracking_error_m,
    )
    print_report(report)
    if args.json_output:
        with open(args.json_output, "w", encoding="utf-8") as output:
            json.dump(report, output, ensure_ascii=False, indent=2)
        print("JSON: {}".format(args.json_output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
