#!/usr/bin/env python3
"""
Enhanced ROS 2 bag → CSV exporter

Improvements over the original:
- Removes hard‑coded debug call.
- Auto‑detects storage backend from metadata (mcap/sqlite3), with optional --storage override.
- Lets you choose an output directory separate from the bag folder.
- Optionally writes a single combined CSV per bag (one row per message with a 'topic' column).
- Creates a small topics_index.csv per bag summarizing message counts and output paths.
- Safer file handling with --overwrite toggle (default: skip existing files).
- Flexible timestamp formats: float seconds (default), ROS split fields, or ISO 8601.
- Configurable list/array delimiter (default ";").

Usage examples
--------------
# export topic CSVs for all bag folders under ./data to ./exports
python3 allbags_to_csv.py ./data --out ./exports

# same, but force sqlite3 storage, lower the message threshold, and produce one CSV per bag
python3 allbags_to_csv.py ./data --min-msgs 5 --storage sqlite3 --single-csv

Notes
-----
- Requires: rclpy, rosbag2_py, rosidl_runtime_py, pyyaml
- You need the corresponding rosbag2 storage plugin installed for the chosen backend.
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime as dt
import os
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import yaml

try:
    import rclpy
    from rosidl_runtime_py.utilities import get_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
except Exception as e:
    rclpy = None
    SequentialReader = None
    StorageOptions = None
    ConverterOptions = None
    get_message = None
    _IMPORT_ERROR = e
else:
    _IMPORT_ERROR = None


# --------------------------- data helpers --------------------------- #

def _ns_to_float_seconds(ts_ns: int) -> float:
    return ts_ns * 1e-9


def _ns_to_iso8601(ts_ns: int) -> str:
    # ROS bag timestamps are nanoseconds since epoch
    seconds, nanos = divmod(ts_ns, 1_000_000_000)
    return dt.datetime.utcfromtimestamp(seconds).replace(tzinfo=dt.timezone.utc).isoformat().replace('+00:00', 'Z')


@dataclasses.dataclass
class TopicMeta:
    name: str
    type: str
    message_count: int


# --------------------------- message flattening --------------------------- #

def _is_ros_message(obj: Any) -> bool:
    return hasattr(obj, '__slots__') and hasattr(obj, '__class__')


def _flatten(obj: Any, prefix: str = "", out: Optional[Dict[str, Any]] = None, array_delim: str = ";") -> Dict[str, Any]:
    """Recursively flatten a ROS message or nested structure.

    Arrays / lists are joined into a semicolon-separated string by default.
    """
    if out is None:
        out = {}

    def emit(key: str, value: Any):
        out[key if not prefix else f"{prefix}.{key}"] = value

    if _is_ros_message(obj):
        for slot in obj.__slots__:
            try:
                value = getattr(obj, slot)
            except Exception:
                value = None
            _flatten(value, f"{prefix}.{slot}" if prefix else slot, out, array_delim)
    elif isinstance(obj, dict):
        for k, v in obj.items():
            _flatten(v, f"{prefix}.{k}" if prefix else str(k), out, array_delim)
    elif isinstance(obj, (list, tuple)):
        try:
            emit("value", array_delim.join(map(str, obj)))
        except Exception:
            emit("value", str(obj))
    else:
        emit("value" if prefix == "" else prefix.split(".")[-1], obj)

    return out


# --------------------------- core export logic --------------------------- #

def read_metadata(bag_dir: Path) -> Tuple[str, List[TopicMeta]]:
    meta_path = bag_dir / 'metadata.yaml'
    with meta_path.open('r') as f:
        meta = yaml.safe_load(f)

    info = meta.get('rosbag2_bagfile_information') or meta
    storage_id = info.get('storage_identifier') or info.get('storage_id') or 'mcap'

    topics_raw = info.get('topics_with_message_count', [])
    topics: List[TopicMeta] = []
    for t in topics_raw:
        md = t.get('topic_metadata', {})
        topics.append(TopicMeta(
            name=md.get('name', ''),
            type=md.get('type', ''),
            message_count=int(t.get('message_count', 0)),
        ))
    return storage_id, topics


def iter_bag_messages(bag_dir: Path, storage_id: str) -> Iterable[Tuple[int, str, Any]]:
    if _IMPORT_ERROR is not None:
        raise RuntimeError(f"Failed to import ROS 2 python libs: {_IMPORT_ERROR}")

    db_uri = str(bag_dir)

    reader = SequentialReader()
    storage_options = StorageOptions(uri=db_uri, storage_id=storage_id)
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    type_map: Dict[str, Any] = {}
    for conn in reader.get_all_topics_and_types():
        if get_message is None:
            raise RuntimeError("rosidl_runtime_py is required to construct message types")
        type_map[conn.name] = get_message(conn.type)

    while reader.has_next():
        topic, data, ts = reader.read_next()
        msg_type = type_map.get(topic)
        if msg_type is None:
            continue
        msg = msg_type()
        # use rclpy.serialization to deserialize
        from rclpy.serialization import deserialize_message
        msg = deserialize_message(data, msg_type)
        yield ts, topic, msg


def sanitize_topic_name(topic: str) -> str:
    return topic.strip('/').replace('/', '_') or 'root'


def ensure_dir(p: Path):
    p.mkdir(parents=True, exist_ok=True)


def write_topic_csv(rows: List[Dict[str, Any]], csv_path: Path, overwrite: bool):
    if csv_path.exists() and not overwrite:
        return  # skip
    ensure_dir(csv_path.parent)
    # Collect union of keys
    header_keys: List[str] = []
    for r in rows:
        for k in r.keys():
            if k not in header_keys:
                header_keys.append(k)
    with csv_path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=header_keys)
        writer.writeheader()
        writer.writerows(rows)


def write_index(index_rows: List[Dict[str, Any]], csv_path: Path, overwrite: bool):
    if csv_path.exists() and not overwrite:
        return
    ensure_dir(csv_path.parent)
    header = sorted({k for r in index_rows for k in r})
    with csv_path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        writer.writerows(index_rows)


# --------------------------- main workflow --------------------------- #

def process_bag(
    bag_dir: Path,
    out_dir: Path,
    min_msgs: int,
    storage_override: Optional[str],
    timestamp_mode: str,
    array_delim: str,
    overwrite: bool,
    single_csv: bool,
) -> Tuple[int, int]:
    """Return (topics_exported, messages_written)."""
    storage_id_meta, topics = read_metadata(bag_dir)
    storage_id = storage_override or storage_id_meta

    # filter topics
    kept = [t for t in topics if t.message_count > min_msgs]
    if not kept:
        print(f"[WARN] {bag_dir.name}: no topics above min_msgs={min_msgs}")
        return 0, 0

    print(f"[INFO] {bag_dir.name}: using storage '{storage_id}', exporting {len(kept)} topic(s)")

    # prepare outputs
    bag_out_dir = out_dir / bag_dir.name
    ensure_dir(bag_out_dir)

    # combined CSV setup
    combined_rows: List[Dict[str, Any]] = [] if single_csv else None  # type: ignore

    counts: Dict[str, int] = {t.name: 0 for t in kept}
    kept_names = set(counts)

    for ts_ns, topic, msg in iter_bag_messages(bag_dir, storage_id):
        if topic not in kept_names:
            continue

        # timestamp
        if timestamp_mode == 'float':
            ts_val: Any = _ns_to_float_seconds(ts_ns)
        elif timestamp_mode == 'ros':
            sec, nsec = divmod(ts_ns, 1_000_000_000)
            ts_val = {'timestamp.sec': sec, 'timestamp.nanosec': nsec}
        elif timestamp_mode == 'iso':
            ts_val = _ns_to_iso8601(ts_ns)
        else:
            raise ValueError(f"unknown timestamp mode: {timestamp_mode}")

        row: Dict[str, Any] = {}
        # flatten message
        try:
            flat = _flatten(msg, prefix="", array_delim=array_delim)
            # _flatten places values under leaf key names. Ensure uniqueness by prefixing fields with their path
            # The implementation already writes keys as dotted paths.
        except Exception as e:
            flat = {"raw_msg": repr(msg), "flatten_error": str(e)}

        # add timestamp
        if timestamp_mode == 'ros' and isinstance(ts_val, dict):
            row.update(ts_val)
        else:
            row['timestamp'] = ts_val
        row.update(flat)

        counts[topic] += 1

        if single_csv:
            row_with_topic = dict(row)
            row_with_topic['topic'] = topic
            combined_rows.append(row_with_topic)  # type: ignore
        else:
            csv_path = bag_out_dir / f"{sanitize_topic_name(topic)}.csv"
            # Buffering per-topic to reduce random IO: store in memory per topic
            # For simplicity, write per topic at the end — collect per-topic rows
            _topic_buffers.setdefault(csv_path, []).append(row)

    messages_written = sum(counts.values())

    # write outputs
    index_rows: List[Dict[str, Any]] = []

    if single_csv:
        combined_path = bag_out_dir / "combined.csv"
        write_topic_csv(combined_rows, combined_path, overwrite)
        for t in kept:
            index_rows.append({
                'topic': t.name,
                'type': t.type,
                'message_count': t.message_count,
                'exported_messages': counts[t.name],
                'csv_path': str(combined_path.relative_to(out_dir)),
            })
    else:
        for csv_path, rows in list(_topic_buffers.items()):
            if csv_path.parent == bag_out_dir:
                write_topic_csv(rows, csv_path, overwrite)
        for t in kept:
            topic_csv = bag_out_dir / f"{sanitize_topic_name(t.name)}.csv"
            index_rows.append({
                'topic': t.name,
                'type': t.type,
                'message_count': t.message_count,
                'exported_messages': counts[t.name],
                'csv_path': str(topic_csv.relative_to(out_dir)),
            })

    write_index(index_rows, bag_out_dir / 'topics_index.csv', overwrite)

    print(f"[OK] {bag_dir.name}: wrote {messages_written} messages from {len(kept)} topic(s)")
    return len(kept), messages_written


_topic_buffers: Dict[Path, List[Dict[str, Any]]] = {}


def discover_bag_dirs(parent: Path) -> List[Path]:
    """Find subdirectories that look like ROS2 bag folders (must contain metadata.yaml)."""
    bag_dirs: List[Path] = []
    for p in parent.iterdir():
        if p.is_dir() and (p / 'metadata.yaml').exists():
            bag_dirs.append(p)
    return sorted(bag_dirs)


def main():
    ap = argparse.ArgumentParser(description="Export ROS 2 bags to CSVs (one per topic or single combined)")
    ap.add_argument('parent_folder', nargs='?', default='.', help='Directory containing bag folders (with metadata.yaml)')
    ap.add_argument('--out', default=None, help='Output directory (default: write inside each bag folder)')
    ap.add_argument('--min-msgs', type=int, default=10, help='Minimum messages per topic to export (strictly greater than)')
    ap.add_argument('--storage', choices=['mcap', 'sqlite3'], default=None, help='Force a storage backend (default: auto from metadata)')
    ap.add_argument('--timestamp', choices=['float', 'ros', 'iso'], default='float', help='Timestamp format: float seconds, ROS split, or ISO-8601')
    ap.add_argument('--array-delim', default=';', help='Delimiter for array/list fields in CSV')
    ap.add_argument('--overwrite', action='store_true', help='Overwrite existing CSVs (default: skip)')
    ap.add_argument('--single-csv', action='store_true', help='Write a single combined CSV per bag with a topic column')

    args = ap.parse_args()

    parent = Path(args.parent_folder).resolve()
    out_base = Path(args.out).resolve() if args.out else None

    if _IMPORT_ERROR is not None:
        raise SystemExit(f"Required ROS libs are missing: {_IMPORT_ERROR}")

    bag_dirs = discover_bag_dirs(parent)
    if not bag_dirs:
        print(f"[WARN] No bag folders with metadata.yaml found under {parent}")
        return

    total_bags = len(bag_dirs)
    total_topics = 0
    total_messages = 0

    for bag in bag_dirs:
        _topic_buffers.clear()
        out_dir = out_base if out_base else bag
        n_topics, n_msgs = process_bag(
            bag_dir=bag,
            out_dir=out_dir,
            min_msgs=args.min_msgs,
            storage_override=args.storage,
            timestamp_mode=args.timestamp,
            array_delim=args.array_delim,
            overwrite=args.overwrite,
            single_csv=args.single_csv,
        )
        total_topics += n_topics
        total_messages += n_msgs

    print(f"[DONE] processed {total_bags} bag(s); exported {total_topics} topic(s); wrote {total_messages} message rows")


if __name__ == '__main__':
    main()
