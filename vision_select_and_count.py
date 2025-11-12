"""Utilities for selecting ROIs and counting cars crossing a line in each ROI.

This module is intentionally self-contained so it can be used both by humans (to
visualise the counting overlay) and by :mod:`iot_publisher` which imports the
:func:`count_stream` generator.  The generator emits the number of cars detected
in the last window for the four directions N, S, E and W.
"""

from __future__ import annotations

import json
import math
import os
import queue
import time
from dataclasses import dataclass, field
from typing import Dict, Iterable, Iterator, List, Tuple

import cv2
import numpy as np

# ---- Settings ----
CAM_INDEX = 0
FRAME_W, FRAME_H = 1280, 720
MIN_AREA = 500  # minimum contour area inside each ROI
LINE_POS = 0.6  # counting line position (fraction inside ROI)
BAND_FRAC = 0.08  # +/- band around the line (fraction of min(w, h))
PRINT_EVERY = 1.0  # seconds
ROI_CFG = "roi_config.json"

# A simple queue to publish live counts to other modules
_counts_q: "queue.Queue[Dict[str, int]]" = queue.Queue(maxsize=1)


def get_counts_queue() -> "queue.Queue[Dict[str, int]]":
    """Return the queue containing the latest counts snapshot."""

    return _counts_q


@dataclass
class ROIState:
    name: str
    rect: Tuple[int, int, int, int]  # (x, y, w, h) in the original frame
    orient: str  # 'h' for N/S, 'v' for E/W
    bs: cv2.BackgroundSubtractor  # background subtractor
    line_px: int
    band_px: int
    tracks: Dict[int, Dict[str, float]] = field(default_factory=dict)
    next_id: int = 0


def _within(val: float, center: float, band: float) -> bool:
    return abs(val - center) <= band


def _select_or_load_rois(frame: np.ndarray) -> Dict[str, Tuple[int, int, int, int]]:
    if os.path.exists(ROI_CFG):
        with open(ROI_CFG, "r", encoding="utf-8") as f:
            data = json.load(f)
        return {k: tuple(map(int, v)) for k, v in data.items()}

    print("Select 4 ROIs in order: N, S, E, W (drag mouse, ENTER after each, ESC to finish)")
    rects = cv2.selectROIs("Select 4 ROIs", frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("Select 4 ROIs")
    if len(rects) != 4:
        raise RuntimeError("You must select exactly 4 ROIs (N, S, E, W).")
    rois = {
        "N": tuple(map(int, rects[0])),
        "S": tuple(map(int, rects[1])),
        "E": tuple(map(int, rects[2])),
        "W": tuple(map(int, rects[3])),
    }
    with open(ROI_CFG, "w", encoding="utf-8") as f:
        json.dump(rois, f, indent=2)
    print(f"Saved ROI configuration to {ROI_CFG}.")
    return rois


def _make_roi_state(name: str, rect: Tuple[int, int, int, int]) -> ROIState:
    x, y, w, h = rect
    orient = "h" if name in {"N", "S"} else "v"
    extent = h if orient == "h" else w
    band_base = max(1, min(w, h))
    line_px = int(extent * LINE_POS)
    band_px = max(2, int(band_base * BAND_FRAC))
    bs = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=25, detectShadows=False)
    return ROIState(name=name, rect=(x, y, w, h), orient=orient, bs=bs, line_px=line_px, band_px=band_px)


def _match_track(state: ROIState, cx: float, cy: float) -> int:
    """Return the track id that best matches the centroid, or -1 for a new one."""

    best_id = -1
    best_dist = float("inf")
    for track_id, track in state.tracks.items():
        dx = track["x"] - cx
        dy = track["y"] - cy
        dist = math.hypot(dx, dy)
        if dist < best_dist:
            best_dist = dist
            best_id = track_id
    if best_dist < 40.0:  # heuristically chosen; works for close consecutive frames
        return best_id
    return -1


def _update_tracks(state: ROIState, detections: Iterable[Tuple[float, float]], now: float) -> int:
    """Update tracks with the provided detections and return the number of crossings."""

    crossings = 0
    line = state.line_px
    band = state.band_px
    orient_x = state.orient == "v"

    updated: Dict[int, Dict[str, float]] = {}

    for cx, cy in detections:
        track_id = _match_track(state, cx, cy)
        if track_id == -1:
            track_id = state.next_id
            state.next_id += 1
            prev_pos = cy if not orient_x else cx
        else:
            prev_pos = state.tracks[track_id]["pos"]
        pos = cy if not orient_x else cx

        counted = state.tracks.get(track_id, {}).get("counted", False)
        if not counted and _within(pos, line, band):
            if (prev_pos < line <= pos) or (prev_pos > line >= pos):
                counted = True
                crossings += 1
        updated[track_id] = {"x": cx, "y": cy, "pos": pos, "counted": counted, "last": now}

    # Keep tracks that were seen recently to avoid losing context on short occlusions.
    for track_id, track in state.tracks.items():
        if track_id in updated:
            continue
        if now - track["last"] < 0.5:
            updated[track_id] = track

    state.tracks = updated
    return crossings


def _process_roi(state: ROIState, frame: np.ndarray, now: float) -> int:
    mask = state.bs.apply(frame)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detections: List[Tuple[float, float]] = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue
        m = cv2.moments(cnt)
        if m["m00"] == 0:
            continue
        cx = float(m["m10"] / m["m00"])
        cy = float(m["m01"] / m["m00"])
        detections.append((cx, cy))

    return _update_tracks(state, detections, now)


def _ensure_resolution(cap: cv2.VideoCapture) -> None:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)


def _emit_counts(counts: Dict[str, int]) -> None:
    try:
        _counts_q.put_nowait(counts.copy())
    except queue.Full:
        try:
            _counts_q.get_nowait()
        except queue.Empty:
            pass
        try:
            _counts_q.put_nowait(counts.copy())
        except queue.Full:
            pass


def count_stream() -> Iterator[Dict[str, int]]:
    """Yield car counts for the directions N, S, E and W.

    The generator aggregates counts over :data:`PRINT_EVERY` seconds and yields
    the totals for that window.  It also publishes the most recent snapshot to
    :func:`get_counts_queue` for other modules.
    """

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open camera index {CAM_INDEX}.")

    _ensure_resolution(cap)

    ok, frame = cap.read()
    if not ok:
        cap.release()
        raise RuntimeError("Failed to read initial frame from camera.")

    rois = _select_or_load_rois(frame)
    states = {name: _make_roi_state(name, rect) for name, rect in rois.items()}

    last_emit = time.time()
    window_counts = {"N": 0, "S": 0, "E": 0, "W": 0}

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            now = time.time()
            for name, state in states.items():
                x, y, w, h = state.rect
                roi_frame = frame[y : y + h, x : x + w]
                if roi_frame.size == 0:
                    continue
                inc = _process_roi(state, roi_frame, now)
                if inc:
                    window_counts[name] += inc

            if now - last_emit >= PRINT_EVERY:
                _emit_counts(window_counts)
                print(f"Counts @ {time.strftime('%H:%M:%S')}: {window_counts}")
                yield window_counts.copy()
                window_counts = {key: 0 for key in window_counts}
                last_emit = now
    finally:
        cap.release()


def _run_viewer() -> None:
    """Simple viewer that draws ROIs and live counts for manual inspection."""

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open camera index {CAM_INDEX}.")

    _ensure_resolution(cap)

    ok, frame = cap.read()
    if not ok:
        cap.release()
        raise RuntimeError("Failed to read initial frame from camera.")

    rois = _select_or_load_rois(frame)
    states = {name: _make_roi_state(name, rect) for name, rect in rois.items()}
    counts = {"N": 0, "S": 0, "E": 0, "W": 0}
    last_emit = time.time()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            now = time.time()
            display = frame.copy()
            for name, state in states.items():
                x, y, w, h = state.rect
                roi_frame = frame[y : y + h, x : x + w]
                cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(
                    display,
                    f"{name}: {counts[name]}",
                    (x + 5, y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
                inc = _process_roi(state, roi_frame, now)
                if inc:
                    counts[name] += inc
            if now - last_emit >= PRINT_EVERY:
                counts = {key: 0 for key in counts}
                last_emit = now
            cv2.imshow("Smart Road — Counts", display)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        for _ in count_stream():
            pass
    except KeyboardInterrupt:
        print("Stopped by user.")
