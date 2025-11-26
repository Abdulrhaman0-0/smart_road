"""Simple ROI-based motion detection for the Smart Road demo."""

from __future__ import annotations

import json
import os
import queue
import time
from dataclasses import dataclass
from typing import Dict, Iterator, List, Tuple

import cv2

CAM_INDEX = 0
FRAME_W = 1280
FRAME_H = 720
MIN_AREA = 500
PRINT_EVERY = 1.0
ROI_CFG = "roi_config.json"

CountsDict = Dict[str, int]

_counts_queue: "queue.Queue[CountsDict]" = queue.Queue(maxsize=1)


def get_counts_queue() -> "queue.Queue[CountsDict]":
    return _counts_queue


@dataclass
class ROIState:
    name: str
    rect: Tuple[int, int, int, int]
    subtractor: cv2.BackgroundSubtractor


def _select_or_load_rois(frame) -> Dict[str, Tuple[int, int, int, int]]:
    if os.path.exists(ROI_CFG):
        with open(ROI_CFG, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        rois = {k: tuple(map(int, v)) for k, v in data.items()}
        expected = {"N", "S", "E", "W"}
        if set(rois) != expected:
            raise RuntimeError(
                f"ROI config must contain exactly N, S, E, and W keys (found {sorted(rois)})."
            )
        for name, rect in rois.items():
            if len(rect) != 4:
                raise RuntimeError(
                    f"ROI '{name}' must be a 4-tuple of (x, y, w, h); got {rect}."
                )
        return rois

    print("Select ROIs in order N, S, E, W (ENTER after each, ESC when done).")
    rois = cv2.selectROIs("Select 4 ROIs", frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow("Select 4 ROIs")
    if len(rois) != 4:
        raise RuntimeError("Need exactly 4 ROIs.")
    mapping = {
        "N": tuple(map(int, rois[0])),
        "S": tuple(map(int, rois[1])),
        "E": tuple(map(int, rois[2])),
        "W": tuple(map(int, rois[3])),
    }
    with open(ROI_CFG, "w", encoding="utf-8") as fh:
        json.dump(mapping, fh, indent=2)
    print(f"Saved ROI configuration to {ROI_CFG}.")
    return mapping


def _make_states(rois: Dict[str, Tuple[int, int, int, int]]) -> Dict[str, ROIState]:
    states: Dict[str, ROIState] = {}
    for name, rect in rois.items():
        subtractor = cv2.createBackgroundSubtractorMOG2(
            history=500, varThreshold=25, detectShadows=False
        )
        states[name] = ROIState(name=name, rect=rect, subtractor=subtractor)
    return states


def _detect(state: ROIState, roi_frame) -> List[Tuple[int, int, int, int]]:
    gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
    mask = state.subtractor.apply(gray)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes: List[Tuple[int, int, int, int]] = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_AREA:
            continue
        box = cv2.boundingRect(contour)
        boxes.append(box)
    return boxes


def _draw_roi(display, state: ROIState, boxes: List[Tuple[int, int, int, int]]):
    x, y, w, h = state.rect
    cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
    for bx, by, bw, bh in boxes:
        gx, gy = x + bx, y + by
        cv2.rectangle(display, (gx, gy), (gx + bw, gy + bh), (255, 0, 0), 2)


def _emit_counts(counts: CountsDict) -> None:
    try:
        _counts_queue.put_nowait(counts.copy())
    except queue.Full:
        try:
            _counts_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            _counts_queue.put_nowait(counts.copy())
        except queue.Full:
            pass


def count_stream(cam_index: int = CAM_INDEX) -> Iterator[CountsDict]:
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open camera index {cam_index}.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    ok, frame = cap.read()
    if not ok:
        cap.release()
        raise RuntimeError("Unable to read initial frame.")

    states = _make_states(_select_or_load_rois(frame))
    last_emit = time.time()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            current_counts: CountsDict = {name: 0 for name in states}
            for name, state in states.items():
                x, y, w, h = state.rect
                roi_frame = frame[y : y + h, x : x + w]
                if roi_frame.size == 0:
                    continue
                boxes = _detect(state, roi_frame)
                current_counts[name] = len(boxes)
            now = time.time()
            if now - last_emit >= PRINT_EVERY:
                _emit_counts(current_counts)
                print(
                    f"Counts @ {time.strftime('%H:%M:%S')}: "
                    f"N={current_counts['N']} S={current_counts['S']} "
                    f"E={current_counts['E']} W={current_counts['W']}"
                )
                yield current_counts.copy()
                last_emit = now
    finally:
        cap.release()


def run_viewer(cam_index: int = CAM_INDEX) -> None:
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        raise RuntimeError(f"Unable to open camera index {cam_index}.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    ok, frame = cap.read()
    if not ok:
        cap.release()
        raise RuntimeError("Unable to read initial frame.")

    states = _make_states(_select_or_load_rois(frame))

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            display = frame.copy()
            for name, state in states.items():
                x, y, w, h = state.rect
                roi_frame = frame[y : y + h, x : x + w]
                if roi_frame.size == 0:
                    continue
                boxes = _detect(state, roi_frame)
                _draw_roi(display, state, boxes)
                cv2.putText(
                    display,
                    f"{name}: {len(boxes)}",
                    (x + 5, y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )
            cv2.imshow("Smart Road - ROIs", display)
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run_viewer()
