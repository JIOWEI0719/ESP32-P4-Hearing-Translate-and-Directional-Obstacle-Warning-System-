import argparse
import time
from datetime import datetime
from pathlib import Path

import cv2


LABELS = [
    "you",
    "me",
    "can",
    "have",
    "what",
    "good",
    "help",
    "thanks",
    "name",
    "sign_language",
    "a_little",
    "able",
]

LABEL_KEYS = {
    ord("1"): 0,
    ord("2"): 1,
    ord("3"): 2,
    ord("4"): 3,
    ord("5"): 4,
    ord("6"): 5,
    ord("7"): 6,
    ord("8"): 7,
    ord("9"): 8,
    ord("0"): 9,
    ord("-"): 10,
    ord("="): 11,
}


def center_square_crop(frame):
    height, width = frame.shape[:2]
    side = min(width, height)
    x0 = (width - side) // 2
    y0 = (height - side) // 2
    return frame[y0 : y0 + side, x0 : x0 + side], (x0, y0, side, side)


def prepare_frame_for_save(frame, size, save_full, save_color):
    if save_full:
        image = frame
    else:
        crop, _ = center_square_crop(frame)
        image = cv2.resize(crop, (size, size), interpolation=cv2.INTER_AREA)

    if save_color:
        return image
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def next_index(label_dir):
    existing = sorted(label_dir.glob("*.jpg"))
    return len(existing) + 1


def save_frame(frame, output_dir, label, index, size, save_full, save_color):
    label_dir = output_dir / label
    label_dir.mkdir(parents=True, exist_ok=True)

    image = prepare_frame_for_save(
        frame,
        size=size,
        save_full=save_full,
        save_color=save_color,
    )
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    path = label_dir / f"{label}_{index:06d}_{stamp}.jpg"
    cv2.imwrite(str(path), image, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    return path


def draw_overlay(frame, label, capture_due_at, counts, output_dir, delay, size, save_full):
    preview = frame.copy()
    height, width = preview.shape[:2]

    if not save_full:
        x, y, side, _ = center_square_crop(frame)[1]
        cv2.rectangle(preview, (x, y), (x + side, y + side), (0, 255, 0), 2)

    now = time.monotonic()
    if capture_due_at is None:
        status = "READY"
    else:
        remaining = max(0.0, capture_due_at - now)
        status = f"CAPTURE IN {remaining:.1f}s"

    mode = "full frame" if save_full else f"center crop -> {size}x{size}"
    lines = [
        f"Label [{LABELS.index(label) + 1}]: {label}",
        f"Status: {status}    Delay: {delay:.1f}s    Mode: {mode}",
        f"Saved for label: {counts[label]}    Output: {output_dir}",
        "Keys: 1-9/0/-/= label | N/P next/prev | Space capture | S save | Q quit",
    ]

    cv2.rectangle(preview, (0, 0), (width, 110), (0, 0, 0), -1)
    for i, text in enumerate(lines):
        color = (0, 0, 255) if capture_due_at is not None and i == 1 else (255, 255, 255)
        cv2.putText(
            preview,
            text,
            (12, 26 + i * 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            color,
            2,
            cv2.LINE_AA,
        )

    return preview


def parse_args():
    parser = argparse.ArgumentParser(
        description="Collect gesture image dataset from an ESP32 RTSP stream."
    )
    parser.add_argument(
        "--url",
        default="rtsp://192.168.137.2:8554",
        help="RTSP stream URL.",
    )
    parser.add_argument(
        "--output",
        default="D:/gesture_dataset_fisheye_224",
        help="Dataset output directory.",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=1.0,
        help="Seconds to wait after pressing Space before saving one frame.",
    )
    parser.add_argument(
        "--size",
        type=int,
        default=224,
        help="Saved image size when center-crop mode is used.",
    )
    parser.add_argument(
        "--save-full",
        action="store_true",
        help="Save the full RTSP frame instead of center-cropped resized images.",
    )
    parser.add_argument(
        "--save-color",
        action="store_true",
        help="Save BGR color images. By default, images are saved as grayscale.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    for label in LABELS:
        (output_dir / label).mkdir(parents=True, exist_ok=True)

    counts = {label: next_index(output_dir / label) - 1 for label in LABELS}
    next_indices = {label: counts[label] + 1 for label in LABELS}

    cap = cv2.VideoCapture(args.url, cv2.CAP_FFMPEG)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open RTSP stream: {args.url}")

    current_label = LABELS[0]
    capture_due_at = None

    print("Dataset collector started.")
    print(f"RTSP: {args.url}")
    print(f"Output: {output_dir}")
    print("Labels:")
    for i, label in enumerate(LABELS):
        key_name = chr(next(key for key, index in LABEL_KEYS.items() if index == i))
        print(f"  {key_name}: {label}")
    print("Keys: 1-9/0/-/= label | N/P next/prev | Space capture after delay | S save now | Q quit")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("Frame read failed, retrying...")
            time.sleep(0.2)
            continue

        now = time.monotonic()
        if capture_due_at is not None and now >= capture_due_at:
            path = save_frame(
                frame,
                output_dir,
                current_label,
                next_indices[current_label],
                args.size,
                args.save_full,
                args.save_color,
            )
            counts[current_label] += 1
            next_indices[current_label] += 1
            capture_due_at = None
            print(f"Saved: {path}")

        preview = draw_overlay(
            frame,
            current_label,
            capture_due_at,
            counts,
            output_dir,
            args.delay,
            args.size,
            args.save_full,
        )
        cv2.imshow("ESP32 RTSP Dataset Collector", preview)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == ord("Q"):
            break
        if key == ord(" "):
            capture_due_at = time.monotonic() + args.delay
            print(f"Capture scheduled in {args.delay:.1f}s for label: {current_label}")
        if key == ord("s") or key == ord("S"):
            path = save_frame(
                frame,
                output_dir,
                current_label,
                next_indices[current_label],
                args.size,
                args.save_full,
                args.save_color,
            )
            counts[current_label] += 1
            next_indices[current_label] += 1
            print(f"Saved single frame: {path}")
        if key == ord("n") or key == ord("N"):
            current_label = LABELS[(LABELS.index(current_label) + 1) % len(LABELS)]
            capture_due_at = None
            print(f"Current label: {current_label}")
        if key == ord("p") or key == ord("P"):
            current_label = LABELS[(LABELS.index(current_label) - 1) % len(LABELS)]
            capture_due_at = None
            print(f"Current label: {current_label}")
        if key in LABEL_KEYS:
            current_label = LABELS[LABEL_KEYS[key]]
            capture_due_at = None
            print(f"Current label: {current_label}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
