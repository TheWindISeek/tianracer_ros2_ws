import math
import sys
from typing import List, Tuple

import matplotlib.pyplot as plt


def parse_positions(path: str) -> Tuple[List[float], List[float]]:
    """Parse all position.x / position.y from an odom.log file."""
    xs: List[float] = []
    ys: List[float] = []

    with open(path, "r") as f:
        lines = f.readlines()

    in_position = False
    x = None
    y = None

    for raw in lines:
        line = raw.strip()

        if line.startswith("position:"):
            in_position = True
            x = None
            y = None
            continue

        if in_position:
            if line.startswith("x:"):
                try:
                    x = float(line.split(":", 1)[1])
                except ValueError:
                    x = None
            elif line.startswith("y:"):
                try:
                    y = float(line.split(":", 1)[1])
                except ValueError:
                    y = None

            if x is not None and y is not None:
                xs.append(x)
                ys.append(y)
                in_position = False

    return xs, ys


def quat_zw_to_yaw(z: float, w: float) -> float:
    """Compute yaw from quaternion (z, w) when x=y=0."""
    return 2.0 * math.atan2(z, w)


def parse_yaws(path: str) -> List[float]:
    """Parse yaw (radians) for each frame from an odom.log file."""
    yaws: List[float] = []

    with open(path, "r") as f:
        lines = f.readlines()

    in_orientation = False
    z = None
    w = None

    for raw in lines:
        line = raw.strip()

        if line.startswith("orientation:"):
            in_orientation = True
            z = None
            w = None
            continue

        if in_orientation:
            if line.startswith("z:"):
                try:
                    z = float(line.split(":", 1)[1])
                except ValueError:
                    z = None
            elif line.startswith("w:"):
                try:
                    w = float(line.split(":", 1)[1])
                except ValueError:
                    w = None

            if z is not None and w is not None:
                yaws.append(quat_zw_to_yaw(z, w))
                in_orientation = False

    return yaws


def normalize_angle(a: float) -> float:
    """Normalize an angle into [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def compute_delta_yaws(yaws: List[float]) -> List[float]:
    """Compute yaw difference Δyaw between consecutive frames."""
    deltas: List[float] = []
    for i in range(1, len(yaws)):
        dyaw = normalize_angle(yaws[i] - yaws[i - 1])
        deltas.append(dyaw)
    return deltas


def find_flat_yaw_segments(
    yaws: List[float],
    max_delta_rad: float = 1e-3,
    min_length: int = 50,
) -> List[Tuple[int, int, float]]:
    """
    Find approximately horizontal/constant yaw segments.

    Returns a list of (start_index, end_index, mean_yaw_rad),
    where indices are inclusive and refer to the original yaw array.

    - max_delta_rad: maximum |Δyaw| (per step) inside a "flat" segment.
    - min_length: minimal number of frames to keep the segment.
    """
    n = len(yaws)
    if n == 0:
        return []

    segments: List[Tuple[int, int, float]] = []
    start = 0

    for i in range(1, n):
        dyaw = normalize_angle(yaws[i] - yaws[i - 1])
        if abs(dyaw) > max_delta_rad:
            # close previous segment [start, i-1]
            length = i - start
            if length >= min_length:
                seg_yaws = yaws[start:i]
                mean_yaw = sum(seg_yaws) / len(seg_yaws)
                segments.append((start, i - 1, mean_yaw))
            start = i

    # last segment
    length = n - start
    if length >= min_length:
        seg_yaws = yaws[start:n]
        mean_yaw = sum(seg_yaws) / len(seg_yaws)
        segments.append((start, n - 1, mean_yaw))

    return segments


def main() -> None:
    if len(sys.argv) > 1:
        log_path = sys.argv[1]
    else:
        log_path = "odom.log"

    xs, ys = parse_positions(log_path)
    yaws = parse_yaws(log_path)

    n = min(len(xs), len(ys), len(yaws))
    xs = xs[:n]
    ys = ys[:n]
    yaws = yaws[:n]

    if n < 2:
        print("Not enough data, at least 2 odom frames are required.")
        return

    deltas = compute_delta_yaws(yaws)

    print(f"Parsed {n} odom frames.")

    # 1) Collect and print unique yaw values that appeared
    unique_yaws = sorted(set(yaws))
    print(f"Found {len(unique_yaws)} unique yaw values.")

    # 2) Find and report "flat" yaw segments (the horizontal parts in the plot)
    flat_segments = find_flat_yaw_segments(
        yaws,
        max_delta_rad=1e-3,  # ≈ 0.057 deg per step
        min_length=50,       # only keep long, clearly horizontal parts
    )
    if flat_segments:
        print("\nDetected flat yaw segments (horizontal parts of the curve):")
        for idx, (start_i, end_i, mean_yaw) in enumerate(flat_segments, start=1):
            length = end_i - start_i + 1
            print(
                f"Segment {idx}: frames [{start_i}..{end_i}] "
                f"(len={length}), yaw ≈ {mean_yaw:+.6f} rad "
                f"({math.degrees(mean_yaw):+8.3f} deg)"
            )
    else:
        print("\nNo sufficiently long flat yaw segments found with current thresholds.")

    # 画图：子图1 轨迹，子图2 yaw 和 Δyaw
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # 轨迹
    ax1.plot(xs, ys, marker=".", linewidth=1)
    ax1.plot(xs[0], ys[0], "go", label="start")
    ax1.plot(xs[-1], ys[-1], "ro", label="end")
    ax1.set_aspect("equal", adjustable="box")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.set_title("Odom trajectory")
    ax1.grid(True)
    ax1.legend()

    # yaw / Δyaw
    steps = list(range(n))
    ax2.plot(steps, [math.degrees(y) for y in yaws], label="yaw [deg]")
    steps_delta = list(range(1, n))
    ax2.plot(
        steps_delta,
        [math.degrees(d) for d in deltas],
        label="Δyaw [deg/step]",
        alpha=0.7,
    )
    ax2.set_xlabel("frame index")
    ax2.set_ylabel("Angle [deg]")
    ax2.set_title("Yaw and yaw change per step")
    ax2.grid(True)
    ax2.legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()

