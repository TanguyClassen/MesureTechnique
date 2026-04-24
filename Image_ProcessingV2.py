"""
ME-301 Measurement Techniques — Image Processing
Submerged Water Jet Project

Script V2: Pendulum deflection angle extraction
Robust version for real experiment videos
"""

import os
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt


# =============================================================================
# PARAMETERS
# =============================================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

VIDEO_PATH = os.path.expanduser("~/Desktop/your_video.MOV")
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "results")

SPHERE_DIAMETER_MM = 20          # 20 / 30 / 40 depending on sphere used
THREAD_LENGTH_MM   = None        # optional: real thread length for validation only

TRANSIENT_FRAMES = 5
RESIZE_FACTOR = 0.5

SHOW_DEBUG_WINDOW = True
DEBUG_EVERY_N_FRAMES = 5

# Calibration
RULER_LENGTH_MM = 50.0
RULER_LENGTH_PX = None           # leave None for interactive calibration

# Detection robustness
RADIUS_TOL_RATIO = 0.12          # ±20% around expected radius
L_TOL_RATIO = 0.12               # ±12% around expected pendulum length
MAX_CENTER_JUMP_PX = 35          # max center jump between consecutive valid frames
MAX_RADIUS_JUMP_PX = 8           # max radius jump between consecutive valid frames

# Hough settings
HOUGH_DP = 1.2
HOUGH_MINDIST = 40
HOUGH_PARAM1 = 80
HOUGH_PARAM2 = 28                # lower = more sensitive, higher = stricter


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def resize_frame(frame, factor):
    if factor == 1.0:
        return frame
    h, w = frame.shape[:2]
    return cv2.resize(
        frame,
        (int(w * factor), int(h * factor)),
        interpolation=cv2.INTER_AREA
    )


def click_two_points(frame, window_title, instruction):
    coords = []
    display = frame.copy()

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(coords) < 2:
            coords.append((x, y))
            cv2.circle(display, (x, y), 5, (0, 255, 255), -1)
            if len(coords) == 2:
                cv2.line(display, coords[0], coords[1], (0, 255, 255), 2)
            cv2.imshow(window_title, display)
            print(f"  Point {len(coords)}: ({x}, {y})")

    cv2.putText(display, instruction, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
    cv2.imshow(window_title, display)
    cv2.setMouseCallback(window_title, on_click)

    while len(coords) < 2:
        cv2.waitKey(20)

    cv2.waitKey(300)
    cv2.destroyWindow(window_title)
    return coords


def click_one_point(frame, window_title, instruction):
    coords = []
    display = frame.copy()

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and not coords:
            coords.append((x, y))
            cv2.circle(display, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow(window_title, display)
            print(f"  Point selected: ({x}, {y})")

    cv2.putText(display, instruction, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
    cv2.imshow(window_title, display)
    cv2.setMouseCallback(window_title, on_click)

    while not coords:
        cv2.waitKey(20)

    cv2.waitKey(300)
    cv2.destroyWindow(window_title)
    return coords[0]


def select_roi(frame, window_title="Select ROI"):
    print("\n[Setup] Select ROI around the pendulum motion, then press ENTER or SPACE.")
    roi = cv2.selectROI(window_title, frame, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow(window_title)
    x, y, w, h = roi
    if w == 0 or h == 0:
        raise RuntimeError("ROI selection cancelled or invalid.")
    return int(x), int(y), int(w), int(h)


def preprocess_for_detection(frame_bgr):
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 2)

    # Helps a lot when sphere is bright/white and contrast varies
    gray = cv2.equalizeHist(gray)
    return gray


def detect_sphere_candidates(frame_bgr, min_r, max_r):
    gray = preprocess_for_detection(frame_bgr)

    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=HOUGH_DP,
        minDist=HOUGH_MINDIST,
        param1=HOUGH_PARAM1,
        param2=HOUGH_PARAM2,
        minRadius=min_r,
        maxRadius=max_r
    )

    if circles is None:
        return []

    circles = np.round(circles[0, :]).astype(int)
    return [(c[0], c[1], c[2]) for c in circles]


def choose_best_circle(candidates, attachment_xy, expected_L_px=None,
                       prev_center=None, prev_radius=None):
    if not candidates:
        return None

    ax, ay = attachment_xy

    scored = []
    for (cx, cy, r) in candidates:
        # Geometric consistency
        dx = cx - ax
        dy = cy - ay
        L_px = np.sqrt(dx**2 + dy**2)

        # Prefer points below the attachment and close to expected length
        score = 0.0

        # Penalise detections above pivot
        if cy < ay:
            score -= 500

        # Penalise horizontal absurdity a bit
        score -= 0.2 * abs(dx)

        if expected_L_px is not None:
            score -= abs(L_px - expected_L_px)

        if prev_center is not None:
            px, py = prev_center
            jump = np.sqrt((cx - px)**2 + (cy - py)**2)
            score -= 1.5 * jump

        if prev_radius is not None:
            score -= 2.0 * abs(r - prev_radius)

        # Slightly favour bigger circles
        score += 0.5 * r

        scored.append((score, (cx, cy, r)))

    scored.sort(key=lambda x: x[0], reverse=True)
    return scored[0][1]


def compute_angle(cx, cy, ax, ay):
    """
    Angle from vertical.
    Positive if sphere is to the right of the pivot.
    """
    dx = cx - ax
    dy = cy - ay
    return np.degrees(np.arctan2(dx, dy))


def draw_debug(vis, attachment_xy, sphere, theta=None, text=None, color=(0, 255, 0)):
    ax, ay = attachment_xy
    cx, cy, r = sphere

    cv2.circle(vis, (cx, cy), r, color, 2)
    cv2.circle(vis, (cx, cy), 3, (0, 0, 255), -1)
    cv2.circle(vis, (ax, ay), 4, (255, 255, 0), -1)
    cv2.line(vis, (ax, ay), (cx, cy), (255, 0, 0), 2)

    if theta is not None:
        cv2.putText(vis, f"theta = {theta:.2f} deg", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    if text is not None:
        cv2.putText(vis, text, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


# =============================================================================
# MAIN
# =============================================================================

def main():
    print("PYTHON USED :", sys.executable)
    print("VIDEO PATH  :", VIDEO_PATH)
    print("VIDEO EXISTS:", os.path.exists(VIDEO_PATH))
    print("OUTPUT DIR  :", OUTPUT_DIR)

    if not os.path.exists(VIDEO_PATH):
        raise FileNotFoundError(f"Video file not found: {VIDEO_PATH}")

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {VIDEO_PATH}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"Video: {total_frames} frames at {fps:.2f} fps")

    ret, first_frame = cap.read()
    if not ret:
        cap.release()
        raise RuntimeError("Could not read first frame.")

    first_small = resize_frame(first_frame, RESIZE_FACTOR)

    # -------------------------------------------------------------------------
    # STEP 1 — Calibration
    # -------------------------------------------------------------------------
    global RULER_LENGTH_PX
    if RULER_LENGTH_PX is None:
        print(f"\n[Calibration] Click the two ends of the {RULER_LENGTH_MM:.0f} mm ruler segment.")
        pts = click_two_points(
            first_small,
            "Calibration",
            f"Click both ends of the {RULER_LENGTH_MM:.0f} mm ruler segment"
        )
        RULER_LENGTH_PX = np.sqrt(
            (pts[1][0] - pts[0][0])**2 +
            (pts[1][1] - pts[0][1])**2
        )
        print(f"  Measured ruler length = {RULER_LENGTH_PX:.2f} px")
    else:
        print(f"[Calibration] Using preset RULER_LENGTH_PX = {RULER_LENGTH_PX:.2f} px")

    K = RULER_LENGTH_MM / RULER_LENGTH_PX   # mm / px in resized image
    expected_r_px = (SPHERE_DIAMETER_MM / 2.0) / K
    min_r = max(3, int((1.0 - RADIUS_TOL_RATIO) * expected_r_px))
    max_r = max(min_r + 1, int((1.0 + RADIUS_TOL_RATIO) * expected_r_px))

    print(f"Calibration factor K = {K:.5f} mm/px")
    print(f"Expected sphere radius in processed frames = {expected_r_px:.2f} px")
    print(f"Detection radius range = [{min_r}, {max_r}] px")

    # -------------------------------------------------------------------------
    # STEP 2 — Attachment point on full frame
    # -------------------------------------------------------------------------
    print("\n[Setup] Click the thread attachment point on the full image.")
    attachment_x, attachment_y = click_one_point(
        first_small,
        "Attachment point",
        "Click the thread attachment point"
          )
    print(f"Attachment point: ({attachment_x}, {attachment_y})")

    # -------------------------------------------------------------------------
    # STEP 3 — ROI around sphere motion only
    # -------------------------------------------------------------------------
    roi_x, roi_y, roi_w, roi_h = select_roi(first_small)
    print(f"ROI selected: x={roi_x}, y={roi_y}, w={roi_w}, h={roi_h}")

    # -------------------------------------------------------------------------
    # STEP 4 — First pass: estimate expected pendulum length
    # -------------------------------------------------------------------------
    print("\n[First pass] Estimating expected pendulum length...")
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    raw_lengths = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        small = resize_frame(frame, RESIZE_FACTOR)
        roi = small[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]

        candidates_roi = detect_sphere_candidates(roi, min_r, max_r)
        candidates_full = [(cx + roi_x, cy + roi_y, r) for (cx, cy, r) in candidates_roi]

        best = choose_best_circle(candidates_full, (attachment_x, attachment_y))
        if best is not None:
            cx, cy, r = best
            L_px = np.sqrt((cx - attachment_x)**2 + (cy - attachment_y)**2)
            if cy > attachment_y:
                raw_lengths.append(L_px)

    if not raw_lengths:
        cap.release()
        raise RuntimeError("No sphere detected during first pass.")

    L_expected_px = float(np.median(raw_lengths))
    L_tol_px = L_TOL_RATIO * L_expected_px

    print(f"Estimated pendulum length = {L_expected_px:.2f} px")
    print(f"Estimated pendulum length = {L_expected_px * K:.2f} mm")
    print(f"Accepted length tolerance = ±{L_tol_px:.2f} px")

    # -------------------------------------------------------------------------
    # STEP 5 — Second pass: robust extraction
    # -------------------------------------------------------------------------
    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    times = []
    angles = []
    L_px_list = []
    L_mm_list = []
    radius_list = []

    failed = 0
    rejected = 0
    frame_idx = 0

    prev_center = None
    prev_radius = None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        small = resize_frame(frame, RESIZE_FACTOR)
        roi = small[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]

        candidates_roi = detect_sphere_candidates(roi, min_r, max_r)
        candidates_full = [(cx + roi_x, cy + roi_y, r) for (cx, cy, r) in candidates_roi]

        best = choose_best_circle(
            candidates_full,
            (attachment_x, attachment_y),
            expected_L_px=L_expected_px,
            prev_center=prev_center,
            prev_radius=prev_radius
        )

        accepted = False
        reject_reason = ""

        if best is not None:
            cx, cy, r = best
            L_px = np.sqrt((cx - attachment_x)**2 + (cy - attachment_y)**2)

            # Physical / temporal consistency checks
            if cy <= attachment_y:
                reject_reason = "sphere above pivot"

            elif abs(L_px - L_expected_px) > L_tol_px:
                reject_reason = "length inconsistency"

            elif prev_center is not None:
                jump = np.sqrt((cx - prev_center[0])**2 + (cy - prev_center[1])**2)
                if jump > MAX_CENTER_JUMP_PX:
                    reject_reason = "center jump too large"

            if prev_radius is not None and reject_reason == "":
                if abs(r - prev_radius) > MAX_RADIUS_JUMP_PX:
                    reject_reason = "radius jump too large"

            if reject_reason == "":
                theta = compute_angle(cx, cy, attachment_x, attachment_y)
                L_mm = L_px * K

                times.append(frame_idx / fps)
                angles.append(theta)
                L_px_list.append(L_px)
                L_mm_list.append(L_mm)
                radius_list.append(r)

                prev_center = (cx, cy)
                prev_radius = r
                accepted = True
            else:
                rejected += 1
        else:
            failed += 1

        if SHOW_DEBUG_WINDOW and frame_idx % DEBUG_EVERY_N_FRAMES == 0:
            vis = small.copy()
            cv2.rectangle(vis, (roi_x, roi_y), (roi_x + roi_w, roi_y + roi_h), (0, 255, 255), 2)

            if best is not None:
                color = (0, 255, 0) if accepted else (0, 0, 255)
                txt = None if accepted else f"Rejected: {reject_reason}"
                theta_dbg = compute_angle(best[0], best[1], attachment_x, attachment_y) if accepted else None
                draw_debug(vis, (attachment_x, attachment_y), best, theta=theta_dbg, text=txt, color=color)
            else:
                cv2.circle(vis, (attachment_x, attachment_y), 4, (255, 255, 0), -1)
                cv2.putText(vis, "No detection", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow("Detection check", vis)
            cv2.waitKey(1)

        if frame_idx % 20 == 0:
            print(f"  Frame {frame_idx + 1}/{total_frames}...")

        frame_idx += 1

    cap.release()
    cv2.destroyAllWindows()

    print(f"\nProcessed {frame_idx} frames")
    print(f"  Failed detections : {failed}")
    print(f"  Rejected detections: {rejected}")
    print(f"  Accepted detections: {len(angles)}")

    angles = np.array(angles)
    times = np.array(times)
    L_px_list = np.array(L_px_list)
    L_mm_list = np.array(L_mm_list)
    radius_list = np.array(radius_list)

    if len(angles) == 0:
        raise RuntimeError("No valid sphere detections in the full video.")

    if len(angles) <= TRANSIENT_FRAMES:
        raise RuntimeError(
            f"Not enough valid frames after transient removal "
            f"(valid={len(angles)}, transient={TRANSIENT_FRAMES})."
        )

    # -------------------------------------------------------------------------
    # STEP 6 — Steady-state statistics
    # -------------------------------------------------------------------------
    angles_s = angles[TRANSIENT_FRAMES:]
    times_s = times[TRANSIENT_FRAMES:]
    L_px_s = L_px_list[TRANSIENT_FRAMES:]
    L_mm_s = L_mm_list[TRANSIENT_FRAMES:]
    radius_s = radius_list[TRANSIENT_FRAMES:]

    theta_mean = np.mean(angles_s)
    theta_std = np.std(angles_s, ddof=1)
    theta_se = theta_std / np.sqrt(len(angles_s))

    L_mm_mean = np.mean(L_mm_s)
    L_mm_std = np.std(L_mm_s, ddof=1)

    r_mean_px = np.mean(radius_s)
    r_std_px = np.std(radius_s, ddof=1)

    print("\n--- Angle results ---")
    print(f"  Mean deflection angle θ = {theta_mean:.3f} °")
    print(f"  Standard deviation      = {theta_std:.3f} °")
    print(f"  Standard error (SE)     = {theta_se:.4f} °")

    print("\n--- Geometric consistency ---")
    print(f"  Mean pendulum length    = {L_mm_mean:.2f} mm")
    print(f"  Length std              = {L_mm_std:.2f} mm")
    print(f"  Mean detected radius    = {r_mean_px:.2f} px")
    print(f"  Radius std              = {r_std_px:.2f} px")

    if THREAD_LENGTH_MM is not None:
        error_pct = abs(L_mm_mean - THREAD_LENGTH_MM) / THREAD_LENGTH_MM * 100
        print(f"  Real thread length      = {THREAD_LENGTH_MM:.2f} mm")
        print(f"  Relative length error   = {error_pct:.2f} %")

    # -------------------------------------------------------------------------
    # STEP 7 — Save CSV
    # -------------------------------------------------------------------------
    csv_path = os.path.join(OUTPUT_DIR, f"angles_d{SPHERE_DIAMETER_MM}mm_v2.csv")
    header = "time_s,angle_deg,L_px,L_mm,radius_px"
    data = np.column_stack([times_s, angles_s, L_px_s, L_mm_s, radius_s])
    np.savetxt(csv_path, data, delimiter=",", header=header, comments="")
    print(f"\nData saved → {csv_path}")

    # -------------------------------------------------------------------------
    # STEP 8 — Plot
    # -------------------------------------------------------------------------
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    # Angle
    ax1 = axes[0]
    ax1.plot(times_s, angles_s, linewidth=0.8, color="steelblue", label="θ per frame")
    ax1.axhline(theta_mean, linewidth=1.5, linestyle="--", color="navy",
                label=f"Mean θ = {theta_mean:.2f}°")
    ax1.fill_between(times_s, theta_mean - theta_std, theta_mean + theta_std,
                     alpha=0.15, color="steelblue", label="±1σ")
    ax1.set_ylabel("Angle θ (°)")
    ax1.set_title(f"Pendulum deflection — sphere d = {SPHERE_DIAMETER_MM} mm")
    ax1.grid(True, linestyle=":", alpha=0.5)
    ax1.legend()

    # Length
    ax2 = axes[1]
    ax2.plot(times_s, L_mm_s, linewidth=0.8, color="darkorange", label="Measured L")
    ax2.axhline(L_mm_mean, linewidth=1.5, linestyle="--", color="saddlebrown",
                label=f"Mean L = {L_mm_mean:.2f} mm")
    if THREAD_LENGTH_MM is not None:
        ax2.axhline(THREAD_LENGTH_MM, linewidth=1.2, linestyle=":",
                    color="black", label=f"Real L = {THREAD_LENGTH_MM:.2f} mm")
    ax2.set_ylabel("Length L (mm)")
    ax2.set_title("Pendulum length consistency")
    ax2.grid(True, linestyle=":", alpha=0.5)
    ax2.legend()

    # Radius
    ax3 = axes[2]
    ax3.plot(times_s, radius_s, linewidth=0.8, color="purple", label="Detected radius")
    ax3.axhline(r_mean_px, linewidth=1.5, linestyle="--", color="indigo",
                label=f"Mean r = {r_mean_px:.2f} px")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Radius (px)")
    ax3.set_title("Detected sphere radius consistency")
    ax3.grid(True, linestyle=":", alpha=0.5)
    ax3.legend()

    plt.tight_layout()

    plot_path = os.path.join(OUTPUT_DIR, f"angle_plot_d{SPHERE_DIAMETER_MM}mm_v2.png")
    plt.savefig(plot_path, dpi=150)
    print(f"Plot saved → {plot_path}")
    plt.show()


if __name__ == "__main__":
    main()