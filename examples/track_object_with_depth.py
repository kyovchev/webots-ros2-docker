import cv2
import numpy as np

SCALE = 0.5

# Stream URLs using native OpenCV VideoCapture
RGB_STREAM_URL = "http://localhost:9090/stream?topic=/object_detector/camera/image_color"
DEPTH_STREAM_URL = "http://localhost:9090/stream?topic=/object_detector/range_finder/image"


def nothing(x):
    """Dummy callback function for OpenCV trackbars."""
    pass


def main():
    print("Connecting to MJPEG streams via cv2.VideoCapture...")

    rgb_cap = cv2.VideoCapture(RGB_STREAM_URL)
    depth_cap = cv2.VideoCapture(DEPTH_STREAM_URL)

    if not rgb_cap.isOpened():
        print("Error: Could not open RGB stream.")
        return
    if not depth_cap.isOpened():
        print("Error: Could not open Depth stream.")
        return

    cv2.namedWindow("Robot Vision & Tracking", cv2.WINDOW_AUTOSIZE)

    # Create a separate window with trackbars for real-time HSV tuning
    cv2.namedWindow("HSV Tuner", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("HSV Tuner", 400, 250)

    # Initial default values for bright yellow (#ffff00)
    cv2.createTrackbar("Lower H", "HSV Tuner", 18, 179, nothing)
    cv2.createTrackbar("Upper H", "HSV Tuner", 32, 179, nothing)
    cv2.createTrackbar("Lower S", "HSV Tuner", 100, 255, nothing)
    cv2.createTrackbar("Upper S", "HSV Tuner", 255, 255, nothing)
    cv2.createTrackbar("Lower V", "HSV Tuner", 100, 255, nothing)
    cv2.createTrackbar("Upper V", "HSV Tuner", 255, 255, nothing)

    # Variables for smoothing bounding box jitter
    smooth_box = None
    alpha = 0.3  # Smoothing factor

    while True:
        ret_rgb, rgb_frame = rgb_cap.read()
        ret_depth, depth_frame = depth_cap.read()

        if not ret_rgb or rgb_frame is None:
            continue

        # Fetch current trackbar positions dynamically
        lh = cv2.getTrackbarPos("Lower H", "HSV Tuner")
        ls = cv2.getTrackbarPos("Lower S", "HSV Tuner")
        lv = cv2.getTrackbarPos("Lower V", "HSV Tuner")
        uh = cv2.getTrackbarPos("Upper H", "HSV Tuner")
        us = cv2.getTrackbarPos("Upper S", "HSV Tuner")
        uv = cv2.getTrackbarPos("Upper V", "HSV Tuner")

        # 1. Process RGB for Yellow Color Detection using trackbar values
        hsv = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([lh, ls, lv])
        upper_yellow = np.array([uh, us, uv])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Anti-flicker morphological filtering
        kernel_large = np.ones((7, 7), np.uint8)
        kernel_small = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_large)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small)

        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # Find contours directly from the filtered mask
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        obj_center = None
        if contours:
            valid_contours = [c for c in contours if cv2.contourArea(c) > 100]
            if valid_contours:
                largest_contour = max(valid_contours, key=cv2.contourArea)

                # Use mask moments for precise center of mass of the masked object region
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    obj_center = (cX, cY)

                x, y, w, h = cv2.boundingRect(largest_contour)

                # Apply Exponential Moving Average (EMA) smoothing to bounding box
                if smooth_box is None:
                    smooth_box = np.array([x, y, w, h], dtype=float)
                else:
                    smooth_box = alpha * \
                        np.array([x, y, w, h], dtype=float) + \
                        (1 - alpha) * smooth_box

                sx, sy, sw, sh = map(int, smooth_box)

                # Draw Red border around the object on the RGB frame
                cv2.rectangle(rgb_frame, (sx, sy),
                              (sx + sw, sy + sh), (0, 0, 255), 3)

                if obj_center:
                    cv2.circle(rgb_frame, obj_center, 5, (0, 0, 255), -1)
        else:
            smooth_box = None

        # 2. Process Depth Stream, Map Mask Center, and Fetch Distance
        target_height = rgb_frame.shape[0]  # Match RGB height (480px)
        distance_str = "Distance to Object: N/A"

        if ret_depth and depth_frame is not None:
            orig_h, orig_w = depth_frame.shape[:2]
            target_width = int(orig_w * (target_height / orig_h))

            depth_display = cv2.resize(
                depth_frame, (target_width, target_height), interpolation=cv2.INTER_NEAREST)

            if obj_center is not None:
                # Map RGB mask center coordinates down to Depth frame native resolution
                depth_x = int(obj_center[0] * (orig_w / rgb_frame.shape[1]))
                depth_y = int(obj_center[1] * (orig_h / rgb_frame.shape[0]))

                if 0 <= depth_x < orig_w and 0 <= depth_y < orig_h:
                    # Fetch depth value from the depth frame at the mask-derived location
                    raw_depth_pixel = depth_frame[depth_y, depth_x]
                    val = np.mean(raw_depth_pixel) if len(
                        raw_depth_pixel.shape) > 0 else raw_depth_pixel
                    distance_str = f"Distance to Object: {val/255:.2f}m"

                    # Map depth pixel position onto the resized display dimension for crosshair drawing
                    disp_depth_x = int(depth_x * (target_width / orig_w))
                    disp_depth_y = int(depth_y * (target_height / orig_h))

                    # Crosshair on the depth image corresponding to the mask center
                    cv2.drawMarker(depth_display, (disp_depth_x, disp_depth_y), (0, 0, 255),
                                   markerType=cv2.MARKER_CROSS, markerSize=15, thickness=2)
        else:
            depth_display = np.zeros((target_height, 640, 3), dtype=np.uint8)

        # 3. Combine panels side-by-side
        combined_streams = np.hstack((rgb_frame, mask_colored, depth_display))

        # 4. Header panel for distance readout
        header_height = 50
        total_width = combined_streams.shape[1]
        header_panel = np.zeros(
            (header_height, total_width, 3), dtype=np.uint8)
        cv2.putText(header_panel, distance_str, (20, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        final_window = np.vstack((header_panel, combined_streams))
        cv2.imshow("Robot Vision & Tracking", cv2.resize(
            final_window, None, fx=SCALE, fy=SCALE))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    rgb_cap.release()
    depth_cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
