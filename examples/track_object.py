import cv2
import numpy as np

# RGB camera stream
RGB_STREAM_URL = "http://localhost:9090/stream?topic=/object_detector/camera/image_color"


def nothing(x):
    """Dummy callback function for OpenCV trackbars."""
    pass


def main():
    print("Connecting to RGB MJPEG stream via cv2.VideoCapture...")

    # rgb_cap = cv2.VideoCapture(RGB_STREAM_URL)    # URL
    # rgb_cap = cv2.VideoCapture(2, cv2.CAP_DSHOW)  # Windows
    rgb_cap = cv2.VideoCapture(2)  # Ubuntu

    if not rgb_cap.isOpened():
        print("Error: Could not open RGB stream.")
        return

    # Main window
    cv2.namedWindow("RGB Camera & Mask", cv2.WINDOW_AUTOSIZE)

    # HSV tuning window
    cv2.namedWindow("HSV Tuner", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("HSV Tuner", 400, 250)

    # Default HSV values for bright yellow
    cv2.createTrackbar("Lower H", "HSV Tuner", 18, 179, nothing)
    cv2.createTrackbar("Upper H", "HSV Tuner", 32, 179, nothing)
    cv2.createTrackbar("Lower S", "HSV Tuner", 100, 255, nothing)
    cv2.createTrackbar("Upper S", "HSV Tuner", 255, 255, nothing)
    cv2.createTrackbar("Lower V", "HSV Tuner", 100, 255, nothing)
    cv2.createTrackbar("Upper V", "HSV Tuner", 255, 255, nothing)

    # Bounding box smoothing
    smooth_box = None
    alpha = 0.3

    # Morphological filtering kernels
    kernel_large = np.ones((7, 7), np.uint8)
    kernel_small = np.ones((3, 3), np.uint8)

    while True:
        ret_rgb, rgb_frame = rgb_cap.read()

        if not ret_rgb or rgb_frame is None:
            continue

        # Read HSV trackbar values
        lh = cv2.getTrackbarPos("Lower H", "HSV Tuner")
        ls = cv2.getTrackbarPos("Lower S", "HSV Tuner")
        lv = cv2.getTrackbarPos("Lower V", "HSV Tuner")

        uh = cv2.getTrackbarPos("Upper H", "HSV Tuner")
        us = cv2.getTrackbarPos("Upper S", "HSV Tuner")
        uv = cv2.getTrackbarPos("Upper V", "HSV Tuner")

        # 1. Convert RGB camera frame (BGR format in OpenCV) to HSV
        hsv = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2HSV)

        # 2. Generate color mask
        lower_yellow = np.array([lh, ls, lv], dtype=np.uint8)
        upper_yellow = np.array([uh, us, uv], dtype=np.uint8)

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 3. Remove noise and fill small gaps
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, kernel_large
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, kernel_small
        )

        # 4. Find contours in the mask
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        obj_center = None
        object_area = 0

        valid_contours = [
            c for c in contours if cv2.contourArea(c) > 100
        ]

        if valid_contours:
            # Select the largest detected object
            largest_contour = max(
                valid_contours, key=cv2.contourArea
            )

            object_area = cv2.contourArea(largest_contour)

            # Calculate object center
            M = cv2.moments(largest_contour)

            if M["m00"] > 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                obj_center = (cX, cY)

            # Calculate bounding box
            x, y, w, h = cv2.boundingRect(largest_contour)
            current_box = np.array([x, y, w, h], dtype=float)

            # Smooth bounding box using EMA
            if smooth_box is None:
                smooth_box = current_box
            else:
                smooth_box = (
                    alpha * current_box
                    + (1 - alpha) * smooth_box
                )

            sx, sy, sw, sh = map(int, smooth_box)

            # Draw bounding box on RGB frame
            cv2.rectangle(
                rgb_frame,
                (sx, sy),
                (sx + sw, sy + sh),
                (0, 0, 255),
                3
            )

            # Draw object center
            if obj_center is not None:
                cv2.circle(
                    rgb_frame,
                    obj_center,
                    5,
                    (0, 0, 255),
                    -1
                )

                cv2.putText(
                    rgb_frame,
                    f"Center: ({cX}, {cY})",
                    (sx, max(sy - 10, 25)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2
                )

            cv2.putText(
                rgb_frame,
                f"Area: {object_area:.0f} px",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

        else:
            # Reset smoothing when the object disappears
            smooth_box = None

            cv2.putText(
                rgb_frame,
                "Object not detected",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        # 5. Convert grayscale mask to BGR for display
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        # 6. Display RGB and mask side-by-side
        combined = np.hstack((rgb_frame, mask_colored))

        cv2.imshow("RGB Camera & Mask", combined)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    rgb_cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
