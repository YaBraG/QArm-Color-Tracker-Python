import time
import cv2
import numpy as np

from qarm import QArmRealSense


def resize_for_display(image: np.ndarray, width: int = 640, height: int = 480) -> np.ndarray:
    """
    Resize an image for OpenCV display.
    """
    return cv2.resize(image, (width, height))


def depth_m_to_display(depth_m, min_depth=0.15, max_depth=2.0):
    """
    Convert physical depth image in meters to an 8-bit display image.

    Invalid depth pixels become black.
    Valid depth is scaled using a fixed range so the image does not flicker.

    Near = brighter
    Far  = darker
    """
    depth = np.squeeze(depth_m).astype(np.float32)

    valid = np.isfinite(depth) & (depth > min_depth) & (depth < max_depth)

    display = np.zeros(depth.shape, dtype=np.uint8)

    clipped = np.clip(depth, min_depth, max_depth)

    scaled = 255.0 * (1.0 - (clipped - min_depth) / (max_depth - min_depth))

    display[valid] = scaled[valid].astype(np.uint8)

    return display


def depth_px_to_display(depth_px: np.ndarray) -> np.ndarray:
    """
    Convert virtual depth pixel image to an 8-bit display image without cv2.normalize().
    This avoids Pylance/OpenCV type warnings.
    """
    depth = np.squeeze(depth_px).astype(np.float32)

    min_value = float(np.min(depth))
    max_value = float(np.max(depth))

    if max_value <= min_value:
        return np.zeros(depth.shape, dtype=np.uint8)

    scaled = 255.0 * (depth - min_value) / (max_value - min_value)

    return scaled.astype(np.uint8)


def main():
    physical_camera = None
    virtual_camera = None

    last_physical_rgb = None
    last_physical_depth_display = None
    last_virtual_rgb = None
    last_virtual_depth_display = None

    try:
        print("Opening physical QArm RealSense...")
        physical_camera = QArmRealSense(
            hardware=1,
            deviceID=0,
            mode="RGB&DEPTH",
            frameWidthRGB=640,
            frameHeightRGB=480,
            frameRateRGB=30,
            frameWidthDepth=640,
            frameHeightDepth=480,
            frameRateDepth=30,
            readMode=1
        )

        print("Opening virtual QArm camera from QLabs...")
        virtual_camera = QArmRealSense(
            hardware=0,
            deviceID=0,
            videoPort=18901,
            mode="RGB&DEPTH",
            readMode=1
        )

        print("Camera bridge running.")
        print("Press q in an OpenCV window to stop.")

        while True:
            # -----------------------------
            # Physical camera
            # -----------------------------
            physical_rgb_timestamp = physical_camera.read_RGB()
            physical_depth_timestamp = physical_camera.read_depth(dataMode="M")

            if physical_rgb_timestamp != -1:
                last_physical_rgb = physical_camera.imageBufferRGB.copy()

            if physical_depth_timestamp != -1:
                physical_depth_m = physical_camera.imageBufferDepthM.copy()

                new_depth_display = depth_m_to_display(
                    physical_depth_m,
                    min_depth=0.15,
                    max_depth=2.0
                )

                # Only update if the depth frame has some valid visible data.
                # This prevents random black blinking.
                if np.count_nonzero(new_depth_display) > 50:
                    last_physical_depth_display = new_depth_display

            # -----------------------------
            # Virtual camera
            # -----------------------------
            virtual_rgb_timestamp = virtual_camera.read_RGB()
            virtual_depth_timestamp = virtual_camera.read_depth(dataMode="PX")

            if virtual_rgb_timestamp != -1:
                last_virtual_rgb = virtual_camera.imageBufferRGB.copy()

            if virtual_depth_timestamp != -1:
                virtual_depth_px = virtual_camera.imageBufferDepthPX.copy()
                last_virtual_depth_display = depth_px_to_display(virtual_depth_px)

            # -----------------------------
            # Display frames
            # -----------------------------
            if last_physical_rgb is not None:
                cv2.imshow(
                    "Physical QArm RGB",
                    resize_for_display(last_physical_rgb, 640, 480)
                )

            if last_physical_depth_display is not None:
                cv2.imshow(
                    "Physical QArm Depth",
                    resize_for_display(last_physical_depth_display, 640, 480)
                )

            if last_virtual_rgb is not None:
                cv2.imshow(
                    "Virtual QArm RGB",
                    resize_for_display(last_virtual_rgb, 640, 480)
                )

            if last_virtual_depth_display is not None:
                cv2.imshow(
                    "Virtual QArm Depth",
                    resize_for_display(last_virtual_depth_display, 640, 480)
                )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Stopping camera bridge.")

    finally:
        if physical_camera is not None:
            physical_camera.terminate()

        if virtual_camera is not None:
            virtual_camera.terminate()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()