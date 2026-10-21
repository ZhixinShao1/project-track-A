import cv2
import numpy as np


class ArucoDetector:

    def __init__(
        self,
        camera_matrix,
        distortion_coefficients,
        marker_size,
        target_id=0
    ):

        self.target_id = target_id

        self.camera_matrix = camera_matrix

        self.distortion_coefficients = (
            distortion_coefficients
        )

        self.marker_size = marker_size

        self.dictionary = (
            cv2.aruco.getPredefinedDictionary(
                cv2.aruco.DICT_4X4_50
            )
        )

        self.parameters = (
            cv2.aruco.DetectorParameters()
        )

        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary,
            self.parameters
        )

        # Marker corner coordinates
        half = marker_size / 2.0

        self.object_points = np.array(
            [
                [-half,  half, 0.0],
                [ half,  half, 0.0],
                [ half, -half, 0.0],
                [-half, -half, 0.0]
            ],
            dtype=np.float32
        )


    def detect(self, image):

        annotated_image = image.copy()

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY
        )

        corners, ids, _ = (
            self.detector.detectMarkers(
                gray
            )
        )

        detected = False
        target_corners = None

        range_m = None
        bearing_rad = None

        rvec = None
        tvec = None

        if ids is not None:

            cv2.aruco.drawDetectedMarkers(
                annotated_image,
                corners,
                ids
            )

            for index, marker_id in enumerate(
                ids.flatten()
            ):

                if marker_id == self.target_id:

                    detected = True

                    target_corners = (
                        corners[index]
                    )

                    image_points = (
                        target_corners
                        .reshape(4, 2)
                        .astype(np.float32)
                    )

                    success, rvec, tvec = (
                        cv2.solvePnP(
                            self.object_points,
                            image_points,
                            self.camera_matrix,
                            self.distortion_coefficients,
                            flags=cv2.SOLVEPNP_IPPE_SQUARE
                        )
                    )

                    if success:

                        # Marker position in OpenCV
                        # camera coordinates:
                        #
                        # x = right
                        # y = down
                        # z = forward

                        marker_x = float(
                            tvec[0, 0]
                        )

                        marker_z = float(
                            tvec[2, 0]
                        )

                        # Planar range
                        range_m = np.sqrt(
                            marker_x ** 2
                            +
                            marker_z ** 2
                        )

                        # Horizontal bearing
                        bearing_rad = np.arctan2(
                            marker_x,
                            marker_z
                        )

                        cv2.drawFrameAxes(
                            annotated_image,
                            self.camera_matrix,
                            self.distortion_coefficients,
                            rvec,
                            tvec,
                            self.marker_size * 0.5
                        )

                        cv2.putText(
                            annotated_image,
                            f"ID: {marker_id}",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2
                        )

                        cv2.putText(
                            annotated_image,
                            f"Range: {range_m:.2f} m",
                            (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2
                        )

                        cv2.putText(
                            annotated_image,
                            (
                                f"Bearing: "
                                f"{np.degrees(bearing_rad):.1f} deg"
                            ),
                            (10, 80),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2
                        )

                    break

        return (
            annotated_image,
            detected,
            target_corners,
            range_m,
            bearing_rad
        )