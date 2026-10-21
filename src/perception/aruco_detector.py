import cv2


class ArucoDetector:

    def __init__(self, target_id=0):

        self.target_id = target_id

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        self.parameters = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(
            self.dictionary,
            self.parameters
        )


    def detect(self, image):

        annotated_image = image.copy()

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY
        )

        corners, ids, rejected = self.detector.detectMarkers(
            gray
        )

        detected = False
        target_corners = None

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
                    target_corners = corners[index]

                    cv2.putText(
                        annotated_image,
                        f"Target ArUco ID: {marker_id}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )

                    break

        return (
            annotated_image,
            detected,
            target_corners
        )