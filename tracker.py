import numpy as np
import cv2
import cv2.aruco as aruco

import pyigtl
import time

from scipy.spatial.transform import Rotation

# .51 rms x 21 images using `calib.ipynb`
mtx = np.array(
    [
        [903.9807830312084, 0.0, 631.032286614575],
        [0.0, 900.4801905267574, 336.07924714162726],
        [0.0, 0.0, 1.0],
    ]
)

dist = np.array(
    [
        [
            0.044630681992044105,
            -0.06477790812848058,
            0.0005125999649508189,
            -0.0021640214399763695,
            -0.006851598275562698,
        ]
    ]
)

marker_len = 1.26 / 100  # cm to m
marker_separation = 0.7 * marker_len
num_markers_x = 5
num_markers_y = 5
arucoboard = cv2.aruco.GridBoard(
    (num_markers_x, num_markers_y),
    marker_len,
    marker_separation,
    cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
)


def main():
    server = pyigtl.OpenIGTLinkServer(port=18944)

    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

    while True:
        if not server.is_connected():
            time.sleep(1)
            print("sleeping")
            continue

        _ret, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        aruco_dict = arucoboard.getDictionary()
        parameters = aruco.DetectorParameters()
        parameters.useAruco3Detection = True
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = aruco.ArucoDetector(aruco_dict, parameters)

        # ids and list of four corner points corr. to each id
        # Do NOT try to undistort image before marker detection, detection will be drastically distorted
        # gray = cv2.undistort(gray, mtx, dist)
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
        detector.refineDetectedMarkers(
            gray, arucoboard, corners, ids, rejectedImgPoints, mtx, dist
        )

        font = cv2.FONT_HERSHEY_SIMPLEX

        if np.all(ids != None):
            objectPoints, imagePoints = arucoboard.matchImagePoints(corners, ids)
            try:
                _, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, mtx, dist)
            except cv2.error:
                print("=====ignoring cv2.error=====")
                continue

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            if objectPoints.shape[0] // 4 > 0:
                cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, marker_len * 2, 2)
                cv2.putText(
                    frame,
                    "tvec" + str(tvec * 1000),
                    (0, 64),
                    font,
                    1,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    "rpy"
                    + str(
                        Rotation.from_matrix(cv2.Rodrigues(rvec)[0][:3, :3]).as_euler(
                            "xyz"
                        )
                    ),
                    (0, 64 * 2),
                    font,
                    1,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            # mm for slicer
            x_n = 21.7
            y_n = 1
            z_n = 145
            tip_to_cam = np.array(
                [[1, 0, 0, x_n], [0, 1, 0, y_n], [0, 0, 1, z_n], [0, 0, 0, 1]],
                dtype=float,
            )

            cam_to_world = np.linalg.inv(vector_to_matrix(tvec * 1000, rvec))
            pose = cam_to_world.dot(tip_to_cam)

            pos_msg = pyigtl.TransformMessage(
                matrix=pose,
                timestamp=time.time(),
                device_name="Position",
            )

            pose_vec = Rotation.from_matrix(cam_to_world[:3, :3]).as_euler("xyz")
            cv2.putText(
                frame,
                f"P_inv{cam_to_world[:3, 3]}",
                (0, 64 * 5),
                font,
                1,
                (255, 0, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                "rpy_inv" + str(pose_vec),
                (0, 64 * 6),
                font,
                1,
                (255, 0, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"P_inv_needle{pose[:3, 3]}",
                (0, 64 * 7),
                font,
                1,
                (255, 0, 0),
                2,
                cv2.LINE_AA,
            )

            cv2.circle(frame, (1920 // 2, 1080 // 2), 10, (255, 0, 0), 2)
        else:
            cv2.putText(frame, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            pos_msg = pyigtl.TransformMessage(
                np.identity(4), timestamp=time.time(), device_name="Position"
            )

        print(pos_msg)
        server.send_message(pos_msg, wait=True)

        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


"""
we want 4x4 homogeneous translation-rotation matrix:

. . . x
. . . y
. . . z
0 0 0 1

where . is rotation matrix rvec and [x, y, z]^T is tvec

Source: https://answers.ros.org/question/314828/opencv-camera-rvec-tvec-to-ros-world-pose/
"""


def vector_to_matrix(
    tvec: cv2.typing.MatLike, rvec: cv2.typing.MatLike
) -> cv2.typing.MatLike:
    needle_pos = np.array(
        [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]], dtype=float
    )

    # Rodrigues needed to turn rvec into a rot mat
    # https://stackoverflow.com/questions/53277597/fundamental-understanding-of-tvecs-rvecs-in-opencv-aruco
    # rvec is a compact Rodrigues vector of form [a, b, c] rather than [theta, x, y, z], so it's not represented in Euler angles
    # https://stackoverflow.com/questions/12933284/rodrigues-into-eulerangles-and-vice-versa
    needle_pos[:3, :3], _ = cv2.Rodrigues(rvec)

    # convert to mm for slicer
    needle_pos[:3, 3] = tvec[:, 0]

    return needle_pos


if __name__ == "__main__":
    main()
