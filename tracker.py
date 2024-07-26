import numpy as np
import cv2
import cv2.aruco as aruco

import pyigtl
import time

# .69 rms x 18 images using `calib.ipynb`
mtx = np.array(
    [
        [1147.6416272659, 0.0, 954.670271690248],
        [0.0, 1149.1872573025537, 520.9501731260477],
        [0.0, 0.0, 1.0],
    ]
)

dist = np.array(
    [
        [
            -0.433043102642359,
            0.2737107734158141,
            -0.003438334732745021,
            0.0008411898428756419,
            -0.10775332214627145,
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
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
        detector.refineDetectedMarkers(
            gray, arucoboard, corners, ids, rejectedImgPoints, mtx, dist
        )

        font = cv2.FONT_HERSHEY_SIMPLEX

        if np.all(ids != None):
            # aruco.drawDetectedMarkers(frame, corners, ids)

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
                    frame, str(tvec[0]), (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA
                )
                cv2.putText(
                    frame,
                    str(tvec[1]),
                    (0, 64 * 2),
                    font,
                    1,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    str(tvec[2]),
                    (0, 64 * 3),
                    font,
                    1,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    str(rvec),
                    (0, 64 * 4),
                    font,
                    1,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            x_n = (-0.01599 - 0.006) * 1000
            y_n = 0
            z_n = -0.1674 * 1000
            cam_to_tip = np.array(
                [[1, 0, 0, x_n], [0, 1, 0, y_n], [0, 0, 1, z_n], [0, 0, 0, 1]],
                dtype=float,
            )

            pose = np.matmul(cam_to_tip, vector_to_matrix(tvec, rvec))
            pos_msg = pyigtl.TransformMessage(
                matrix=pose,
                timestamp=time.time(),
                device_name="Position",
            )

            cv2.putText(
                frame,
                f"{pose[0][3]}, {pose[1][3]}, {pose[2][3]}",
                (0, 64 * 5),
                font,
                1,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            cv2.circle(frame, (1920 // 2, 1080 // 2), 10, (255, 0, 0), 2)
        else:
            # code to show 'No Ids' when no markers are found
            cv2.putText(frame, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            pos_msg = pyigtl.TransformMessage(
                np.identity(4), timestamp=time.time(), device_name="Position"
            )

        print(pos_msg)
        server.send_message(pos_msg, wait=True)

        # display the resulting frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


"""
we want 4x4 but it's just composed of the homogeneous 
translation and rotation matrix like this (in form of [R|t]):

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

    rvec[2] = 0

    # Rodrigues needed to turn rvec into a rot mat!
    # https://stackoverflow.com/questions/53277597/fundamental-understanding-of-tvecs-rvecs-in-opencv-aruco
    needle_pos[:3, :3], _ = cv2.Rodrigues(rvec)

    # convert to mm for slicer
    needle_pos[:3, 3] = tvec[:, 0] * 1000
    return needle_pos


if __name__ == "__main__":
    main()
