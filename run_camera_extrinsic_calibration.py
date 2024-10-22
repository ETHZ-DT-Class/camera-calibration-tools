#!/usr/bin/env python3

from typing import Dict, Callable, List, Optional

import os
import cv2
import requests
import yaml
import rospy
import numpy as np
from sensor_msgs.msg import CameraInfo, CompressedImage
from turbojpeg import TurboJPEG, TJPF_BGR

from dt_computer_vision.camera import CameraModel, Pixel, BGRImage, NormalizedImagePoint
from dt_computer_vision.camera.calibration.extrinsics.boards import CalibrationBoard8by6, CalibrationBoard
from dt_computer_vision.camera.calibration.extrinsics.chessboard import \
    compute_homography_maps, find_corners, get_ground_corners_and_error
from dt_computer_vision.camera.calibration.extrinsics.exceptions import NoCornersFoundException
from dt_computer_vision.camera.calibration.extrinsics.ransac import estimate_homography
from dt_computer_vision.camera.calibration.extrinsics.rendering import draw_corners, \
    top_view_projected_corners, draw_gui, GUI_RIGHT_IMAGE_ROI, GUI_BTN1_ROI, GUI_BTN2_ROI, GUI_SIZE, \
    VALIDATION_GUI_BTN1_ROI, draw_validation_gui, VALIDATION_GUI_RIGHT_IMAGE_ROI
from dt_computer_vision.camera.homography import HomographyToolkit, ResolutionIndependentHomography, \
    ResolutionDependentHomography
from dt_computer_vision.camera.types import RegionOfInterest
from dt_computer_vision.ground_projection.types import GroundPoint


KNOWN_BOARDS: Dict[str, CalibrationBoard] = {
    "8x6": CalibrationBoard8by6,
}

VEHICLE_NAME = os.environ.get("VEHICLE_NAME")
TOPIC_IMAGE = f"/{VEHICLE_NAME}/camera_node/image/compressed"
TOPIC_CAMERA_INFO = f"/{VEHICLE_NAME}/camera_node/camera_info"


class CameraExtrinsicsCalibration:

    def __init__(self,
                 camera: CameraModel,
                 board: CalibrationBoard,
                 save: Callable[[ResolutionIndependentHomography], None] = None,
                 on_cancel: Callable[[], None] = None,
                 on_cancelled: Callable[[], None] = None,
                 on_save: Callable[[], None] = None,
                 on_saved: Callable[[], None] = None,
                 quiet: bool = True):
        # camera model
        self._camera: CameraModel = camera
        self._is_shutdown: bool = False
        self._was_cancelled: bool = False
        self._quiet: bool = quiet
        # board to use
        self._board: CalibrationBoard = board
        # store homography
        self._H: Optional[np.ndarray] = None
        self._error: Optional[float] = None
        # callbacks
        self._save: Optional[Callable[[ResolutionIndependentHomography], None]] = save
        self._on_cancel: Optional[Callable[[], None]] = on_cancel
        self._on_cancelled: Optional[Callable[[], None]] = on_cancelled
        self._on_save: Optional[Callable[[], None]] = on_save
        self._on_saved: Optional[Callable[[], None]] = on_saved
        # create window
        self._window = "Camera Extrinsic Calibration"
        self._mouse_registered = False

    @property
    def H(self) -> Optional[np.ndarray]:
        return self._H

    @property
    def is_shutdown(self) -> bool:
        return self._is_shutdown

    @property
    def was_cancelled(self) -> bool:
        return self._was_cancelled

    def shutdown(self):
        self._is_shutdown = True

    def on_new_frame(self, distorted: Optional[BGRImage]):
        if self.is_shutdown:
            return
        
        # no frame?
        if distorted is None:
            print("Is none")
            # display empty GUI
            gui = draw_gui(False, None, None, 0, self._board, None, self._error)
            # cv2.imshow(self._window, gui)
            cv2.waitKey(1)
            return

        # rectify image
        rectified: BGRImage = self._camera.rectifier.rectify(distorted, interpolation=cv2.INTER_CUBIC)

        # find corners
        corners: List[Pixel] = []
        try:
            corners = find_corners(rectified, self._board, win_size=3)
            board_found = True
        except NoCornersFoundException:
            board_found = False

        if board_found:
            # re-orient corners if necessary
            p0, p_1 = corners[0], corners[-1]
            cx, cy = self._camera.cx, self._camera.cy
            # we want the first point (red) to be to the left of the principal point
            if p0.x > cx and p_1.x < cx:
                corners = corners[::-1]

            # estimate homography
            H: np.ndarray = estimate_homography(corners, self._board, self._camera)
            self._H = H
            # print(H)

            # draw detected corners on top of the image
            image_w_corners: BGRImage = draw_corners(rectified, self._board, corners)

            # reproject all corners found in the image onto the ground plane and compute the errors
            image_corners: List[NormalizedImagePoint]
            ground_corners: List[GroundPoint]
            ground_corners_projected: List[GroundPoint]
            errors: List[float]

            image_corners, ground_corners, _, errors = \
                get_ground_corners_and_error(self._camera, corners, self._board, H)
            assert len(errors) == len(corners)

            # compute average error
            avg_error = np.average(errors)
            std_error = np.std(errors)
            if not self._quiet:
                print(f"Board detected, overall error: {avg_error:.4f}m +/- {std_error:.4f}m")


            # compute best error
            best_error: float = min(avg_error, self._error or avg_error)
            self._error = best_error

            # create re-projection image
            _, _, rw, rh = GUI_RIGHT_IMAGE_ROI
            right = top_view_projected_corners(ground_corners, errors, (rw, rh), start_y=0.15)

            # create GUI image
            gui = draw_gui(True, image_w_corners, right, len(corners), self._board, avg_error, best_error)
        else:
            # create GUI image
            active: bool = self._H is not None
            gui = draw_gui(active, rectified, None, 0, self._board, None, self._error)
        
        # display GUI
        cv2.imshow(self._window, gui)
        if not self._mouse_registered:
            cv2.setMouseCallback(self._window, self._gui_mouse_click)
            self._mouse_registered = True
        cv2.waitKey(500)

    def _save_calibration(self):
        if self._H is None:
            return
        # call callback
        if self._on_save:
            self._on_save()
        # stop printing
        self._quiet = True

        # # print calibration
        # H_str: str = yaml.safe_dump({'homography': self._H.flatten().tolist()})
        # print(f"Extrinsics calibration:\n-----------------------\n\n{H_str}\n\n-----------------------\n")

        # convert to resolution independent
        H_rd: ResolutionDependentHomography = ResolutionDependentHomography.read(self._H)
        H_ri: ResolutionIndependentHomography = H_rd.camera_independent(self._camera)
        # save
        if self._save:
            self._save(H_ri)
        # exit
        self._quit()
        # call callback
        if self._on_saved:
            self._on_saved()

    def _cancel(self):
        # call callback
        if self._on_cancel:
            self._on_cancel()
        # stop everything
        self._was_cancelled = True
        self._quit()
        # call callback
        if self._on_cancelled:
            self._on_cancelled()

    def _quit(self):
        self._is_shutdown = True
        # destroy windows on exit
        try:
            cv2.destroyAllWindows()
        except BaseException:
            pass

    def _gui_mouse_click(self, event, x, y, _, __):
        # check if left mouse click
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        # check which button was pressed
        btn_to_fcn = {
            GUI_BTN2_ROI: self._save_calibration,
            GUI_BTN1_ROI: self._cancel,
        }

        for btn, fcn in btn_to_fcn.items():
            bx, by, bw, bh = btn
            if bx < x < bx + bw and by < y < by + bh:
                fcn()
                print("End of mouse click func")
                return


class CameraExtrinsicsValidation:

    def __init__(self,
                 camera: CameraModel,
                 board: CalibrationBoard,
                 on_closed: Callable[[], None] = None,
                 pixels_per_meter: int = 800):
        assert camera.H is not None
        print("Starting to initialize validator... (can take up to a minute)")
        # camera model
        self._camera: CameraModel = camera
        # board to use
        self._board: CalibrationBoard = board
        # jpeg decoder
        self._jpeg = TurboJPEG()
        # other stuff
        self._is_shutdown: bool = False
        self._ppm: int = pixels_per_meter
        # callbacks
        self._on_closed: Optional[Callable[[], None]] = on_closed
        # create window
        self._window = "Camera Extrinsic Validation"
        self._mouse_registered = False
        # # compute the region of interest
        roi: RegionOfInterest = RegionOfInterest(origin=self._board.chessboard_offset, size=self._board.size)
        # create maps
        self._mapx, self._mapy, self._mask, _ = \
            compute_homography_maps(self._camera, self._camera.H, self._ppm, roi)
        print("Validator initialized successfully.")

    @property
    def is_shutdown(self) -> bool:
        return self._is_shutdown

    def shutdown(self):
        self._is_shutdown = True

    def on_new_frame(self, distorted: Optional[BGRImage]):
        if self.is_shutdown:
            return

        # no frame?
        if distorted is None:
            return

        # rectify image
        rectified: BGRImage = self._camera.rectifier.rectify(distorted, interpolation=cv2.INTER_CUBIC)

        # apply homography
        projected: BGRImage = cv2.remap(rectified, self._mapx, self._mapy, cv2.INTER_LINEAR)

        # fix missing pixels
        projected = cv2.inpaint(projected, self._mask, 3, cv2.INPAINT_NS)

        # flip and rotate the image so that it appears as it is seen from the camera
        projected = cv2.flip(projected, 0)
        projected = cv2.rotate(projected, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # resize projected
        _, _, w, h = VALIDATION_GUI_RIGHT_IMAGE_ROI
        projected = cv2.resize(projected, (w, h), cv2.INTER_CUBIC)

        # display GUI
        gui = draw_validation_gui(rectified, projected)
        cv2.imshow(self._window, gui)
        if not self._mouse_registered:
            cv2.setMouseCallback(self._window, self._gui_mouse_click)
            self._mouse_registered = True
        cv2.waitKey(500)

    def _quit(self):
        self._is_shutdown = True
        # destroy windows on exit
        try:
            cv2.destroyAllWindows()
        except BaseException:
            pass
        # call callback
        if self._on_closed:
            self._on_closed()

    def _gui_mouse_click(self, event, x, y, _, __):
        # check if left mouse click
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        # check if we clicked on the button
        bx, by, bw, bh = VALIDATION_GUI_BTN1_ROI
        if bx < x < bx + bw and by < y < by + bh:
            self._quit()


class CameraExtrinsicsCalibrationNode:

    def __init__(self):
        rospy.init_node("extrinsic_calibration_node", anonymous=False)

        # get the name of the robot
        self.veh = VEHICLE_NAME
        # static parameters
        self.board_name = "8x6"  # only supported board at the moment
        # select board
        if self.board_name not in KNOWN_BOARDS:
            rospy.logfatal(f"Unknown board '{self.board_name}'")
            exit(1)
        self.board: CalibrationBoard = KNOWN_BOARDS[self.board_name]
        # camera info
        self._camera: Optional[CameraModel] = None
        # calibrator object
        self._calibrator: Optional[CameraExtrinsicsCalibration] = None
        # validator object
        self._validator: Optional[CameraExtrinsicsValidation] = None
        # JPEG decoder
        self._jpeg = TurboJPEG()
        # subscribers
        self._img_sub = rospy.Subscriber(
            TOPIC_IMAGE, CompressedImage, self._img_cb, queue_size=1, buff_size=20*1024*1024  # 20MB in bytes
        )
        self._cinfo_sub = rospy.Subscriber(
            TOPIC_CAMERA_INFO, CameraInfo, self._cinfo_cb, queue_size=1
        )

    def _on_calibration_cancelled(self):
        rospy.loginfo("Calibration cancelled. Shutting down.")
        rospy.signal_shutdown("Calibration cancelled.")
        self._calibrator = None

    def _save_calibration(self, Hindep: ResolutionIndependentHomography):
        Hdep: ResolutionDependentHomography = Hindep.camera_specific(self._camera)
        
        H = Hdep @ self._camera.homography_pixel2vector()

        H_str: str = yaml.safe_dump({'homography': H.flatten().tolist()})
        print(f"Extrinsics calibration to be saved:\n-----------------------\n\n{H_str}\n\n-----------------------\n")
        
        # --- use Duckiebot's Files API to manipulate files
        # create URL
        url: str = f"http://{self.veh}.local/files/data/config/calibrations/camera_extrinsic/{self.veh}.yaml"
        # save homography
        requests.post(url, data=yaml.dump({"homography": H.flatten().tolist()}))

    def _on_calibration_saved(self):
        # add H to camera model
        self._camera.H = self._calibrator.H
        # create validator
        self._validator = CameraExtrinsicsValidation(
            camera=self._camera,
            board=self.board,
            on_closed=self._on_validation_closed,
        )

    def _on_validation_closed(self):
        rospy.loginfo("Shutting down.")
        rospy.signal_shutdown("Validation closed.")

    def _cinfo_cb(self, msg):
        if self._camera is not None:
            return
        # create camera object
        self._camera = CameraModel(
            width=msg.width,
            height=msg.height,
            K=np.reshape(msg.K, (3, 3)),
            D=msg.D,
            P=np.reshape(msg.P, (3, 4)),
        )
        # create calibrator
        self._calibrator = CameraExtrinsicsCalibration(
            camera=self._camera,
            board=self.board,
            save=self._save_calibration,
            on_saved=self._on_calibration_saved,
            on_cancelled=self._on_calibration_cancelled,
            quiet=False
        )
        # once we got the camera info, we can stop the subscriber
        rospy.loginfo("Camera info message received. Unsubscribing from camera_info topic.")
        # noinspection PyBroadException
        try:
            self._cinfo_sub.shutdown()
        except BaseException:
            pass

    def _img_cb(self, msg):
        # make sure we have received camera info
        if self._camera is None:
            return
        # make sure we are still running
        if rospy.is_shutdown():
            return
        # decode image
        img: BGRImage = self._jpeg.decode(msg.data, pixel_format=TJPF_BGR)
        # pass image to validator (if any), fallback to calibrator
        if self._validator:
            print("Passing image to validator")
            self._validator.on_new_frame(img)
        elif self._calibrator:
            # we are still calibrating, pass image to calibrator instead
            self._calibrator.on_new_frame(img)
        else:
            pass


if __name__ == "__main__":
    node = CameraExtrinsicsCalibrationNode()
    # spin forever
    rospy.spin()