from typing import Tuple

import numpy as np
import cv2


def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:              The shape of the steer matrix.

    Return:
        steer_matrix_left:  The steering (angular rate) matrix for Braitenberg-like control
                            using the masked left lane markings (numpy.ndarray)
    """

    # TODO: implement your own solution here
    steer_matrix_left = np.zeros(shape)
    steer_matrix_left[:, shape[1]//3:] = -1
    # ---
    return steer_matrix_left


def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:               The shape of the steer matrix.

    Return:
        steer_matrix_right:  The steering (angular rate) matrix for Braitenberg-like control
                             using the masked right lane markings (numpy.ndarray)
    """

    # TODO: implement your own solution here
    steer_matrix_right = np.zeros(shape)
    steer_matrix_right[:, :shape[1]*2//3] = 1
    # ---
    return steer_matrix_right


def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Args:
        image: An image from the robot's camera in the BGR color space (numpy.ndarray)
    Return:
        left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
        right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
    """
    
    sens = 100
    white_lower_hsv = np.array([0, 0, 255-sens])
    white_upper_hsv = np.array([255, sens, 255])
    yellow_lower_hsv = np.array([10, 50, 80])
    yellow_upper_hsv = np.array([30, 255, 255])

    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    imggray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    sigma = np.pi

    img_gaussian = cv2.GaussianBlur(imggray,(0,0), sigma)
    sobelx = cv2.Sobel(img_gaussian, cv2.CV_64F, 1, 0)
    sobely = cv2.Sobel(img_gaussian, cv2.CV_64F, 0, 1)
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)    

    thresh_w = 10
    thresh_y = 20

    mask_mag_white = (Gmag > thresh_w)
    mask_mag_yellow = (Gmag > thresh_y)
    
    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)

    width = image.shape[1]
    mask_left = np.ones(sobelx.shape)
    mask_left[:,int(np.floor(width/2)):width + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(width/2))] = 0
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)

    mask_left_edge = mask_left * mask_mag_yellow * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge = mask_right * mask_mag_white * mask_sobelx_pos * mask_sobely_neg * mask_white

    return mask_left_edge, mask_right_edge
