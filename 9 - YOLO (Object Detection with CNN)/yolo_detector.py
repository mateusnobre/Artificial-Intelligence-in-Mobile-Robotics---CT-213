from keras.models import load_model
import cv2
import numpy as np
from utils import sigmoid


class YoloDetector:
    """
    Represents an object detector for robot soccer based on the YOLO algorithm.
    """
    def __init__(self, model_name, anchor_box_ball=(5, 5), anchor_box_post=(2, 5)):
        """
        Constructs an object detector for robot soccer based on the YOLO algorithm.

        :param model_name: name of the neural network model which will be loaded.
        :type model_name: str.
        :param anchor_box_ball: dimensions of the anchor box used for the ball.
        :type anchor_box_ball: bidimensional tuple.
        :param anchor_box_post: dimensions of the anchor box used for the goal post.
        :type anchor_box_post: bidimensional tuple.
        """
        self.network = load_model(model_name + '.hdf5')
        self.network.summary()  # prints the neural network summary
        self.anchor_box_ball = anchor_box_ball
        self.anchor_box_post = anchor_box_post

    def detect(self, image):
        """
        Detects robot soccer's objects given the robot's camera image.

        :param image: image from the robot camera in 640x480 resolution and RGB color space.
        :type image: OpenCV's image.
        :return: (ball_detection, post1_detection, post2_detection), where each detection is given
                by a 5-dimensional tuple: (probability, x, y, width, height).
        :rtype: 3-dimensional tuple of 5-dimensional tuples.
        """
        # Todo: implement object detection logic
        image = self.preprocess_image(image)
        output = self.network.predict(image)
        ball_detection, post1_detection, post2_detection = self.process_yolo_output(output) 
        return ball_detection, post1_detection, post2_detection

    def preprocess_image(self, image):
        """
        Preprocesses the camera image to adapt it to the neural network.

        :param image: image from the robot camera in 640x480 resolution and RGB color space.
        :type image: OpenCV's image.
        :return: image suitable for use in the neural network.
        :rtype: NumPy 4-dimensional array with dimensions (1, 120, 160, 3).
        """
        # Todo: implement image preprocessing logic
        image = cv2.resize(image, (160, 120), interpolation = cv2.INTER_AREA)
        image = np.array(image)
        image = image / 255.0
        image = np.reshape(image, (1, 120, 160, 3))
        return image

    def process_yolo_output(self, output):
        """
        Processes the neural network's output to yield the detections.

        :param output: neural network's output.
        :type output: NumPy 4-dimensional array with dimensions (1, 15, 20, 10).
        :return: (ball_detection, post1_detection, post2_detection), where each detection is given
                by a 5-dimensional tuple: (probability, x, y, width, height).
        :rtype: 3-dimensional tuple of 5-dimensional tuples.
        """
        coord_scale = 4 * 8  # coordinate scale used for computing the x and y coordinates of the BB's center
        bb_scale = 640  # bounding box scale used for computing width and height
        output = np.reshape(output, (15, 20, 10))  # reshaping to remove the first dimension
        ball_detection = [0,0,0,0,0]
        post1_detection = [0,0,0,0,0]
        post2_detection = [0,0,0,0,0]
        # Todo: implement YOLO logic
        
        # get indices of cell with max probability
        indices_b = np.unravel_index(np.argmax(output[:,:,0], axis = None), output[:,:,0].shape)
        ib, jb = indices_b

        # use indices to make transformations
        Txb = output[ib][jb][1]
        Tyb = output[ib][jb][2]
        Twb = output[ib][jb][3]
        Thb = output[ib][jb][4]
        ball_detection[0] = sigmoid(output[ib][jb][0])
        ball_detection[1] = (jb + sigmoid(Txb))* coord_scale
        ball_detection[2] = (ib + sigmoid(Tyb))* coord_scale
        ball_detection[3] = bb_scale * 5 * np.exp(Twb)
        ball_detection[4] = bb_scale * 5 * np.exp(Thb)

        # get indices of cell with max probability
        indices_p1 = np.unravel_index(np.argmax(output[:,:,5], axis = None), output[:,:,5].shape)
        ip1, jp1 = indices_p1

        # use indices to make transformations
        Txp1 = output[ip1][jp1][6]
        Typ1 = output[ip1][jp1][7]
        Twp1 = output[ip1][jp1][8]
        Thp1 = output[ip1][jp1][9]
        post1_detection[0] = sigmoid(output[ip1][jp1][5])
        post1_detection[1] = (jp1 + sigmoid(Txp1))* coord_scale
        post1_detection[2] = (ip1 + sigmoid(Typ1))* coord_scale
        post1_detection[3] = bb_scale * 2 * np.exp(Twp1)
        post1_detection[4] = bb_scale * 5 * np.exp(Thp1)
        
        output[ip1][jp1][5] = -100 # set current max cell to negative to be able to find the 2nd cell with largest probability
        # get indices of cell with max probability
        indices_p2 = np.unravel_index(np.argmax(output[:,:,5], axis = None), output[:,:,5].shape)
        ip2, jp2 = indices_p2

        # use indices to make transformations
        Txp2 = output[ip2][jp2][6]
        Typ2 = output[ip2][jp2][7]
        Twp2 = output[ip2][jp2][8]
        Thp2 = output[ip2][jp2][9]
        post2_detection[0] = sigmoid(output[ip2][jp2][5])
        post2_detection[1] = (jp2 + sigmoid(Txp2))* coord_scale
        post2_detection[2] = (ip2 + sigmoid(Typ2))* coord_scale
        post2_detection[3] = bb_scale * 2 * np.exp(Twp2)
        post2_detection[4] = bb_scale * 5 * np.exp(Thp2)
        return ball_detection, post1_detection, post2_detection
