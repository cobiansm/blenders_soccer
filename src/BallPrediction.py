import numpy as np
import cv2

class BallPrediction:
    def __init__(self):
        n_states = 4
        n_measures = 2
        self.kalman = cv2.KalmanFilter(n_states, n_measures)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) * 0.03

        self.kernel = np.ones((5,5),np.uint8)

    def get_predictions(self, frame, center, predict_num=4):
        for p in range(predict_num):
            if p == 0:
                prediction = self._predict(np.array((center.x, center.y), np.float32))
                first_prediction = prediction
            else:
                prediction = self._predict(np.array((old_prediction[0], old_prediction[1]), np.float32))
            cv2.circle(frame, (prediction[0], prediction[1]),5, color=(255,0,0), thickness=3)
            old_prediction = prediction
        
        try:
            slope = (prediction[1] - first_prediction[1]) / (prediction[0] - first_prediction[0])

            # calculate y-intercept
            b = first_prediction[1]- (slope * first_prediction[0])

            # draw infinite line
            cv2.line(frame, (0, int(b)), (1280, int((slope * 1280) + b)), color=(255,255,255), thickness=2)
            cv2.line(frame, (first_prediction[0], first_prediction[1]), (first_prediction[0],prediction[1]), color=(255,255,255), thickness=2)
        except:
            pass

    def _predict(self, value):
        self.kalman.correct(value)
        tp = self.kalman.predict()
        return int(tp[0]), int(tp[1])