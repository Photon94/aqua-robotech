import numpy as np

import cv2 as cv
import pymurapi as mur
from collections import namedtuple
from transitions import Machine
import typing
import numpy
import time

SIMULATOR = False

if not SIMULATOR:
    import picamera

# пороговое значение для определения контуров
CONTOURS_SENSITIVE = 30

# эпсилон при сравнении контуров
CONTOURS_EPSILON_PERCENT = 1

# угол обзора камеры
CAMERA_VIEWING_ANGLE = 50


class PID:
    """PID Controller
    """

    def __init__(self, *, p=0.2, i=0.0, d=0.0, current_time=None, s=100):

        self.Kp = p
        self.Ki = i
        self.Kd = d

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time
        self.saturation = s
        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        """
        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

            if output > self.saturation:
                self.output = self.saturation
            elif output < -self.saturation:
                self.output = -self.saturation
            else:
                self.output = output

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time


def color(r, g, b):
    def inner(func):
        def wrapped(self, *args, **kwargs):
            self.auv.set_rgb_color(r, g, b)
            func(self, *args, **kwargs)
        return wrapped
    return inner


class AUV:
    state = ''
    states = ['undefined', 'start_position', 'turn_forward', 'rotate', 'stop']

    def __init__(self):
        self.machine = Machine(model=self, states=AUV.states, initial='undefined')
        # после перехода очищаем словарь с маркерами
        self.machine.add_transition(
            trigger='find', source='undefined', dest='start_position'
        )

        self.auv = mur.mur_init()

        self.speed = 0
        self.depth = 0
        self.yaw = 0
        self.roll = 0

        if SIMULATOR:
            self.yaw_controller = PID(p=0.3, s=20)
            self.roll_controller = PID(p=0.3, s=20)
            self.speed_controller = PID(p=1, s=20)
            self.depth_controller = PID(p=1, s=20)
        else:
            self.yaw_controller = PID(p=0.3, s=40)
            self.roll_controller = PID(p=0.5, s=50)
            self.speed_controller = PID(p=0.5, s=50)
            self.depth_controller = PID(p=0.5, s=50)

        # курс при старте робота
        self.origin = self.get_yaw()

    @staticmethod
    def clamp_to_360(angle):
        if angle < 0.0:
            return angle + 360.0
        if angle > 360.0:
            return angle - 360.0
        return angle

    @staticmethod
    def to_180(angle):
        if angle > 180.0:
            return angle - 360.0
        return angle

    def calculate(self):
        self.update_image()
        # тут вычисляется функция состояния
        getattr(self, self.state)()

        self.speed_controller.update(-self.speed)
        self.yaw_controller.update(self.yaw)
        self.roll_controller.update(self.roll)
        self.depth_controller.update(self.depth)

        self.yaw_left = self.yaw_controller.output + self.speed_controller.output
        self.yaw_right = -self.yaw_controller.output + self.speed_controller.output

        self.roll_left = self.roll_controller.output + self.depth_controller.output
        self.roll_right = -self.roll_controller.output + self.depth_controller.output

        if self.yaw_left > 100:
            self.yaw_left = 100
        elif self.yaw_left < -100:
            self.yaw_left = -100

        if self.yaw_right > 100:
            self.yaw_right = 100
        elif self.yaw_right < -100:
            self.yaw_right = -100

        if self.roll_left > 100:
            self.roll_left = 100
        elif self.roll_left < -100:
            self.roll_left = -100

        if self.roll_right > 100:
            self.roll_right = 100
        elif self.roll_right < -100:
            self.roll_right = -100

        if SIMULATOR:
            self.auv.set_motor_power(0, self.yaw_left)
            self.auv.set_motor_power(1, self.yaw_right)
        else:
            self.auv.set_motor_power(3, self.yaw_left)
            self.auv.set_motor_power(0, self.yaw_right)
            # уточнить номера моторов!
            self.auv.set_motor_power(1, self.roll_right)
            self.auv.set_motor_power(2, self.roll_left)

    def get_yaw(self):
        return self.auv.get_yaw()

    def update_image(self):
        if SIMULATOR:
            image = self.auv.get_image_front()
        else:
            with picamera.PiCamera() as camera:
                camera.resolution = (320, 240)
                camera.framerate = 24
                image = np.empty((240, 320, 3), dtype=np.uint8)
                camera.capture(image, 'rgb')

        self.rgb_image = image
        self.hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        return image

    @color(50, 50, 0)
    def undefined(self):
        self.speed = 20
        self.depth = 20
        pass

    def start_position(self):
        pass

    def turn_forward(self):
        pass

    def rotate(self):
        pass

    def stop(self):
        pass


auv = AUV()
font = cv.FONT_HERSHEY_COMPLEX_SMALL

while True:

    # if settings.SIMULATOR:
    #     cv.imshow('', auv.rgb_image)
    #     cv.waitKey(1)
    #
    #     t_color = (255, 128, 255)
    #     c_color = (0, 255, 0)
    #     x_center = int(auv.resolution[1] / 2)
    #     # vertical line in center of screen
    #     cv.line(auv.rgb_image, (x_center, 0), (x_center, auv.resolution[0]), (128, 128, 128), 1)
    #     cv.putText(auv.rgb_image, auv.state, (5, 20), font, 0.5, t_color, 1, cv.LINE_AA)
    #     # cv.putText(image, 'e:' + '{:.2f}'.format(e_x), (5, 40), font, 0.5, t_color, 1, cv.LINE_AA)
    #     cv.putText(auv.rgb_image, 'u:' + '{:.2f}/{:.2f}'.format(auv.left, auv.right), (5, 60), font, 0.5, t_color, 1, cv.LINE_AA)
    #     # cv.putText(image, 's:' + str(area), (5, 80), font, 0.5, t_color, 1, cv.LINE_AA)
    #     cv.drawContours(auv.rgb_image, auv.contours, -1, c_color, 2)

    auv.calculate()




