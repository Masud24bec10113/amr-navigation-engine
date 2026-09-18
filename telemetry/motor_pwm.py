import numpy as np


class MotorControllerTelemetry:
    def __init__(self, max_rad_s=15.0, pwm_period=255):
        self.max_w = max_rad_s
        self.pwm_max = pwm_period

    def rad_s_to_pwm(self, angular_speed):
        duty = int(np.clip(abs(angular_speed) / self.max_w * self.pwm_max, 0, self.pwm_max))
        direction = "FWD" if angular_speed >= 0 else "REV"
        return direction, duty

    def serialize(self, w_left, w_right, packet_id=0):
        dir_l, pwm_l = self.rad_s_to_pwm(w_left)
        dir_r, pwm_r = self.rad_s_to_pwm(w_right)
        return f"$AMR,ID={packet_id:04d},L_DIR={dir_l},L_PWM={pwm_l:03d},R_DIR={dir_r},R_PWM={pwm_r:03d}*FF"
