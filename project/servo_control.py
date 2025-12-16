#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import lgpio
import time

# ---------- BCM PIN DEFINITIONS (TB6612) ----------

# Motor A (Right)
PWMA = 13
AIN1 = 6
AIN2 = 5

# Motor B (Left)
PWMB = 22
BIN1 = 17
BIN2 = 27

# Standby (must be high to enable TB6612)
STBY = 16

PWM_FREQ = 1000          # DC motor PWM frequency
CHIP = 0               # Raspberry Pi 5 = gpiochip0 or gpiochip4 (use what worked before)
MAX_SPEED = 1.0          # expected incoming values [-1..1]

# ---------- SERVO (CAMERA) SETTINGS ----------

SERVO_PIN = 26           # BCM 26
SERVO_FREQ = 50          # 50 Hz for hobby servo

SERVO_MIN_ANGLE = 0.0
SERVO_MAX_ANGLE = 180.0

# duty % for 0° and 180° (tweak if servo endpoints look off)
SERVO_MIN_DUTY = 2.5
SERVO_MAX_DUTY = 12.0


class MotorControlNode(Node):

    def __init__(self):
        super().__init__('motor_and_servo_control')

        self.get_logger().info("Initializing Motor + Servo Control Node (lgpio + TB6612 + servo)...")

        # ---- GPIO SETUP ----
        self.h = lgpio.gpiochip_open(CHIP)

        # DC motor pins
        for pin in [PWMA, AIN1, AIN2, PWMB, BIN1, BIN2, STBY]:
            lgpio.gpio_claim_output(self.h, pin)

        # Servo pin
        lgpio.gpio_claim_output(self.h, SERVO_PIN)

        # Start with everything OFF
        lgpio.gpio_write(self.h, STBY, 0)
        lgpio.gpio_write(self.h, AIN1, 0)
        lgpio.gpio_write(self.h, AIN2, 0)
        lgpio.gpio_write(self.h, BIN1, 0)
        lgpio.gpio_write(self.h, BIN2, 0)
        lgpio.tx_pwm(self.h, PWMA, PWM_FREQ, 0)
        lgpio.tx_pwm(self.h, PWMB, PWM_FREQ, 0)

        # Enable TB6612
        lgpio.gpio_write(self.h, STBY, 1)

        # Initialize servo at 90° (center)
        self.servo = lgpioServo(CHIP, SERVO_PIN)

        # ---- ROS SUBSCRIBERS ----
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float32, '/camera_servo', self.servo_callback, 10)

        self.get_logger().info("Ready. Listening for /cmd_vel (motors) and /camera_servo (servo).")

    # ---------------------------------------------------------
    # DC MOTOR CONTROL
    # ---------------------------------------------------------

    def set_motor(self, pwm_pin, in1_pin, in2_pin, value):
        """
        value: [-1.0 .. 1.0]
        negative → reverse
        positive → forward
        """

        # Clamp input
        value = max(-1.0, min(1.0, value))
        duty = abs(value) * 100.0

        if value > 0:
            lgpio.gpio_write(self.h, in1_pin, 1)
            lgpio.gpio_write(self.h, in2_pin, 0)
        elif value < 0:
            lgpio.gpio_write(self.h, in1_pin, 0)
            lgpio.gpio_write(self.h, in2_pin, 1)
        else:
            # stop / brake
            lgpio.gpio_write(self.h, in1_pin, 0)
            lgpio.gpio_write(self.h, in2_pin, 0)
            duty = 0

        lgpio.tx_pwm(self.h, pwm_pin, PWM_FREQ, duty)

    def cmd_vel_callback(self, msg: Twist):
        """
        Differential-drive mixing:
            left  = linear.x - angular.z
            right = linear.x + angular.z
        """

        linear = float(msg.linear.x)
        angular = float(msg.angular.z)

        left_cmd = linear - angular
        right_cmd = linear + angular

        # Clamp to [-1..1]
        left_cmd = max(min(left_cmd, 1.0), -1.0)
        right_cmd = max(min(right_cmd, 1.0), -1.0)

        # Motor B = left
        self.set_motor(PWMB, BIN1, BIN2, left_cmd)

        # Motor A = right
        self.set_motor(PWMA, AIN1, AIN2, right_cmd)

        self.get_logger().info(f"L={left_cmd:.2f}, R={right_cmd:.2f}")

    # ---------------------------------------------------------
    # SERVO CONTROL
    # ---------------------------------------------------------

    class LgpioServo:
        def __init__(self, CHIP, SERVO_PIN):
            self.chip = lgpio.gpiochip_open(CHIP)
            self.pin = gpio_pin

            lgpio.gpio_claim_output(self.chip, self.pin)
            self.wave_id = None

        def set_angle(self, angle):
            angle = max(0, min(180, angle))

            pulse_width = int(500 + (angle / 180.0) * 2000)

            pulses = [
                lgpio.pulse(self.pin, 1, pulse_width),
                lgpio.pulse(self.pin, 0, 20000 - pulse_width)
            ]

            try:
                if self.wave_id is not None:
                    lgpio.wave_tx_stop(self.chip)
                    lgpio.wave_tx_delete(self.chip)
            
            except:
                pass

            self.wave_id = lgpio.wave_tx_new(self.chip, pulses)
            lgpio.wave_send_repeat(self.chip, self.wave_id)

        def stop(self):
            lgpio.wave_tx_stop(self.chip)
            lgpio.wave_tx_delete(self.chip)
            lgpio.gpiochip_close(self.chip)

    def servo_callback(self, msg: Float32):
        """
        Expecting msg.data to be an angle in degrees [0..180].
        (If your publisher sends -1..1 from joystick, map that to an angle there,
         or you can modify this function to do it.)
        """
        angle = msg.data
        self.servo.set_angle(angle)

    # ---------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info("Shutting down motors and servo...")

        # Stop motors
        self.set_motor(PWMA, AIN1, AIN2, 0)
        self.set_motor(PWMB, BIN1, BIN2, 0)

        # Optionally move servo back to center or just stop PWM
        self.set_servo_angle(90.0)   # or lgpio.tx_pwm(self.h, SERVO_PIN, SERVO_FREQ, 0)

        lgpio.gpio_write(self.h, STBY, 0)
        lgpio.gpiochip_close(self.h)

        super().destroy_node()


# ---------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
