#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import lgpio
import time

# ---------- BCM PIN DEFINITIONS (DRV8833) ----------

# Motor A (Right)
AIN1 = 5
AIN2 = 6

# Motor B (Left)
BIN1 = 17
BIN2 = 27

# Standby (must be high to enable DRV8833)
STBY = 16

PWM_FREQ = 1000          # DC motor PWM frequency
CHIP = 0                 # Raspberry Pi 5 = gpiochip0 or gpiochip4 (use what worked before)
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

        self.get_logger().info("Initializing Motor + Servo Control Node (lgpio + DRV8833 + servo)...")

        # ---- GPIO SETUP ----
        self.h = lgpio.gpiochip_open(CHIP)

        # DC motor pins (no separate PWM pins on DRV8833)
        for pin in [AIN1, AIN2, BIN1, BIN2, STBY]:
            lgpio.gpio_claim_output(self.h, pin)

        # Servo pin
        lgpio.gpio_claim_output(self.h, SERVO_PIN)
        lgpio.gpio_write(self.h, SERVO_PIN, 0)

        # Start with everything OFF
        lgpio.gpio_write(self.h, STBY, 0)
        lgpio.gpio_write(self.h, AIN1, 0)
        lgpio.gpio_write(self.h, AIN2, 0)
        lgpio.gpio_write(self.h, BIN1, 0)
        lgpio.gpio_write(self.h, BIN2, 0)

        # Ensure PWM duty is 0 on all H-bridge inputs
        lgpio.tx_pwm(self.h, AIN1, PWM_FREQ, 0)
        lgpio.tx_pwm(self.h, AIN2, PWM_FREQ, 0)
        lgpio.tx_pwm(self.h, BIN1, PWM_FREQ, 0)
        lgpio.tx_pwm(self.h, BIN2, PWM_FREQ, 0)

        # Enable DRV8833
        lgpio.gpio_write(self.h, STBY, 1)

        # Initialize servo at 90° (center)
        self.current_servo_angle = 90.0
        self.servo_received_first_command = False
        self.set_servo_angle(self.current_servo_angle)

        # ---- ROS SUBSCRIBERS ----
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float32, '/camera_servo', self.servo_callback, 10)

        self.get_logger().info("Ready. Listening for /cmd_vel (motors) and /camera_servo (servo).")

    # ---------------------------------------------------------
    # DC MOTOR CONTROL  (DRV8833: PWM directly on IN pins)
    # ---------------------------------------------------------

    def set_motor(self, in1_pin, in2_pin, value):
        """
        value: [-1.0 .. 1.0]
        negative → reverse
        positive → forward

        DRV8833: PWM is applied to the 'active' input,
        the other input is held LOW.
        """

        # Clamp input
        value = max(-1.0, min(1.0, value))
        duty = abs(value) * 100.0

        # Stop PWM on both inputs and pull them low first
        lgpio.tx_pwm(self.h, in1_pin, PWM_FREQ, 0)
        lgpio.tx_pwm(self.h, in2_pin, PWM_FREQ, 0)
        lgpio.gpio_write(self.h, in1_pin, 0)
        lgpio.gpio_write(self.h, in2_pin, 0)

        if value > 0:
            # Forward: in2 = PWM, in1 = LOW (or vice versa, as long as you're consistent)
            lgpio.tx_pwm(self.h, in2_pin, PWM_FREQ, duty)
        elif value < 0:
            # Reverse: in1 = PWM, in2 = LOW
            lgpio.tx_pwm(self.h, in1_pin, PWM_FREQ, duty)
        else:
            # Stop (coast): both LOW, 0% duty
            duty = 0
            # already set above

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
        self.set_motor(BIN1, BIN2, left_cmd)

        # Motor A = right
        self.set_motor(AIN1, AIN2, right_cmd)

        self.get_logger().info(f"L={left_cmd:.2f}, R={right_cmd:.2f}")

    # ---------------------------------------------------------
    # SERVO CONTROL  (unchanged)
    # ---------------------------------------------------------

    def angle_to_duty(self, angle_deg: float) -> float:
        """Map servo angle (0–180°) to PWM duty cycle (%)"""
        angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle_deg))
        duty = SERVO_MIN_DUTY + (angle / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)) * (SERVO_MAX_DUTY - SERVO_MIN_DUTY)
        return duty

    def set_servo_angle(self, angle_deg: float):
        duty = self.angle_to_duty(angle_deg)
        lgpio.tx_pwm(self.h, SERVO_PIN, SERVO_FREQ, duty)

    def servo_callback(self, msg: Float32):
        """
        Expecting msg.data to be an angle in degrees [0..180].
        (If your publisher sends -1..1 from joystick, map that to an angle there,
         or you can modify this function to do it.)
        """
        angle = float(msg.data)

        # Optional: clamp for safety
        angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle))

        if not self.servo_received_first_command:
            self.get_logger().info(f"First Servo command received: moving to {angle:.1f}°")
            self.servo_received_first_command = True
        elif abs(angle - self.current_servo_angle) < 0.1:
            return

        self.current_servo_angle = angle
        self.set_servo_angle(angle)

        self.get_logger().info(f"Servo angle: {angle:.1f}°")

    # ---------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info("Shutting down motors and servo...")

        # Stop motors
        self.set_motor(AIN1, AIN2, 0)
        self.set_motor(BIN1, BIN2, 0)

        # If servo was used, move it to center and stop PWM
        if self.servo_received_first_command:
            self.set_servo_angle(90.0)
            time.sleep(0.1)
            lgpio.tx_pwm(self.h, SERVO_PIN, SERVO_FREQ, 0)

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
