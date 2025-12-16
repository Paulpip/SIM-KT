from controller import Robot
import math

TIME_STEP = 64         
R_WHEEL = 0.021         
L = 0.1054              

BASE_LINEAR_SPEED = 0.15   
BASE_ANG_SPEED   = 0.7     

LINEAR_CORR = 1       
ANGLE_CORR  = 1       
CIRCLE_CORR = 1.015   

CIRCLE_SPEED_FACTOR = 0.4

robot = Robot()

left_motor  = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_sensor  = robot.getDevice('left wheel sensor')
right_sensor = robot.getDevice('right wheel sensor')

left_sensor.enable(TIME_STEP)
right_sensor.enable(TIME_STEP)

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

MAX_WHEEL_SPEED = left_motor.getMaxVelocity()

def _step():
    if robot.step(TIME_STEP) == -1:
        raise SystemExit


def _get_wheel_angles():
    _step()
    start_l = left_sensor.getValue()
    start_r = right_sensor.getValue()
    while True:
        _step()
        d_l = left_sensor.getValue() - start_l
        d_r = right_sensor.getValue() - start_r
        yield d_l, d_r


def brake(settle_time=0.05):
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    steps = int(settle_time * 1000 / TIME_STEP)
    for _ in range(steps):
        _step()

def go_straight(distance_m, linear_speed_m_s=BASE_LINEAR_SPEED):

    direction = 1 if distance_m >= 0 else -1
    distance_m = abs(distance_m)

    wheel_speed = (linear_speed_m_s / R_WHEEL) * direction

    gen = _get_wheel_angles()
    next(gen)

    left_motor.setVelocity(wheel_speed)
    right_motor.setVelocity(wheel_speed)

    target_s = distance_m * LINEAR_CORR

    for d_l, d_r in gen:
        s = R_WHEEL * (d_l + d_r) / 2.0
        if abs(s) >= target_s:
            break

    brake()


def rotate_in_place(angle_rad, angular_speed_rad_s=BASE_ANG_SPEED):

    direction = 1 if angle_rad >= 0 else -1
    angle_rad = abs(angle_rad)

    wheel_speed = angular_speed_rad_s * L / (2.0 * R_WHEEL)

    gen = _get_wheel_angles()
    next(gen)

    w_l = -wheel_speed * direction
    w_r =  wheel_speed * direction
    left_motor.setVelocity(w_l)
    right_motor.setVelocity(w_r)

    target_angle = angle_rad * ANGLE_CORR

    for d_l, d_r in gen:
        phi = (R_WHEEL / L) * (d_r - d_l)
        if abs(phi) >= target_angle:
            break

    brake()


def move_circle(radius_m=1.0, turns=1.0):

    if radius_m <= L / 2.0:
        raise ValueError("radius_m должен быть больше L/2")

    k = (radius_m + L / 2.0) / (radius_m - L / 2.0)

    omega_outer = MAX_WHEEL_SPEED * CIRCLE_SPEED_FACTOR
    omega_inner = omega_outer / k

    w_l = omega_inner
    w_r = omega_outer

    gen = _get_wheel_angles()
    next(gen)

    left_motor.setVelocity(w_l)
    right_motor.setVelocity(w_r)

    target_angle = 2.0 * math.pi * turns * CIRCLE_CORR

    while True:
        d_l, d_r = next(gen)
        phi = (R_WHEEL / L) * (d_r - d_l)
        if abs(phi) >= target_angle:
            break

    brake()

def main():
    go_straight(1.0)
    rotate_in_place(math.pi / 2)
    move_circle(radius_m=1.0, turns=1.0)

    while robot.step(TIME_STEP) != -1:
        pass


if __name__ == "__main__":
    main()
