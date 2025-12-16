from controller import Robot
import math

TIME_STEP = 64              
R_WHEEL = 0.021             
L = 0.1054                  

robot = Robot()

lm = robot.getDevice('left wheel motor')
rm = robot.getDevice('right wheel motor')
le = robot.getDevice('left wheel sensor')
re = robot.getDevice('right wheel sensor')

le.enable(TIME_STEP)
re.enable(TIME_STEP)

lm.setPosition(float('inf'))
rm.setPosition(float('inf'))

lm.setVelocity(0.0)
rm.setVelocity(0.0)

STRAIGHT_SPEED = 5.0   
TURN_SPEED = 3.0

SIDE_LEN = 0.5         
RECT_W = 0.6           
RECT_H = 0.4           


def _step():
    if robot.step(TIME_STEP) == -1:
        raise SystemExit


def pause(seconds: float):
    lm.setVelocity(0.0)
    rm.setVelocity(0.0)
    steps = int(seconds * 1000 / TIME_STEP)
    for _ in range(steps):
        _step()


def go_straight(distance_m, wheel_speed=STRAIGHT_SPEED):
    direction = 1 if distance_m >= 0 else -1
    distance_m = abs(distance_m)

    target_angle = distance_m / R_WHEEL

    _step()
    start_l = le.getValue()
    start_r = re.getValue()

    lm.setVelocity(direction * wheel_speed)
    rm.setVelocity(direction * wheel_speed)

    while robot.step(TIME_STEP) != -1:
        ang_l = le.getValue() - start_l
        ang_r = re.getValue() - start_r
        avg = (ang_l + ang_r) / 2.0

        if abs(avg) >= target_angle:
            break

    lm.setVelocity(0.0)
    rm.setVelocity(0.0)
    _step()
    _step()


def rotate_in_place(angle_rad, wheel_speed=TURN_SPEED):
    direction = 1 if angle_rad >= 0 else -1
    target = abs(angle_rad)

    _step()
    start_l = le.getValue()
    start_r = re.getValue()

    lm.setVelocity(-direction * wheel_speed)
    rm.setVelocity(direction * wheel_speed)

    while robot.step(TIME_STEP) != -1:
        d_l = le.getValue() - start_l
        d_r = re.getValue() - start_r
        dtheta = (R_WHEEL / L) * (d_r - d_l)

        if abs(dtheta) >= target:
            break

    lm.setVelocity(0.0)
    rm.setVelocity(0.0)
    _step()
    _step()


def draw_regular_polygon(n_sides, side_length_m, direction=1):
    turn_angle = direction * (2.0 * math.pi / n_sides)
    for _ in range(n_sides):
        go_straight(side_length_m, STRAIGHT_SPEED)
        rotate_in_place(turn_angle, TURN_SPEED)


def draw_rectangle(width_m, height_m, direction=1):
    turn = direction * (math.pi / 2)
    for _ in range(2):
        go_straight(width_m, STRAIGHT_SPEED)
        rotate_in_place(turn, TURN_SPEED)
        go_straight(height_m, STRAIGHT_SPEED)
        rotate_in_place(turn, TURN_SPEED)


def main():
    draw_regular_polygon(n_sides=3, side_length_m=SIDE_LEN, direction=1)
    pause(0.5)

    draw_rectangle(width_m=RECT_W, height_m=RECT_H, direction=1)
    pause(0.5)

    draw_regular_polygon(n_sides=5, side_length_m=SIDE_LEN, direction=1)
    pause(0.5)

    for n in range(3, 10):
        draw_regular_polygon(n_sides=n, side_length_m=SIDE_LEN, direction=1)
        pause(0.5)

    while robot.step(TIME_STEP) != -1:
        pass


if __name__ == "__main__":
    main()
