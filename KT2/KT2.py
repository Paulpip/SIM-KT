from controller import Robot
import math

SPEED = 4.0
TURN_SPEED = 2.0

STOP_THRESHOLD = 900.0           
SIDE_SENSOR_SEE_BOX = 0.4        
OFFSET = 0.10                    
TOTAL_BOXES = 4
FINAL_DISTANCE = 1.0

WHEEL_RADIUS = 0.021
WHEEL_CIRC = 2 * math.pi * WHEEL_RADIUS
TURN_EPS = 0.04                 

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor  = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

ds_left  = robot.getDevice("left wheel sensor")
ds_right = robot.getDevice("right wheel sensor")
ds_left.enable(timestep)
ds_right.enable(timestep)

gyro = robot.getDevice("gyro")
gyro.enable(timestep)

front_ir = robot.getDevice("front infrared sensor")
front_ir.enable(timestep)

us_right = robot.getDevice("right ultrasonic sensor")
us_left  = robot.getDevice("left ultrasonic sensor")
us_right.enable(timestep)
us_left.enable(timestep)

robot.step(timestep)

def set_drive(vl, vr):
    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)

def stop():
    set_drive(0.0, 0.0)

def reset_local_odometry(ctx):
    ctx["start_l"] = ds_left.getValue()
    ctx["start_r"] = ds_right.getValue()

def local_distance(ctx):
    dl = (abs(ds_left.getValue() - ctx["start_l"]) / (2 * math.pi)) * WHEEL_CIRC
    dr = (abs(ds_right.getValue() - ctx["start_r"]) / (2 * math.pi)) * WHEEL_CIRC
    return (dl + dr) / 2.0

def update_gyro(ctx):
    now = robot.getTime()
    dt = now - ctx["old_time"]
    ctx["old_time"] = now
    ctx["angle"] += gyro.getValues()[2] * dt

def update_global_odometer(ctx):
    curr_l = ds_left.getValue()
    curr_r = ds_right.getValue()

    diff_l = (curr_l - ctx["prev_l"]) * WHEEL_RADIUS
    diff_r = (curr_r - ctx["prev_r"]) * WHEEL_RADIUS

    ctx["total_dist"] += (diff_l + diff_r) / 2.0
    ctx["prev_l"] = curr_l
    ctx["prev_r"] = curr_r

def pick_side_ultrasonic(side):
    return us_right.getValue() if side == 1 else us_left.getValue()

def schedule_turn(ctx, side_sign, deg):
    return ctx["angle"] + math.radians(deg) * side_sign

def turn_to_target(ctx):
    diff = ctx["target_angle"] - ctx["angle"]
    if abs(diff) > TURN_EPS:
        if diff > 0:
            set_drive(-TURN_SPEED, TURN_SPEED)
        else:
            set_drive(TURN_SPEED, -TURN_SPEED)
        return False
    stop()
    return True

def state_1(ctx, dist_now):
    if front_ir.getValue() < STOP_THRESHOLD:
        set_drive(SPEED, SPEED)
        return 1

    print(f"--- Ящик {ctx['boxes_done'] + 1} обнаружен ---")
    ctx["target_angle"] = schedule_turn(ctx, ctx["side"], 90)
    reset_local_odometry(ctx)
    return 2

def state_2(ctx, dist_now):
    if turn_to_target(ctx):
        reset_local_odometry(ctx)
        return 3
    return 2

def state_3(ctx, dist_now):
    set_drive(SPEED, SPEED)
    sensor_val = pick_side_ultrasonic(ctx["side"])

    if sensor_val < SIDE_SENSOR_SEE_BOX:
        ctx["box_depth"] = dist_now

    if dist_now > (ctx["box_depth"] + OFFSET):
        print(f"Длина ящика: {ctx['box_depth']:.2f} м")
        ctx["target_angle"] = schedule_turn(ctx, -ctx["side"], 90)
        return 4

    return 3

def state_4(ctx, dist_now):
    if turn_to_target(ctx):
        reset_local_odometry(ctx)
        ctx["temp_width"] = 0.0
        return 5
    return 4

def state_5(ctx, dist_now):
    set_drive(SPEED, SPEED)
    sensor_val = pick_side_ultrasonic(ctx["side"])

    if sensor_val < SIDE_SENSOR_SEE_BOX:
        ctx["temp_width"] = dist_now

    if dist_now > (ctx["temp_width"] + OFFSET):
        print("Ширина пройдена.")
        ctx["target_angle"] = schedule_turn(ctx, -ctx["side"], 90)
        return 6

    return 5

def state_6(ctx, dist_now):
    if turn_to_target(ctx):
        reset_local_odometry(ctx)
        return 7
    return 6

def state_7(ctx, dist_now):
    total_return_dist = ctx["box_depth"] + OFFSET
    if dist_now < total_return_dist:
        set_drive(SPEED, SPEED)
        return 7

    ctx["target_angle"] = schedule_turn(ctx, ctx["side"], 90)
    return 8

def state_8(ctx, dist_now):
    if not turn_to_target(ctx):
        return 8

    ctx["boxes_done"] += 1
    print(f"Ящик {ctx['boxes_done']} пройден. Дистанция сейчас: {ctx['total_dist']:.2f}")

    ctx["side"] *= -1
    reset_local_odometry(ctx)

    return 9 if ctx["boxes_done"] >= TOTAL_BOXES else 1

def state_9(ctx, dist_now):
    if dist_now < FINAL_DISTANCE:
        set_drive(SPEED, SPEED)
        return 9

    stop()
    print("=" * 30)
    print("ЗАДАНИЕ ВЫПОЛНЕНО.")
    print(f"Общий пройденный путь: {ctx['total_dist']:.3f} метров")
    print("=" * 30)
    return -1

handlers = {
    1: state_1,
    2: state_2,
    3: state_3,
    4: state_4,
    5: state_5,
    6: state_6,
    7: state_7,
    8: state_8,
    9: state_9,
}

ctx = {
    "state": 1,
    "start_l": ds_left.getValue(),
    "start_r": ds_right.getValue(),
    "angle": 0.0,
    "target_angle": 0.0,
    "old_time": robot.getTime(),

    "box_depth": 0.0,
    "temp_width": 0.0,

    "boxes_done": 0,
    "side": 1,

    "total_dist": 0.0,
    "prev_l": ds_left.getValue(),
    "prev_r": ds_right.getValue(),
}

print("Старт!")

while robot.step(timestep) != -1:
    update_gyro(ctx)
    update_global_odometer(ctx)
    dist_now = local_distance(ctx)

    st = ctx["state"]
    next_state = handlers[st](ctx, dist_now)
    if next_state == -1:
        break
    ctx["state"] = next_state

stop()
robot.step(timestep)
print("Контроллер завершён.")
