from controller import Robot
import math

TIME_STEP = 8          
R_WHEEL   = 0.021       
L         = 0.1054      

V_LINEAR  = 0.5
R_CIRCLE  = 0.3

robot = Robot()

lm = robot.getDevice('left wheel motor')
rm = robot.getDevice('right wheel motor')
le = robot.getDevice('left wheel sensor')
re = robot.getDevice('right wheel sensor')

le.enable(TIME_STEP)
re.enable(TIME_STEP)

lm.setPosition(float('inf'))
rm.setPosition(float('inf'))

robot.step(TIME_STEP)

def move_circle(R_c, angle_rad, direction, v=V_LINEAR):

    omega_robot = direction * v / R_c

    v_r = v + (L / 2.0) * omega_robot
    v_l = v - (L / 2.0) * omega_robot

    w_r = v_r / R_WHEEL
    w_l = v_l / R_WHEEL

    S_target = abs(angle_rad) * R_c

    start_l = le.getValue()
    start_r = re.getValue()

    lm.setVelocity(w_l)
    rm.setVelocity(w_r)

    while robot.step(TIME_STEP) != -1:
        dl = le.getValue() - start_l
        dr = re.getValue() - start_r
        s_center = R_WHEEL * (dl + dr) / 2.0
        if abs(s_center) >= S_target:
            break

    lm.setVelocity(0)
    rm.setVelocity(0)
    robot.step(TIME_STEP)

half_turn = math.pi
move_circle(R_CIRCLE, half_turn, direction=+1)
move_circle(R_CIRCLE, half_turn, direction=-1)
move_circle(R_CIRCLE, half_turn, direction=-1)
move_circle(R_CIRCLE, half_turn, direction=+1)
# directions = [1, -1, 1, -1]
# for d in directions:
   # move_circle(R_CIRCLE, half_turn, d)


print("Хозянин, я могу быть свободен?")
