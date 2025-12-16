from controller import Robot
import math

TIME_STEP = 8
R_WHEEL   = 0.021
L         = 0.1054

robot = Robot()

lm = robot.getDevice('left wheel motor')
rm = robot.getDevice('right wheel motor')
le = robot.getDevice('left wheel sensor')
re = robot.getDevice('right wheel sensor')

us_f  = robot.getDevice('front ultrasonic sensor')
us_fl = robot.getDevice('front left ultrasonic sensor')
us_fr = robot.getDevice('front right ultrasonic sensor')

for s in (le, re):
    s.enable(TIME_STEP)
for s in (us_f, us_fl, us_fr):
    s.enable(TIME_STEP)

lm.setPosition(float('inf'))
rm.setPosition(float('inf'))
lm.setVelocity(0.0)
rm.setVelocity(0.0)

robot.step(TIME_STEP)

V_LINEAR = 0.55
R_CIRCLE = 1          
TURN = 2 * math.pi       


segments = [
    (+1, TURN), 
    (+1, TURN),  
]

OBS_STOP  = 0.25   
OBS_CLEAR = 0.35   
AVOID_TURN_W = 2.5 
AVOID_FWD_V  = 0.25
AVOID_STEP   = 0.25

def clamp_dist(d, default=5.0):
    return d if d > 0 else default

def stop():
    lm.setVelocity(0.0)
    rm.setVelocity(0.0)

def set_vw(v, w):

    v_r = v + (L/2.0) * w
    v_l = v - (L/2.0) * w
    rm.setVelocity(v_r / R_WHEEL)
    lm.setVelocity(v_l / R_WHEEL)

prev_l = le.getValue()
prev_r = re.getValue()
total_path = 0.0

def update_path():
    global prev_l, prev_r, total_path
    cur_l = le.getValue()
    cur_r = re.getValue()

    dl = (cur_l - prev_l) * R_WHEEL
    dr = (cur_r - prev_r) * R_WHEEL

    prev_l, prev_r = cur_l, cur_r

    ds = (dl + dr) / 2.0
    total_path += abs(ds)

seg_i = 0
seg_target = 0.0
seg_start_l = le.getValue()
seg_start_r = re.getValue()

def seg_progress():
    dl = (le.getValue() - seg_start_l) * R_WHEEL
    dr = (re.getValue() - seg_start_r) * R_WHEEL
    return (dl + dr) / 2.0

def start_segment(i):
    global seg_i, seg_target, seg_start_l, seg_start_r
    seg_i = i
    seg_start_l = le.getValue()
    seg_start_r = re.getValue()
    direction, ang = segments[seg_i]
    seg_target = abs(ang) * R_CIRCLE  

def follow_current_segment():
    direction, ang = segments[seg_i]
    omega = direction * V_LINEAR / R_CIRCLE
    set_vw(V_LINEAR, omega)

avoid_mode = False
avoid_phase = 0
avoid_dir = 1
avoid_start_l = 0.0
avoid_start_r = 0.0

def read_obs():
    df  = clamp_dist(us_f.getValue())
    dfl = clamp_dist(us_fl.getValue())
    dfr = clamp_dist(us_fr.getValue())
    return df, dfl, dfr

def start_avoid(df, dfl, dfr):
    global avoid_mode, avoid_phase, avoid_dir, avoid_start_l, avoid_start_r
    avoid_mode = True
    avoid_phase = 0
    avoid_dir = +1 if dfl < dfr else -1
    avoid_start_l = le.getValue()
    avoid_start_r = re.getValue()

def avoid_step():
    global avoid_mode, avoid_phase, avoid_start_l, avoid_start_r

    df, dfl, dfr = read_obs()

    if avoid_phase == 0:
        lm.setVelocity(-avoid_dir * AVOID_TURN_W)
        rm.setVelocity(+avoid_dir * AVOID_TURN_W)
        if df > OBS_CLEAR and dfl > OBS_CLEAR and dfr > OBS_CLEAR:
            stop()
            robot.step(TIME_STEP)
            avoid_phase = 1
            avoid_start_l = le.getValue()
            avoid_start_r = re.getValue()
        return

    if avoid_phase == 1:
        set_vw(AVOID_FWD_V, 0.0)
        dl = (le.getValue() - avoid_start_l) * R_WHEEL
        dr = (re.getValue() - avoid_start_r) * R_WHEEL
        s = (dl + dr) / 2.0
        if abs(s) >= AVOID_STEP:
            stop()
            robot.step(TIME_STEP)
            avoid_mode = False
            avoid_phase = 0

print("Старт: одна восьмёрка (2 круга) + объезд + вывод пути")
start_segment(0)

while robot.step(TIME_STEP) != -1:
    update_path()

    df, dfl, dfr = read_obs()
    if not avoid_mode and (df < OBS_STOP or dfl < OBS_STOP or dfr < OBS_STOP):
        start_avoid(df, dfl, dfr)

    if avoid_mode:
        avoid_step()
        continue

    follow_current_segment()

    if abs(seg_progress()) >= seg_target:
        stop()
        robot.step(TIME_STEP)
        seg_i += 1
        if seg_i >= len(segments):
            break
        start_segment(seg_i)

stop()
robot.step(TIME_STEP)

print(f"Восьмёрка завершена. Общий пройденный путь ≈ {total_path:.3f} м")
print("Хозяин, я могу быть свободен?")
