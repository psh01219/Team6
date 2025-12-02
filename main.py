#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (
    Motor, TouchSensor, ColorSensor,
    InfraredSensor, UltrasonicSensor, GyroSensor
)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from collections import deque   # ★ BFS용

# ---------------- 기본 설정 ----------------
ev3 = EV3Brick()

# 모터
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
arm = Motor(Port.C)

# 주행 베이스
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# 라인 센서 (바닥 라인 추종용)
left = ColorSensor(Port.S1)   # 왼쪽 라인센서
right = ColorSensor(Port.S4)  # 오른쪽 라인센서

# 블록 색 인식용 센서 (집게 근처에 달려 있다고 가정)
block_sensor = ColorSensor(Port.S3)

# 초음파 센서 (앞쪽에 달려 있다고 가정)
ultra_sensor = UltrasonicSensor(Port.S2)

threshold = 30      # 라인 검정/바탕 경계값 (reflection 기준, 트랙에서 튜닝 필수)
kp = 0.7            # P 제어 계수
OBSTACLE_DIST = 35  # 이 거리(mm) 이하이면 블록 있다고 판단 (튜닝 필수)

# 방향 코드
N, E, S, W = 1, 2, 3, 4

# ---------------- 셀 번호 → 격자 좌표 매핑 ----------------
# (x, y)
CELL_POS = {
    1: (0, 2),
    2: (0, 3),
    3: (0, 4),
    5: (1, 2),
    6: (1, 3),
    7: (1, 4),
    9: (3, 2),
    10: (3, 3),
    11: (3, 4),
}

# 빨간 존 / 파란 존 위치
RED_GOAL = (1, 0)
BLUE_GOAL = (2, 0)

# BFS용 격자 크기 (x: 0~3, y: 0~4 라고 가정)
W = 4  # x 최대 3
H = 5  # y 최대 4
# 장애물 없다고 가정 (모두 0: 통로)
G = [[0] * W for _ in range(H)]

# ---------------- 현재 상태 기억용 전역 변수 ----------------
current_cell = None   # 지금 이동하고 있는(향하고 있는) 셀 번호
grabbed_cell = None   # 블록을 실제로 잡은 셀 번호
grabbed_pos = None    # 블록을 잡은 셀 좌표 (x, y)
grabbed_color = None  # "red" 또는 "blue"

is_searching = True   # ★ 검색 중인지(초음파 ON) / 운반 중인지(초음파 OFF)

# ---------------- 집게(팔) 동작 함수 ----------------
def grab_object():
    """블록을 잡는 동작."""
    arm.run_until_stalled(200, then=Stop.HOLD, duty_limit=50)

def release_object():
    """블록을 놓는 동작."""
    arm.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)

# ---------------- 블록 색 판별 ----------------
def measure_block_rgb():
    """
    block_sensor로 블록의 RGB값을 여러 번 읽어서 평균을 낸 뒤 반환.
    네 실험에 따르면:
      - 빨강 블록: (2,0,0) 근처
      - 파랑 블록: (0,0,2) 근처
    """
    samples = 5
    r_sum = g_sum = b_sum = 0
    for _ in range(samples):
        r, g, b = block_sensor.rgb()
        r_sum += r
        g_sum += g
        b_sum += b
        wait(10)

    r_avg = r_sum / samples
    g_avg = g_sum / samples
    b_avg = b_sum / samples

    print("block rgb avg:", r_avg, g_avg, b_avg)
    return (r_avg, g_avg, b_avg)

def classify_block_color():
    """
    측정된 RGB 값으로 빨강/파랑 판별.
    - 빨강일 때  r≈2, b≈0
    - 파랑일 때 r≈0, b≈2
    => r > b 이면 빨강, b > r 이면 파랑으로 판별.
    """
    r, g, b = measure_block_rgb()

    if r > b:
        ev3.speaker.beep()
        print("RED")
        return "red"
    elif b > r:
        print("BLUE")
        ev3.speaker.beep()
        ev3.speaker.beep()
        return "blue"
    else:
        return "red"

# ---------------- 라인 팔로잉 기본 함수 ----------------
def left_line_following(speed, k):
    left_reflection = left.reflection()
    error = left_reflection - threshold
    turn_rate = k * error
    robot.drive(speed, turn_rate)

def right_line_following(speed, k):
    right_reflection = right.reflection()
    error = right_reflection - threshold
    turn_rate = k * error
    robot.drive(speed, turn_rate)

def obstacle_detected():
    """앞에 일정 거리 이내에 물체가 있으면 True (검색 모드일 때만)."""
    if not is_searching:
        return False
    dist = ultra_sensor.distance()
    if dist is not None and dist < OBSTACLE_DIST:
        return True
    return False

# ---------------- 방향 회전 / 맨해튼 이동 ----------------
def turn_min(now_dir, target_dir):
    """
    현재 방향(now_dir)에서 목표 방향(target_dir)으로
    최소 회전 각도로 회전시키고, 최종 방향을 리턴.
    """
    diff = (target_dir - now_dir) % 4
    angle = [0, 90, 180, -90][diff]
    robot.turn(angle)
    return target_dir

def move_manhattan(start_xy, goal_xy, now_dir):
    """
    start_xy에서 goal_xy까지 (x,y)축으로만 움직이는 맨해튼 이동.
    각 1칸 이동마다 follow_line_one_cell()을 한 번 호출.
    now_dir: 현재 바라보는 방향 (N,E,S,W 중 하나의 숫자)
    """
    x, y = start_xy
    gx, gy = goal_xy

    dx = gx - x
    dy = gy - y

    # x 축 방향 먼저 이동 (E/W)
    if dx != 0:
        target_dir = E if dx > 0 else W
        now_dir = turn_min(now_dir, target_dir)
        steps = abs(dx)
        for _ in range(steps):
            follow_line_one_cell()
            x += 1 if target_dir == E else -1

    # y 축 방향 이동 (N/S)
    if dy != 0:
        target_dir = N if dy > 0 else S
        now_dir = turn_min(now_dir, target_dir)
        steps = abs(dy)
        for _ in range(steps):
            follow_line_one_cell()
            y += 1 if target_dir == N else -1

    return (x, y), now_dir

# ---------------- BFS ----------------
def bfs(s, g):
    """
    s: 시작 좌표 (x,y)
    g: 목표 좌표 (x,y)
    G[y][x] == 0 일 때 통과 가능, 1이면 벽
    """
    dist = [[None] * W for _ in range(H)]
    prev = [[None] * W for _ in range(H)]
    q = deque([s])
    dist[s[1]][s[0]] = 0

    while q:
        x, y = q.popleft()
        if (x, y) == g:
            break
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H and not G[ny][nx] and dist[ny][nx] is None:
                dist[ny][nx] = dist[y][x] + 1
                prev[ny][nx] = (x, y)
                q.append((nx, ny))

    path = []
    if dist[g[1]][g[0]] is not None:
        p = g
        while p:
            path.append(p)
            p = prev[p[1]][p[0]]
        path.reverse()
    return path, dist

def go_to_goal_with_bfs():
    """
    grabbed_pos, grabbed_color를 이용해:
      S = grabbed_pos
      G = RED_GOAL 또는 BLUE_GOAL
    BFS로 경로를 구하고, move_manhattan으로 한 칸씩 이동.
    """
    if grabbed_pos is None or grabbed_color is None:
        print("No grabbed block info.")
        return

    if grabbed_color == "red":
        goal = RED_GOAL
    else:
        goal = BLUE_GOAL

    start = grabbed_pos
    print("BFS from", start, "to", goal)
    path, dist = bfs(start, goal)
    print("path:", path)

    if not path or len(path) <= 1:
        print("No path or already at goal.")
        return

    # EV3가 현재 보고 있는 방향 (가정: 동쪽 E에서 시작)
    now_dir = E
    current = path[0]

    for nxt in path[1:]:
        current, now_dir = move_manhattan(current, nxt, now_dir)

# ---------------- 장애물 감지 → 잡기 + 색 판단 + BFS 이동 ----------------
def handle_obstacle_and_grab():
    """
    블록 감지 시 공통 동작:
    1) 정지
    2) 20mm 앞으로 전진
    3) 그랩
    4) 잠깐 대기 후 색 판별
    5) '빨간색' 또는 '파란색' 말하기
    6) 현재 셀 번호와 좌표, 색을 전역 변수에 저장
    7) BFS로 RED/BLUE 존까지 최단 경로 이동
    """
    global grabbed_cell, grabbed_pos, grabbed_color, current_cell, is_searching

    # 더 이상 탐색 모드 아님 (운반 중엔 초음파 무시)
    is_searching = False

    robot.stop()
    robot.straight(20)
    grab_object()

    wait(300)

    color_name = classify_block_color()
    grabbed_color = color_name

    if color_name == "red":
        ev3.speaker.say("빨간색")
    else:
        ev3.speaker.say("파란색")

    grabbed_cell = current_cell
    grabbed_pos = CELL_POS.get(grabbed_cell, None)

    print("=== BLOCK INFO ===")
    print("cell:", grabbed_cell)
    print("pos :", grabbed_pos)
    print("color:", grabbed_color)

    # ★ 여기서 BFS로 RED/BLUE 존까지 이동
    if grabbed_pos is not None:
        go_to_goal_with_bfs()

    ev3.speaker.beep()
    return True

# ---------------- 한 칸 라인트레이싱 (오른쪽 센서 기준) ----------------
def follow_line_one_cell():
    for _ in range(1):
        # 1) 흰 → 검
        while True:
            if obstacle_detected():
                return handle_obstacle_and_grab()

            left_reflection = left.reflection()
            right_reflection = right.reflection()

            if right_reflection < threshold:
                robot.stop()
                break
            else:
                error = left_reflection - threshold
                turn_rate = kp * error
                robot.drive(100, turn_rate)
            wait(10)

        # 2) 검 → 흰
        while True:
            if obstacle_detected():
                return handle_obstacle_and_grab()

            left_reflection = left.reflection()
            right_reflection = right.reflection()

            if right_reflection > threshold:
                robot.stop()
                break
            else:
                error = left_reflection - threshold
                turn_rate = kp * error
                robot.drive(100, turn_rate)
            wait(10)

    return False

# ---------------- 한 칸 라인트레이싱 (왼쪽 센서 기준: 5 → 9 구간용) ----------------
def follow_line_one_cell_left():
    for _ in range(1):
        # 1) 흰 → 검
        while True:
            if obstacle_detected():
                return handle_obstacle_and_grab()

            left_reflection = left.reflection()

            if left_reflection < threshold:
                robot.stop()
                break
            else:
                left_line_following(100, kp)
            wait(10)

        # 2) 검 → 흰
        while True:
            if obstacle_detected():
                return handle_obstacle_and_grab()

            left_reflection = left.reflection()

            if left_reflection > threshold:
                robot.stop()
                break
            else:
                left_line_following(100, kp)
            wait(10)

    return False

# ---------------- ㄹ자 전역 탐색 ----------------
def search_L_shape():
    """
    탐색 순서: 1 → 2 → 3 → 7 → 6 → 5 → 9 → 10 → 11
    각 구간 진입 전에 current_cell을 해당 셀 번호로 세팅
    """
    global current_cell, is_searching
    is_searching = True  # 탐색 시작할 때 초음파 ON

    # 1행: 1 → 2 → 3 (실제 라인 구조 때문에 4번 이동)
    top_cells_for_step = [1, 2, 3, 3]
    for cell_id in top_cells_for_step:
        current_cell = cell_id
        found = follow_line_one_cell()
        if found:
            return

    # 3 → 7 : 아래로 한 칸
    robot.turn(90)
    current_cell = 7
    found = follow_line_one_cell()
    if found:
        return

    # 7 → 6 → 5 : 왼쪽으로 두 칸
    robot.turn(90)
    for cell_id in [6, 5]:
        current_cell = cell_id
        found = follow_line_one_cell()
        if found:
            return

    # 5 → 9 : 아래로 한 칸 (왼쪽 센서 기준)
    robot.turn(-90)
    current_cell = 9
    found = follow_line_one_cell_left()
    if found:
        return

    # 9 → 10 → 11 : 오른쪽으로 두 칸
    robot.turn(90)
    for cell_id in [10, 11]:
        current_cell = cell_id
        found = follow_line_one_cell()
        if found:
            return

    ev3.speaker.say("L search done")

# ---------------- 메인 ----------------
if __name__ == "__main__":
    ev3.speaker.beep()
    release_object()
    search_L_shape()
    # 이후에 상태 확인도 가능:
    # print("최종 잡은 셀:", grabbed_cell, "좌표:", grabbed_pos, "색:", grabbed_color)
