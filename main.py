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
from collections import deque  # ★ BFS 용

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
kp = 0.65           # P 제어 계수
OBSTACLE_DIST = 37  # 이 거리(mm) 이하이면 블록 있다고 판단 (튜닝 필수)

# 방향 코드
# N: y +1 (아래쪽으로 한 칸)
# E: x +1
# S: y -1 (위쪽으로 한 칸)
# W: x -1
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
    9: (2, 2),
    10: (2, 3),
    11: (2, 4),
}

# ㄹ자 탐색 순서
L_ORDER = [1, 2, 3, 7, 6, 5, 9, 10, 11]

# 빨간 존 / 파란 존 위치
RED_GOAL = (1, 0)
BLUE_GOAL = (2, 0)

# ---------------- BFS용 격자 크기 (x: 0~2, y: 0~4) ----------------
GRID_W = 3  # x = 0,1,2
GRID_H = 5  # y = 0~4

# 장애물 없다고 가정 (모두 0: 통로) – 셀 자체는 다 통행 가능
G = [[0] * GRID_W for _ in range(GRID_H)]

# ★★★ 맵 상에서 "이동이 금지된 간선(벽)" 정의 ★★★
BLOCKED_EDGES = {
    # 기존 막힌 간선
    ((1, 1), (1, 2)),
    ((1, 2), (1, 1)),
    ((2, 1), (2, 2)),
    ((2, 2), (2, 1)),
    # ★ 추가: (1,0) <-> (0,0) 이동 불가
    ((1, 0), (0, 0)),
    ((0, 0), (1, 0)),
    # ★ 추가: (2,0) <-> (1,0) 이동 불가
    ((2, 0), (1, 0)),
    ((1, 0), (2, 0)),
}

def can_move(x, y, nx, ny):
    """(x,y)에서 (nx,ny)로 이동 가능한지 검사 (벽 고려)."""
    if not (0 <= nx < GRID_W and 0 <= ny < GRID_H):
        return False
    if G[ny][nx]:
        return False
    if ((x, y), (nx, ny)) in BLOCKED_EDGES:
        return False
    return True

# ---------------- 현재 상태 기억용 전역 변수 ----------------
current_cell = None
grabbed_cell = None
grabbed_pos = None
grabbed_color = None

# ---------------- 집게(팔) 동작 함수 ----------------
def grab_object():
    arm.run_until_stalled(200, then=Stop.HOLD, duty_limit=50)

def release_object():
    arm.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)

# ---------------- 블록 색 판별 ----------------
def measure_block_rgb():
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
    r, g, b = measure_block_rgb()

    if r > b:
        print("RED")
        return "red"
    elif b > r:
        print("BLUE")
        return "blue"
    else:
        return "red"

# ---------------- 라인 팔로잉 기본 함수 (탐색용) ----------------
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
    dist = ultra_sensor.distance()
    if dist is not None and dist < OBSTACLE_DIST:
        return True
    return False

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

# ---------------- 방향 계산 헬퍼 ----------------
def direction_between(start_xy, goal_xy):
    """start_xy -> goal_xy 로 갈 때 필요한 방향(N/E/S/W)을 리턴."""
    x, y = start_xy
    gx, gy = goal_xy
    dx = gx - x
    dy = gy - y

    if dx == 1:
        return E
    elif dx == -1:
        return W
    elif dy == 1:
        return N
    elif dy == -1:
        return S
    else:
        return None

def opposite_dir(d):
    """180도 반대 방향 리턴."""
    if d is None:
        return None
    return ((d - 1 + 2) % 4) + 1

# ---------------- BFS + 맨해튼 이동용 함수들 ----------------
def use_right_edge_sensor(pos, target_dir):
    x, y = pos

    # (1,1)은 항상 오른쪽 센서
    if (x, y) == (1, 1):
        return True

    # (0,1)
    if (x, y) == (0, 1):
        if target_dir in (N, E):
            return True
        else:
            return False

    # (0,2) : 셀 1
    if (x, y) == (0, 2):
        if target_dir == N:
            return True
        if target_dir in (E, S):
            return False

    # (0,3) : 셀 2
    if (x, y) == (0, 3):
        if target_dir == N:
            return True
        if target_dir == S:
            return False

    # (0,4) : 셀 3
    if (x, y) == (0, 4):
        if target_dir == E:
            return True
        if target_dir == S:
            return False

    # (1,4) : 셀 7
    if (x, y) == (1, 4):
        if target_dir == W:
            return False
        if target_dir == E:
            return True

    # (1,3) : 셀 6
    if (x, y) == (1, 3):
        return True

    # (1,2) : 셀 5
    if (x, y) == (1, 2):
        if target_dir == W:
            return True
        if target_dir == E:
            return False

    # (2,2) : 셀 9
    if (x, y) == (2, 2):
        if target_dir == W:
            return True
        if target_dir == N:
            return False

    # (2,3) : 셀 10
    if (x, y) == (2, 3):
        if target_dir == S:
            return True
        if target_dir == N:
            return False

    # (2,4) : 셀 11
    if (x, y) == (2, 4):
        if target_dir == W:
            return False
        if target_dir == S:
            return True

    # RED_GOAL (1,0) : N으로 나갈 때 왼쪽 센서
    if (x, y) == (1, 0) and target_dir == N:
        return False

    # BLUE_GOAL (2,0) : N으로 나갈 때 왼쪽 센서
    if (x, y) == (2, 0) and target_dir == N:
        return False

    # 기본 규칙
    if target_dir in (N, E):
        return True
    else:
        return False

def follow_line_one_cell_move_dir(pos, target_dir):
    use_right = use_right_edge_sensor(pos, target_dir)

    for _ in range(1):
        # 1) 흰 → 검
        while True:
            L = left.reflection()
            R = right.reflection()

            edge_val = R if use_right else L
            ctrl_val = L if use_right else R

            if edge_val < threshold:
                robot.stop()
                break
            else:
                error = ctrl_val - threshold
                robot.drive(100, kp * error)
            wait(10)

        # 2) 검 → 흰
        while True:
            L = left.reflection()
            R = right.reflection()

            edge_val = R if use_right else L
            ctrl_val = L if use_right else R

            if edge_val > threshold:
                robot.stop()
                break
            else:
                error = ctrl_val - threshold
                robot.drive(100, kp * error)
            wait(10)

def turn_min(now_dir, target_dir):
    diff = (target_dir - now_dir) % 4
    angle = [0, 90, 180, -90][diff]
    robot.turn(angle)
    return target_dir

def move_manhattan_step(start_xy, goal_xy, now_dir):
    target_dir = direction_between(start_xy, goal_xy)
    if target_dir is None:
        return start_xy, now_dir

    # 최소 회전
    now_dir = turn_min(now_dir, target_dir)
    # 해당 방향으로 라인트레이싱 한 칸
    follow_line_one_cell_move_dir(start_xy, target_dir)

    # (1,1) / (2,1) 도착 시 잠깐 정지
    if goal_xy == (1, 1) or goal_xy == (2, 1):
        robot.stop()
        wait(100)
        robot.straight(30)

    # (0,1) 도착 시 보정
    if goal_xy == (0, 1):
        robot.stop()
        wait(100)
        robot.straight(30)
        wait(100)

    return goal_xy, now_dir

def bfs(s, g):
    dist = [[None] * GRID_W for _ in range(GRID_H)]
    prev = [[None] * GRID_W for _ in range(GRID_H)]
    q = deque([s])
    dist[s[1]][s[0]] = 0

    while q:
        x, y = q.popleft()
        if (x, y) == g:
            break
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy

            if not can_move(x, y, nx, ny):
                continue

            if dist[ny][nx] is None:
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

# ---------------- 목표존까지 이동 + 복귀 ----------------
def go_to_goal_with_bfs():
    global grabbed_cell, grabbed_pos, grabbed_color, current_cell

    if grabbed_pos is None or grabbed_color is None:
        print("No grabbed block info.")
        return

    # 1) 색깔에 따라 목표 존 선택
    if grabbed_color == "red":
        goal = RED_GOAL
    else:
        goal = BLUE_GOAL

    start = grabbed_pos

    print("===== GO TO GOAL BFS =====")
    print("start:", start, " color:", grabbed_color, " goal:", goal)
    path, dist = bfs(start, goal)
    print("path:", path)

    if not path or len(path) <= 1:
        print("No path or already at goal.")
        return

    # 시작 방향: N으로 가정
    now_dir = N
    current = path[0]

    # 경로 따라 이동 → 목표 존 도착
    step_idx = 0
    for nxt in path[1:]:
        step_idx += 1
        target_dir = direction_between(current, nxt)
        print("[GO] step", step_idx, ":", current, "->", nxt, " dir(need):", target_dir)
        current, now_dir = move_manhattan_step(current, nxt, now_dir)

    print("Arrived at goal:", goal, " dir(now):", now_dir)
    robot.stop()
    wait(200)

    # 블록 내려놓기
    release_object()
    wait(200)

    # 앞으로 밀어넣기
    robot.straight(100)
    wait(100)

    # 다시 후진해서 원위치
    robot.straight(-100)
    wait(100)

    # 180도 회전 (이제 필드 안쪽을 보게 됨)
    robot.turn(180)
    # ★ 실제 방향도 180도 반대로 갱신
    now_dir = opposite_dir(now_dir)
    print("Turn 180 done. now_dir(opposite):", now_dir)
    wait(200)

    # ---------- 다시 블록 발견 위치로 복귀 ----------
    start_back = goal
    goal_back = grabbed_pos

    print("===== RETURN BFS =====")
    print("BFS back from", start_back, "to", goal_back)
    path_back, dist_back = bfs(start_back, goal_back)
    print("path_back:", path_back)

    if path_back and len(path_back) > 1:
        current = path_back[0]
        step_idx = 0
        for nxt in path_back[1:]:
            step_idx += 1
            target_dir = direction_between(current, nxt)
            # 우리가 가야 할 방향을 dir로 찍기
            print("[BACK] step", step_idx, ":", current, "->", nxt, " dir(need):", target_dir)
            current, now_dir = move_manhattan_step(current, nxt, now_dir)
    else:
        print("No path_back or already at original position.")

    current_cell = grabbed_cell
    print("Back at original cell:", current_cell, " pos:", grabbed_pos)

    # ★ 블록 잡은 셀을 제외한 남은 L_ORDER 출력
    if grabbed_cell in L_ORDER:
        idx = L_ORDER.index(grabbed_cell)
        remaining = L_ORDER[idx+1:]
        print("Remaining L_ORDER after grabbed cell", grabbed_cell, ":", remaining)

    robot.stop()
    wait(300)
    search_L_shape()

# ---------------- 장애물 감지 → 잡기 + 색 판단 + BFS 이동 ----------------
def handle_obstacle_and_grab():
    global grabbed_cell, grabbed_pos, grabbed_color, current_cell

    robot.stop()
    grab_object()
    robot.straight(100)
    wait(300)

    color_name = classify_block_color()
    grabbed_color = color_name

    grabbed_cell = current_cell
    grabbed_pos = CELL_POS.get(grabbed_cell, None)

    print("=== BLOCK INFO ===")
    print("cell:", grabbed_cell)
    print("pos :", grabbed_pos)
    print("color:", grabbed_color)

    if grabbed_pos is not None:
        go_to_goal_with_bfs()

    return True

# ---------- ㄹ자 탐색: 부분 재개용 헬퍼들 ----------

def l_from_3_onward():
    """현재 cell=3 에서 시작해 7→6→5→9→10→11 진행."""
    global current_cell

    # 3 → 7 : 아래로 한 칸
    robot.turn(90)
    current_cell = 7
    found = follow_line_one_cell()
    if found:
        return

    l_from_7_onward()

def l_from_7_onward():
    """현재 cell=7 에서 시작해 6→5→9→10→11 진행."""
    global current_cell

    # 7 → 6 → 5 : 왼쪽으로 두 칸
    robot.turn(90)
    for cell_id in [6, 5]:
        current_cell = cell_id
        found = follow_line_one_cell()
        if found:
            return

    l_from_5_onward()

def l_from_6_onward():
    """현재 cell=6 에서 시작해 5→9→10→11 진행."""
    global current_cell

    current_cell = 5
    found = follow_line_one_cell()
    if found:
        return

    l_from_5_onward()

def l_from_5_onward():
    """현재 cell=5 에서 시작해 9→10→11 진행."""
    global current_cell

    # 5 → 9 : 아래로 한 칸 (왼쪽 센서 기준)
    robot.turn(-90)
    current_cell = 9
    found = follow_line_one_cell_left()
    if found:
        return

    l_from_9_onward()

def l_from_9_onward():
    """현재 cell=9 에서 시작해 10→11 진행."""
    global current_cell

    robot.turn(90)
    for cell_id in [10, 11]:
        current_cell = cell_id
        found = follow_line_one_cell()
        if found:
            return

def l_from_10_onward():
    """현재 cell=10 에서 시작해 11만 진행."""
    global current_cell

    current_cell = 11
    found = follow_line_one_cell()
    if found:
        return

# ---------------- ㄹ자 전역 탐색 ----------------
def search_L_shape():
    """
    ㄹ자 탐색 순서:
      L_ORDER = [1, 2, 3, 7, 6, 5, 9, 10, 11]

    - 처음 시작(current_cell=None) : 전체 순서대로 탐색
    - 블록 잡고 복귀한 후: current_cell 이후 셀만 계속 탐색
    """
    global current_cell

    # 1) 처음 시작: current_cell 이 아직 정해지지 않은 경우
    if current_cell is None:
        # 맨 위 줄: [준비, 셀1, 셀2, 셀3]
        top_cells_for_step = [None, 1, 2, 3]
        for cell_id in top_cells_for_step:
            if cell_id is not None:
                current_cell = cell_id
            found = follow_line_one_cell()
            if found:
                return

        # 3 이후부턴 공통 헬퍼 사용
        l_from_3_onward()
        return

    # 2) 블록 복귀 후: current_cell 에 따라 분기

    # --- 왼쪽 세로줄 (1,2,3)에서 복귀한 경우 ---
    if current_cell == 1:
        # 1은 이미 봤으니 2,3만
        for cell_id in [2, 3]:
            current_cell = cell_id
            found = follow_line_one_cell()
            if found:
                return
        l_from_3_onward()
        return

    if current_cell == 2:
        # 1,2는 이미 봤으니 3부터
        current_cell = 3
        found = follow_line_one_cell()
        if found:
            return
        l_from_3_onward()
        return

    if current_cell == 3:
        l_from_3_onward()
        return

    # --- 위 가로줄 오른쪽 (7,6,5)에서 복귀한 경우 ---
    if current_cell == 7:
        l_from_7_onward()
        return

    if current_cell == 6:
        l_from_6_onward()
        return

    if current_cell == 5:
        l_from_5_onward()
        return

    # --- 오른쪽 세로줄 (9,10,11)에서 복귀한 경우 ---
    if current_cell == 9:
        l_from_9_onward()
        return

    if current_cell == 10:
        l_from_10_onward()
        return

    if current_cell == 11:
        # 이미 ㄹ자 탐색 끝까지 온 상태
        return

    # 예외적인 값이면 그냥 종료
    return

# ---------------- 메인 ----------------
if __name__ == "__main__":
    release_object()
    search_L_shape()
