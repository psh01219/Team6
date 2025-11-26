#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()

left = Motor(Port.A)
right = Motor(Port.D)
arm = Motor(Port.C)


robot = DriveBase(left,right,55.5,104)
left = ColorSensor(Port.S1)
right = ColorSensor(Port.S4)
object_detector = ColorSensor(Port.S3)
ultra_sensor = UltrasonicSensor(Port.S2)


threshold = 40
kp = 0.7

N,E,S,W = 1,2,3,4

def left_line_following(speed,kp) :
    threshold = 40
    left_reflection = left.reflection()
    error = left_reflection - threshold
    turn_rate = kp * error
    robot.drive(speed,turn_rate)

def right_line_following(speed,kp) :
    threshold = 40
    right_reflection = right.reflection()
    error = right_reflection - threshold
    turn_rate = kp * error
    robot.drive(speed,turn_rate)

def n_move(n,direction="right") :
    for _ in range(n) :
        if direction == "right" :
            while right.reflection() > 50:
                left_line_following(100,1.2)
            while left.reflection() <=50:
                right_line_following(100,1.2)
        elif direction =="left" :
            while left.reflection() > 50:
                left_line_following(100,1.2)
            while right.reflection() <=50:
                right_line_following(100,1.2)
    robot.stop()

def grap_object() :
    arm.run_until_stalled(200, then = Stop.COAST, duty_limit=50)
def release_object() :
    arm.run_until_stalled(-200, then = Stop.COAST, duty_limit=50)    

def follow_line_one_cell() :
    for i in range(1):
        while True:
            left_reflection = left.reflection()
            right_reflection = right.reflection()

            if right_reflection < threshold:
                robot.stop()
                break
            else:
                error = left_reflection - threshold
                turn_rate = kp * error
                robot.drive(100,turn_rate)
            wait(10)    

        while True:
            left_reflection = left.reflection()
            right_reflection = right.reflection()

            if right_reflection > threshold:
                robot.stop()
                break
            else:
                error = left_reflection - threshold
                turn_rate = kp * error
                robot.drive(100,turn_rate)
            wait(10) 

from collections import deque
MAP = [
".....",
"..#..",
"..#.."

]

G = [[1 if c == "#" else 0 for c in r] for r in MAP]
W = len(G[0])
H = len(G)


def turn_min(now_dir, target_dir) :
    diff = (target_dir-now_dir) %4
    angle = [0,90,180,-90][diff]
    robot.turn(angle)
    return target_dir


def move_manhattan(start_xy, goal_xy, now_dir) :

    x,y = start_xy

    gx,gy = goal_xy
    dx = gx - x
    dy = gy - y
    if dx !=0:
        target_dir = E if dx > 0 else W

        now_dir = turn_min(now_dir, target_dir)
        steps = abs(dx)
        for _ in range(steps):
            follow_line_one_cell()
            x += 1 if target_dir == E else -1

    if dy !=0:
        target_dir = N if dy >0 else S
        now_dir = turn_min(now_dir,target_dir)
        steps = abs(dy)
        for _ in range(steps):
            follow_line_one_cell()
            y += 1 if target_dir == N else -1
    return (x,y) , now_dir        


def bfs(s,g) :
    dist = [[None]*W for _ in range(H)]
    prev = [[None]*W for _ in range(H)]
    q = deque([s])
    dist[s[1]][s[0]] = 0
    while q:
        x,y = q.popleft()
        if (x,y) ==g:
            break
        for dx,dy in ((1,0), (-1,0), (0,1), (0,-1)) :
            nx,ny = x+dx, y+dy
            if 0 <= nx < W and 0 <= ny <H and not G[ny][nx] and dist[ny][nx] is None:
                dist[ny][nx] = dist[y][x]+1
                prev[ny][nx] = (x,y)
                q.append((nx,ny))

    path = []
    if dist[g[1]][g[0]] is not None:
        p =g
        while p:
            path.append(p)
            p = prev[p[1]][p[0]]
        path.reverse()
    return path, dist        


# def show(path,dist) :
#     grid = [['#' if G[y][x] else '.' for x in range(W)] for y in range(H)]
#     for x,y in path:
#         if(x,y) not in (S,E): grid[y][x] = '*'
#     grid[S[1]][S[0]], grid[E[1]][E[0]] = 'S','G'
#     print("=== 경로 맵 ===")
#     for r in grid: print(''.join(r))
#     print("\n==== 거리 히트맵 ===")
#     for r in dist: print(' '.join(' .' if d is None else "%2d" % d for d in r))
#     print("\n경로:" , path)
#     print("길이:", len(path)-1 if path else "없음")

# if __name__ =="__main__" :

#     path,dist = bfs(S,E)
#     print("===현재 MAP 출력 ===")
#     for r in MAP:
#         print(r)
#     print()
#     print("12")
#     show(path,dist)  

path1,dist1 = bfs((0,0),(2,0))
print(path1[0], path1[-1])
path2,dist2 = bfs((2,0), (1,0))

release_object()    
robot.straight(450)
grap_object()
move_manhattan(path2[0],dist2[-1],E)
release_object()







move_manhattan(path1[0], path1[-1], E)
while True :
    distance = ultra_sensor.distance()
    print(distance)
    if distance < 100 :
        robot.stop()
        grap_object()












# turn_table = [0,90,180,-90]

# now_dir1 = 1
# target_dir1 = 4
# direction = (target_dir1 - now_dir1) %4
# angle1 = turn_table[direction]

# now_dir2 = 4
# target_dir2 = 1
# direction = (target_dir2 - now_dir2) %4
# angle2 = turn_table[direction]

# now_dir3 = 1
# target_dir3 = 2
# direction = (target_dir3 - now_dir3) %4
# angle3 = turn_table[direction]

# now_dir4 = 2
# target_dir4 = 1
# direction = (target_dir4 - now_dir4) %4
# angle4 = turn_table[direction]
   

# while True:
#     left_reflection = cs1.reflection()
#     right_reflection = cs4.reflection()

#     if left_reflection < threshold:
#         robot.stop()
#         break
#     else:
#         error = right_reflection - threshold
#         turn_rate = kp*error
#         robot.drive(100,turn_rate)
#     wait(10)   

  
    
# robot.turn(angle1)

# for i in range(2):
#     while True:
#         left_reflection = cs1.reflection()
#         right_reflection = cs4.reflection()

#         if right_reflection < threshold:
#             robot.stop()
#             break
#         else:
#             error = left_reflection - threshold
#             turn_rate = kp * error
#             robot.drive(100,turn_rate)
#         wait(10)    

#     while True:
#         left_reflection = cs1.reflection()
#         right_reflection = cs4.reflection()

#         if right_reflection > threshold:
#             robot.stop()
#             break
#         else:
#             error = left_reflection - threshold
#             turn_rate = kp * error
#             robot.drive(100,turn_rate)
#         wait(10)    
    
# robot.turn(angle2)
# for i in range(2):
#     while True:
#         left_reflection = cs1.reflection()
#         right_reflection = cs4.reflection()

#         if right_reflection < threshold:
#             robot.stop()
#             break
#         else:
#             error = left_reflection - threshold
#             turn_rate = kp * error
#             robot.drive(100,turn_rate)
#         wait(10)    

#     while True:
#         left_reflection = cs1.reflection()
#         right_reflection = cs4.reflection()

#         if right_reflection > threshold:
#             robot.stop()
#             break
#         else:
#             error = left_reflection - threshold
#             turn_rate = kp * error
#             robot.drive(100,turn_rate)
#         wait(10)    
    

# robot.turn(angle3)



# for i in range(1):
#     while True:
#         left_reflection = cs1.reflection()
#         right_reflection = cs4.reflection()

#         if right_reflection < threshold:
#             robot.stop()
#             break
#         else:
#             error = left_reflection - threshold
#             turn_rate = kp * error
#             robot.drive(100,turn_rate)
#         wait(10)    

#     while True:
#         left_reflection = cs1.reflection()
#         right_reflection = cs4.reflection()

#         if right_reflection > threshold:
#             robot.stop()
#             break
#         else:
#             error = left_reflection - threshold
#             turn_rate = kp * error
#             robot.drive(100,turn_rate)
#         wait(10)    
# wait(100)

# for i in range(1):
#     while True:
#         left_reflection = cs1.reflection()
#         right_reflection = cs4.reflection()

#         if right_reflection < threshold:
#             robot.stop()
#             break
#         else:
#             error = left_reflection - threshold
#             turn_rate = kp * error
#             robot.drive(100,turn_rate)
#         wait(10)    

#     while True:
#         left_reflection = cs1.reflection()
#         right_reflection = cs4.reflection()

#         if right_reflection > threshold:
#             robot.stop()
#             break
#         else:
#             error = left_reflection - threshold
#             turn_rate = kp * error
#             robot.drive(100,turn_rate)
#         wait(10)        
    
# robot.turn(angle4)

# while True:
#     left_reflection = cs1.reflection()
#     right_reflection = cs4.reflection()

#     if left_reflection < threshold:
#         robot.stop()
#         break
#     else:
#         error = right_reflection - threshold
#         turn_rate = kp * error
#         robot.drive(100,turn_rate)
#         wait(10)    


