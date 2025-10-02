import numpy as np
import random

# 5x5 미로 정의 (0: 통로, 1: 벽)
maze = np.array([
    [0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 1, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 0, 0]
])

SIZE = maze.shape[0]
START = (0,0)
GOAL = (4,4)

# 상태 개수: (x,y) 좌표 → 정수로 변환
def coord_to_state(x,y): return y*SIZE+x
def state_to_coord(s): return (s%SIZE, s//SIZE)

# 행동 정의 (상, 우, 하, 좌)
ACTIONS = {0:(0,-1), 1:(1,0), 2:(0,1), 3:(-1,0)}

# 환경 step 함수
def step(pos, action):
    x,y = pos
    dx,dy = ACTIONS[action]
    nx,ny = x+dx,y+dy

    # 경계 및 벽 체크
    if 0<=nx<SIZE and 0<=ny<SIZE and maze[ny][nx]==0:
        new_pos = (nx,ny)
    else:
        new_pos = (x,y)  # 못 가면 제자리

    # 보상
    if new_pos == GOAL:
        reward, done = 100.0, True
    else:
        reward, done = -1.0, False
    return new_pos, reward, done

# Q-learning 학습
N_STATES = SIZE*SIZE
N_ACTIONS = 4
Q = np.zeros((N_STATES,N_ACTIONS))

alpha,gamma = 0.2,0.99
epsilon,eps_min,eps_decay = 1.0,0.05,0.95
rng = np.random.default_rng(0)

for ep in range(100):
    pos = START
    s = coord_to_state(*pos)
    total = 0
    done = False

    while not done:
        # ε-greedy 선택
        if rng.random() < epsilon:
            a = rng.integers(N_ACTIONS)
        else:
            a = np.argmax(Q[s])
        
        pos2,r,done = step(pos,a)
        s2 = coord_to_state(*pos2)
        
        # Q 업데이트
        Q[s,a] += alpha*(r + gamma*np.max(Q[s2]) - Q[s,a])
        
        pos, s = pos2, s2
        total += r

    epsilon = max(eps_min, epsilon*eps_decay)
    if (ep+1)%10==0:
        print(f"Episode {ep+1} | Return {total} | epsilon {epsilon:.3f}")

# 학습된 정책으로 주행
pos = START
path = [pos]
total=0
done=False
while not done:
    s = coord_to_state(*pos)
    a = np.argmax(Q[s])
    pos,r,done = step(pos,a)
    path.append(pos)
    total += r
print("\n최종 경로:", path)
print("총 보상:", total, " | 목표 도달:", done)
