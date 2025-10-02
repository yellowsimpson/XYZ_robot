import gym
from gym import spaces
import numpy as np
from dataclasses import dataclass
import random

# ---------------------------
# 1) 커스텀 GridWorld 환경
# ---------------------------
@dataclass
class GridConfig:
    size: int = 6  # 좌표 0..5  (6x6 격자)
    start: tuple = (0, 0)
    goal: tuple = (5, 5)
    # 길이 3짜리 벽 2개 (예시)
    walls: tuple = ((1, 2), (2, 2), (3, 2),  # 가로벽 (y=2, x=1..3)
                    (4, 1), (4, 2), (4, 3))  # 세로벽 (x=4, y=1..3)
    max_steps: int = 200

class GridWorldEnv(gym.Env):
    """
    관측: Discrete(size*size)  → 상태번호 s = y*size + x
    행동: 0:상, 1:우, 2:하, 3:좌
    보상: 목표 도달 +100, 일반 이동 -1, 벽/경계로 못 움직이면 -1(제자리)
    에피소드 종료: 목표 도달 or 스텝 초과
    """
    metadata = {"render.modes": ["human"]}

    def __init__(self, config: GridConfig = GridConfig(), seed: int | None = None):
        super().__init__()
        self.cfg = config
        self.size = config.size
        self.start = config.start
        self.goal = config.goal
        self.walls = set(config.walls)
        self.max_steps = config.max_steps

        self.action_space = spaces.Discrete(4)
        self.observation_space = spaces.Discrete(self.size * self.size)

        self.pos = None
        self.steps = 0
        self.rng = np.random.default_rng(seed)

    def coord_to_state(self, x, y):
        return y * self.size + x

    def state_to_coord(self, s):
        y, x = divmod(s, self.size)[0], s % self.size  # (y, x)
        return x, y

    def reset(self, *, seed: int | None = None, options=None):
        if seed is not None:
            super().reset(seed=seed)
        self.pos = tuple(self.start)
        self.steps = 0
        return self.coord_to_state(*self.pos), {}  # obs, info

    def step(self, action):
        x, y = self.pos
        dx, dy = 0, 0
        if action == 0:   # 상
            dy = -1
        elif action == 1: # 우
            dx = 1
        elif action == 2: # 하
            dy = 1
        elif action == 3: # 좌
            dx = -1

        nx, ny = x + dx, y + dy

        # 경계 체크
        if not (0 <= nx < self.size and 0 <= ny < self.size):
            nx, ny = x, y  # 못 나감 → 제자리

        # 벽 체크
        if (nx, ny) in self.walls:
            nx, ny = x, y  # 벽이면 이동 불가(패널티는 이동 비용과 동일하게 -1로 둠)

        self.pos = (nx, ny)
        self.steps += 1

        # 보상 및 종료
        if self.pos == self.goal:
            reward = 100.0
            terminated = True
        else:
            reward = -1.0
            terminated = False

        truncated = self.steps >= self.max_steps
        obs = self.coord_to_state(*self.pos)
        info = {}
        return obs, reward, terminated, truncated, info

    def render(self, mode="human"):
        board = [[" . "]*self.size for _ in range(self.size)]
        for (wx, wy) in self.walls:
            board[wy][wx] = "[#]"
        sx, sy = self.start
        gx, gy = self.goal
        board[sy][sx] = " S "
        board[gy][gx] = " G "
        x, y = self.pos
        board[y][x] = " A "
        print("\n".join("".join(row) for row in board))
        print()

# --------------------------------
# 2) Q-learning(탐험 ε-greedy)
# --------------------------------
def q_learning_train(
    env: gym.Env,
    episodes: int = 100,
    alpha: float = 0.1,      # 학습률
    gamma: float = 0.99,     # 감가율
    epsilon: float = 1.0,    # 초기 탐험률
    epsilon_min: float = 0.05,
    epsilon_decay: float = 0.98,
    seed: int | None = 42
):
    rng = np.random.default_rng(seed)
    n_states = env.observation_space.n
    n_actions = env.action_space.n
    Q = np.zeros((n_states, n_actions))

    returns = []

    for ep in range(episodes):
        obs, _ = env.reset()
        done = False
        ep_ret = 0.0

        while not done:
            # ε-greedy
            if rng.random() < epsilon:
                action = env.action_space.sample()
            else:
                action = int(np.argmax(Q[obs]))

            next_obs, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated

            # TD 업데이트
            best_next = np.max(Q[next_obs])
            td_target = reward + gamma * best_next * (0.0 if terminated else 1.0)
            Q[obs, action] += alpha * (td_target - Q[obs, action])

            obs = next_obs
            ep_ret += reward

        returns.append(ep_ret)
        epsilon = max(epsilon_min, epsilon * epsilon_decay)

        if (ep + 1) % 10 == 0:
            print(f"Episode {ep+1:3d} | Return: {ep_ret:7.2f} | epsilon: {epsilon:.3f}")

    return Q, returns

# --------------------------------
# 3) 학습 후 경로 시각화(정책 실행)
# --------------------------------
def greedy_rollout(env: gym.Env, Q, render: bool = True, max_steps: int = 200):
    obs, _ = env.reset()
    traj = []
    done = False
    total_reward = 0.0
    steps = 0

    while not done and steps < max_steps:
        x = obs % env.size
        y = obs // env.size
        traj.append((x, y))

        action = int(np.argmax(Q[obs]))
        obs, reward, terminated, truncated, _ = env.step(action)
        done = terminated or truncated
        total_reward += reward
        steps += 1
        if render:
            env.render()

    x = obs % env.size
    y = obs // env.size
    traj.append((x, y))
    return traj, total_reward, done

# ---------------------------
# 4) 실행
# ---------------------------
if __name__ == "__main__":
    cfg = GridConfig(
        size=6,
        start=(0, 0),
        goal=(5, 5),
        walls=((1, 2), (2, 2), (3, 2), (4, 1), (4, 2), (4, 3)),
        max_steps=200,
    )
    env = GridWorldEnv(cfg, seed=0)

    print("=== 학습 시작 (100 에피소드) ===")
    Q, returns = q_learning_train(
        env,
        episodes=100,
        alpha=0.2,
        gamma=0.99,
        epsilon=1.0,
        epsilon_min=0.05,
        epsilon_decay=0.95,
        seed=0
    )

    print("\n=== 학습된 정책으로 롤아웃 ===")
    traj, total_reward, solved = greedy_rollout(env, Q, render=True)
    print("경로:", traj)
    print("총 보상:", total_reward, "| 목표 도달:", solved)
