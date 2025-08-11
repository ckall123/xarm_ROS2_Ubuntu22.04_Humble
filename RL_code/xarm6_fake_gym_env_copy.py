import gymnasium as gym
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from stable_baselines3.common.callbacks import BaseCallback

# =========================================================
# 簡單模擬物體
# =========================================================
class SimObject:
    def __init__(self, name, position):
        self.name = name
        self.position = np.array(position, dtype=np.float32)
        self.attached = False


# =========================================================
# TensorBoard Callback
# =========================================================
class TensorboardCallback(BaseCallback):
    """
    自動紀錄 reward 和 episode 長度的 TensorBoard callback
    """
    def __init__(self, verbose=0):
        super().__init__(verbose)
        self.episode_rewards = []
        self.episode_lengths = []

    def _on_step(self) -> bool:
        # 收集 reward
        if "episode" in self.locals:
            ep_info = self.locals["episode"]
            self.logger.record("reward/episode_reward", ep_info["r"])
            self.logger.record("episode/length", ep_info["l"])
        return True


# =========================================================
# XArm6 Gym Environment
# =========================================================
class XArm6GymEnv(gym.Env):
    """
    XArm6 強化學習環境，可 Fake 或 ROS 模式
    """

    metadata = {"render_modes": []}

    def __init__(self, mode="fake", fk_mode="simple", enable_collision=False, table_height=0.0):
        super().__init__()

        self.mode = mode
        self.fk_mode = fk_mode
        self.enable_collision = enable_collision
        self.table_height = table_height
        self.use_fake = (mode == "fake")

        # --- ROS Node ---
        if not self.use_fake:
            rclpy.init(args=None)
            self.node = Node('xarm6_gym_env')

        # --- xArm6 Joint Limits ---
        self.JOINT_LIMITS = np.array([
            [-3.1067,  3.1067],
            [-2.0595,  2.0944],
            [-3.1067,  0.1919],
            [-3.1067,  3.1067],
            [-1.6929,  3.1067],
            [-3.1067,  3.1067]
        ], dtype=np.float32)

        # --- ROS Pub/Sub ---
        if not self.use_fake:
            self.cmd_pub = self.node.create_publisher(
                JointTrajectory, '/xarm6_traj_controller/joint_trajectory', 10)
            self.state_sub = self.node.create_subscription(
                JointState, '/joint_states', self._joint_state_cb, 10)

        # --- Gym Spaces ---
        self.action_space = gym.spaces.Box(
            low=np.array([-1]*6 + [0.0], dtype=np.float32),
            high=np.array([1]*6 + [0.8552], dtype=np.float32)
        )

        # --- 模擬物體 ---
        self.objects = [
            SimObject("cube1", [0.5, 0.0, 0.05]),
            SimObject("cylinder1", [0.6, 0.1, 0.05]),
        ]
        obs_dim = 7 + len(self.objects)*3
        self.observation_space = gym.spaces.Box(
            low=np.array([-np.pi]*6 + [0] + [-np.inf]*(len(self.objects)*3), dtype=np.float32),
            high=np.array([np.pi]*6 + [1] + [np.inf]*(len(self.objects)*3), dtype=np.float32)
        )

        # --- 狀態變數 ---
        self.current_joint_state = np.zeros(6, dtype=np.float32)
        self.gripper_angle = 0.0
        self.attached_object = None

    # =========================================================
    # ROS Callback
    # =========================================================
    def _joint_state_cb(self, msg):
        self.current_joint_state = np.array(msg.position[:6], dtype=np.float32)

    # =========================================================
    # Forward Kinematics
    # =========================================================
    def _compute_ee_position(self, joint_angles):
        if self.fk_mode == "real":
            # TODO: 真實 FK 計算
            pass
        # 簡化版 FK
        x = 0.3 + 0.2*np.cos(joint_angles[0]) + 0.2*np.cos(joint_angles[0]+joint_angles[1])
        y = 0.0 + 0.2*np.sin(joint_angles[0]) + 0.2*np.sin(joint_angles[0]+joint_angles[1])
        z = 0.05 + 0.1*(joint_angles[2] + 1.5)
        return np.array([x, y, max(0.0, z)], dtype=np.float32)

    # =========================================================
    # Attach / Detach
    # =========================================================
    def attach_object(self, obj):
        self.attached_object = obj
        obj.attached = True

    def detach_object(self):
        if self.attached_object:
            self.attached_object.attached = False
            self.attached_object = None

    # =========================================================
    # Observation
    # =========================================================
    def _get_obs(self):
        obs = list(self.current_joint_state) + [self.gripper_angle]
        for obj in self.objects:
            obs.extend(obj.position)
        return np.array(obs, dtype=np.float32)

    # =========================================================
    # Reward
    # =========================================================
    def _compute_reward(self, obs):
        ee_pos = self._compute_ee_position(self.current_joint_state)
        objects_pos = obs[7:].reshape(-1, 3)

        # 最近物體距離
        min_dist = np.inf
        target_idx = -1
        for i, pos in enumerate(objects_pos):
            if np.linalg.norm(pos) == 0:
                continue
            dist = np.linalg.norm(ee_pos - pos)
            if dist < min_dist:
                min_dist = dist
                target_idx = i

        reward = max(0.0, 1.0 - min_dist / 0.5)

        # 夾取獎勵
        if self.attached_object is not None:
            reward += 2.0
            obj_pos = objects_pos[target_idx]
            if obj_pos[2] > 0.15:
                reward += 1.0

        # 簡單碰撞懲罰
        if self.enable_collision:
            if ee_pos[2] < self.table_height:
                reward -= 1.0

        return reward

    # =========================================================
    # Step
    # =========================================================
    def step(self, action):
        self.gripper_angle = np.clip(self.gripper_angle + action[6], 0.0, 0.8552)

        # 動作
        if not self.use_fake:
            traj = JointTrajectory()
            traj.joint_names = [f'joint{i+1}' for i in range(6)]
            point = JointTrajectoryPoint()
            point.positions = (self.current_joint_state + action[:6]).tolist()
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 50000000
            traj.points.append(point)
            self.cmd_pub.publish(traj)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        else:
            self.current_joint_state = np.clip(
                self.current_joint_state + action[:6],
                self.JOINT_LIMITS[:, 0],
                self.JOINT_LIMITS[:, 1]
            )

        ee_pos = self._compute_ee_position(self.current_joint_state)

        # Attach / Detach
        if self.gripper_angle < 0.02:
            self.detach_object()
        elif self.gripper_angle > 0.6 and self.attached_object is None:
            for obj in self.objects:
                if np.linalg.norm(ee_pos - obj.position) < 0.05:
                    self.attach_object(obj)
                    break

        # 更新物體位置
        if self.attached_object:
            self.attached_object.position = ee_pos + np.array([0,0,-0.05],dtype=np.float32)

        obs = self._get_obs()
        reward = self._compute_reward(obs)

        done = False
        if self.attached_object and self.attached_object.position[2] > 0.2:
            done = True

        trunc = False
        return obs, reward, done, trunc, {}

    # =========================================================
    # Reset
    # =========================================================
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.current_joint_state = np.zeros(6, dtype=np.float32)
        self.gripper_angle = 0.0
        self.attached_object = None
        for i, obj in enumerate(self.objects):
            obj.position = np.array([0.5 + 0.1*i, 0.1*i, 0.05], dtype=np.float32)
            obj.attached = False
        obs = self._get_obs()
        return obs, {}

    # =========================================================
    # Close
    # =========================================================
    def close(self):
        if not self.use_fake:
            self.node.destroy_node()
            rclpy.shutdown()


# =========================================================
# 測試訓練腳本
# =========================================================
if __name__ == "__main__":
    from stable_baselines3 import SAC
    from stable_baselines3.common.env_checker import check_env

    env = XArm6GymEnv(mode="ros", fk_mode="simple", enable_collision=True, table_height=0.0)
    check_env(env, warn=True)

    model = SAC(
        policy="MlpPolicy",
        env=env,
        verbose=1,
        tensorboard_log="./sac_xarm6_tensorboard/"
    )

    model.learn(
        total_timesteps=10000,
        log_interval=1,
        callback=TensorboardCallback()
    )

    model.save("sac_xarm6_model")
    env.close()
