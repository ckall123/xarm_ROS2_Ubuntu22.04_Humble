import gymnasium as gym
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from sim_object import SimObject
from spawner import randomize_objects  # 假設有這個 function 協助 reset 時刷新物件位置

class XArm6GymEnv(gym.Env):
    """ROS + Gym 環境整合的 xArm6 機械手臂環境，支援觀察正規化、碰撞處理、可插拔 Reward 設計。"""

    metadata = {"render_modes": []}

    _LIMITS = np.array([
        [-3.1067,  3.1067],
        [-2.0595,  2.0944],
        [-3.1067,  0.1919],
        [-3.1067,  3.1067],
        [-1.6929,  3.1067],
        [-3.1067,  3.1067],
    ], dtype=np.float32)

    def __init__(self, fk_mode="simple", enable_collision=False, table_height=0.0, max_steps=200):
        super().__init__()

        self.fk_mode = fk_mode
        self.enable_collision = enable_collision
        self.table_height = table_height
        self.max_steps = max_steps
        self.step_count = 0

        # 初始化 ROS
        rclpy.init(args=None)
        self.node: Node = Node("xarm6_gym_env")

        self.cmd_pub = self.node.create_publisher(JointTrajectory, "/xarm6_traj_controller/joint_trajectory", 10)
        self.state_sub = self.node.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

        # 定義動作空間與觀察空間
        self.action_space = gym.spaces.Box(
            low=np.array([-1] * 6 + [0.0], dtype=np.float32),
            high=np.array([1] * 6 + [0.8552], dtype=np.float32),
        )

        self.objects = randomize_objects()

        obs_dim = 7 + len(self.objects) * 3
        self.observation_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(obs_dim,), dtype=np.float32
        )

        self.current_joint_state = np.zeros(6, dtype=np.float32)
        self.gripper_angle: float = 0.0
        self.attached_object: SimObject | None = None

    def _joint_state_cb(self, msg: JointState):
        """接收當前關節狀態的回調"""
        self.current_joint_state = np.array(msg.position[:6], dtype=np.float32)

    def _compute_ee_position(self, joint_angles: np.ndarray) -> np.ndarray:
        """        
        簡化版 forward kinematics，估算手臂末端位置

        💡 它怎麼算的：
            x = 0.3 + 0.2 * cos(j1) + 0.2 * cos(j1 + j2)
            y = 0.0 + 0.2 * sin(j1) + 0.2 * sin(j1 + j2)
            z = 0.05 + 0.1 * (j3 + 1.5)

            這邊只用前 3 個關節（joint1, joint2, joint3） 來估算位置（超粗略但夠用）。
                x/y 是 2D 平面上連桿轉折造成的偏移量。
                z 是 joint3 像伸縮一樣影響高度（有加上 +1.5 是位移校正用）。
        """
        if self.fk_mode == "real":
            raise NotImplementedError("Real FK not yet implemented")
        x = 0.3 + 0.2 * np.cos(joint_angles[0]) + 0.2 * np.cos(joint_angles[0] + joint_angles[1])
        y = 0.0 + 0.2 * np.sin(joint_angles[0]) + 0.2 * np.sin(joint_angles[0] + joint_angles[1])
        z = 0.05 + 0.1 * (joint_angles[2] + 1.5)
        return np.array([x, y, max(0.0, z)], dtype=np.float32) # max(0.0, z) 防止 z 變負數 避免手臂穿牆穿地

    def attach_object(self, obj: SimObject):
        """將物體標記為抓取狀態"""
        self.attached_object = obj
        obj.attached = True

    def detach_object(self):
        """釋放抓取的物體"""
        if self.attached_object:
            self.attached_object.attached = False
            self.attached_object = None

    def _normalize_obs(self, raw_obs: np.ndarray) -> np.ndarray:
        """將觀察值正規化至 [-1, 1]，有助於 RL 訓練穩定性"""
        joint_scaled = 2 * (self.current_joint_state - self._LIMITS[:, 0]) / (self._LIMITS[:, 1] - self._LIMITS[:, 0]) - 1
        grip_scaled = 2 * self.gripper_angle / 0.8552 - 1
        object_scaled = raw_obs[7:] / 1.0  # 假設範圍約在 ±1.0 m
        return np.concatenate([joint_scaled, [grip_scaled], object_scaled])

    def _check_invalid_state(self) -> bool:
        """
        檢查是否進入撞擊地板或自身等無效狀態

        判斷條件有兩個：
            1. 末端執行器 z 軸低於桌面高度：
                ee_pos[2] < self.table_height：撞到桌子底下啦！→ 不行！

            2. joint 角度有 NaN（數值錯亂）：
                np.isnan(...)：例如某次控制出現爆炸數字，結果變成 NaN，這樣也會停住。
                
        在 step() 裡，這個函數會用來：
            決定 reward 要不要扣分（撞地扣 -1 分）
            判斷是不是 episode 要 done（出事就 done）
        """
        ee_pos = self._compute_ee_position(self.current_joint_state)
        return ee_pos[2] < self.table_height or np.any(np.isnan(self.current_joint_state))

    def _get_obs(self):
        """蒐集觀察資料（joint 角度、物體位置），並正規化"""
        raw_obs = list(self.current_joint_state) + [self.gripper_angle]
        for obj in self.objects:
            raw_obs.extend(obj.position)
        return self._normalize_obs(np.asarray(raw_obs, dtype=np.float32))

    def _compute_reward(self, obs: np.ndarray) -> float:
        """
        計算當前步驟的 reward 分數
        TODO: 將此函數改寫成支援 VLM (Vision Language Model) feedback 的形式。
        可拆分為多層 reward 組件，便於改寫與 debug。
        """
        reward = 0.0
        ee_pos = self._compute_ee_position(self.current_joint_state)
        object_positions = obs[7:].reshape(-1, 3)
        dists = np.linalg.norm(object_positions - ee_pos, axis=1)
        reward += max(0.0, 1.0 - dists.min() / 0.5)

        if self.attached_object:
            reward += 2.0
            if self.attached_object.position[2] > 0.15:
                reward += 1.0

        if self.enable_collision and self._check_invalid_state():
            reward -= 1.0

        return reward

    def step(self, action: np.ndarray):
        """執行一步動作，更新觀察與狀態"""
        self.step_count += 1
        self.gripper_angle = np.clip(self.gripper_angle + action[6], 0.0, 0.8552)

        traj = JointTrajectory()
        traj.joint_names = [f"joint{i+1}" for i in range(6)]
        point = JointTrajectoryPoint()
        point.positions = (self.current_joint_state + action[:6]).tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50_000_000
        traj.points.append(point)
        self.cmd_pub.publish(traj)
        rclpy.spin_once(self.node, timeout_sec=0.05)

        ee_pos = self._compute_ee_position(self.current_joint_state)

        if self.gripper_angle < 0.02:
            self.detach_object()
        elif self.gripper_angle > 0.6 and self.attached_object is None:
            for obj in self.objects:
                if np.linalg.norm(ee_pos - obj.position) < 0.05:
                    self.attach_object(obj)
                    break

        if self.attached_object:
            self.attached_object.position = ee_pos + np.array([0, 0, -0.05], dtype=np.float32)

        obs = self._get_obs()
        reward = self._compute_reward(obs)
        done = self._check_invalid_state() or (self.attached_object and self.attached_object.position[2] > 0.2)
        truncated = self.step_count >= self.max_steps

        info = {
            "ee_pos": ee_pos.tolist(),
            "gripper": self.gripper_angle,
            "attached": self.attached_object.name if self.attached_object else None,
        }
        return obs, reward, done, truncated, info

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        """重置環境狀態，包括物件位置與關節姿態"""
        super().reset(seed=seed)
        self.step_count = 0
        self.current_joint_state[:] = 0.0
        self.gripper_angle = 0.0
        self.detach_object()

        self.objects = randomize_objects()
        obs = self._get_obs()
        return obs, {}

    def close(self):
        """關閉 ROS node"""
        self.node.destroy_node()
        rclpy.shutdown()
