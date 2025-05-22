import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import math

class SimpleArmCommander(Node):
    def __init__(self):
        super().__init__('simple_arm_commander')
        self.client = ActionClient(self, MoveGroup, '/move_action')

    def send_goal(self):
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('等待 MoveIt server 中...')

        # 建立 MoveGroup goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'xarm6'

        # 設定每個 joint 的目標角度（degree → radians）
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_angles = [0, 0, 0, 0, 0, 0]

        constraint = Constraints()
        for name, deg in zip(joint_names, joint_angles):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = math.radians(deg)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraint.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraint)

        # 發送目標
        self.client.send_goal_async(goal_msg, feedback_callback=None).add_done_callback(self.goal_callback)

    def goal_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('✅ 已接受，等結果...')
            goal_handle.get_result_async().add_done_callback(self.result_callback)
        else:
            self.get_logger().error('❌ 被拒絕了')

    def result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('🎉 成功移動到目標角度！')
        else:
            self.get_logger().error(f'⚠️ 失敗，代碼：{result.error_code.val}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleArmCommander()
    node.send_goal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
