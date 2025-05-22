import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

class GripperCommander(Node):
    def __init__(self):
        super().__init__('gripper_commander')
        self.client = ActionClient(self, MoveGroup, '/move_action')

        # 👉 這裡設定夾爪目標位置（0 = 張開，0.85 = 閉合）
        self.gripper_position = 0.80  # ✅ 修改這裡就好

    def send_goal(self):
        while not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('等待 MoveIt gripper server 中...')

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'xarm_gripper'

        constraint = Constraints()
        jc = JointConstraint()
        jc.joint_name = 'drive_joint'
        jc.position = self.gripper_position
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        constraint.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(constraint)

        self.client.send_goal_async(goal_msg).add_done_callback(self.goal_callback)

    def goal_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('✅ 夾爪指令已接受，等待結果...')
            goal_handle.get_result_async().add_done_callback(self.result_callback)
        else:
            self.get_logger().error('❌ 夾爪動作被拒絕')

    def result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('🎉 成功控制夾爪位置！')
        else:
            self.get_logger().error(f'⚠️ 控制失敗，錯誤代碼：{result.error_code.val}')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = GripperCommander()
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
