"""Standalone example nodes: expose an Action result as a Future for their executor.

Does not spin a node; the caller must keep its existing executor running. These
examples use native ActionClient, not the public ROS2RobotInterface wrapper.
"""
from action_msgs.msg import GoalStatus
from rclpy.task import Future
from arms_ros2_control_msgs.action import JointTrajectory


def send_joint_trajectory_action(client, goal):
    result = Future()

    def received_result(future):
        try:
            wrapped = future.result()
            response = wrapped.result
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                response.success = False
            result.set_result(response)
        except Exception as exc:
            result.set_exception(exc)

    def accepted(future):
        try:
            handle = future.result()
            if not handle.accepted:
                result.set_result(JointTrajectory.Result(success=False, message='Goal rejected'))
                return
            handle.get_result_async().add_done_callback(received_result)
        except Exception as exc:
            result.set_exception(exc)

    client.send_goal_async(goal).add_done_callback(accepted)
    return result
