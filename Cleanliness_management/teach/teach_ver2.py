#!/usr/bin/env python3
import rclpy
import DR_init
import time
import numpy as np
import sys, select

ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("teach_joint_path", namespace=ROBOT_ID)

    DR_init.__dsr__node   = node
    DR_init.__dsr__id     = ROBOT_ID
    DR_init.__dsr__model  = ROBOT_MODEL

    from DSR_ROBOT2 import get_current_posj, release_compliance_ctrl, task_compliance_ctrl, DR_Error

    release_compliance_ctrl()
    #start_joint = [1.81, 13.67, 64.42, -0.01, 54.83, 0.00]  # ← 여기만 수정하면 됨
    #movej(start_joint, v=30, a=30)
    ultra_low_stx = [5, 5, 5, 0, 0, 0]
    task_compliance_ctrl(ultra_low_stx, 0)

    joint_paths = []
    count = 0  # 저장 횟수 카운터

    node.get_logger().info("Teach 모드 시작: 엔터 누르면 종료")

    try:
        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                _ = sys.stdin.readline()
                break

            try:
                joint_pos = get_current_posj()
            except DR_Error:
                node.get_logger().warn("get_current_joint_position() 오류, 재시도")
                time.sleep(0.01)
                continue

            if hasattr(joint_pos, "__len__") and len(joint_pos) == 6:
                count += 1
                node.get_logger().info(f"[{count}] 관절 위치 저장: {joint_pos}")
                joint_paths.append(list(joint_pos))
                time.sleep(2.0)  # 2초 간격 저장
            else:
                node.get_logger().warn("잘못된 관절 위치 데이터")
                time.sleep(0.01)

    except KeyboardInterrupt:
        pass

    np.save("joint_path.npy", np.array(joint_paths))
    node.get_logger().info(f"joint_path.npy 저장 완료, 총 {len(joint_paths)} 점")

    release_compliance_ctrl()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
