#!/usr/bin/env python3

import rclpy
import DR_init
import numpy as np
import time

# 로봇 설정
ROBOT_ID    = "dsr01"
ROBOT_MODEL = "m0609"
FORCE_THRESHOLD = 15.0  # [N]
STIFFNESS = [100, 100, 100, 20, 20, 20]

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("joint_and_compliance_with_force_interrupt", namespace=ROBOT_ID)

    # DSR API 초기화
    DR_init.__dsr__node   = node
    DR_init.__dsr__id     = ROBOT_ID
    DR_init.__dsr__model  = ROBOT_MODEL

    from DSR_ROBOT2 import amovej, DR_Error, wait, get_tool_force
    from DSR_ROBOT2 import task_compliance_ctrl, get_current_pose, release_compliance_ctrl
    from DSR_ROBOT2 import ROBOT_SPACE_TASK

    try:
        # 1단계: joint path 따라 이동
        try:
            joint_path = np.load("joint_path.npy")
            time.sleep(0.5)
        except FileNotFoundError:
            node.get_logger().error("❌ joint_path.npy 파일을 찾을 수 없습니다.")
            rclpy.shutdown()
            return

        node.get_logger().info(f"📍 총 {len(joint_path)}개의 관절 위치를 따라 이동을 시작합니다.")

        for i, joints in enumerate(joint_path):
            fx, fy, fz, *_ = get_tool_force()
            total_force = (fx**2 + fy**2 + fz**2)**0.5

            if total_force > FORCE_THRESHOLD:
                node.get_logger().warn(f"⚠️ 외력 감지됨: {total_force:.2f} N → 5초 대기 후 순응 제어 진입")
                time.sleep(5.0)

                current_pos = get_current_pose(ROBOT_SPACE_TASK)

                task_compliance_ctrl(STIFFNESS)
                node.get_logger().info(f"✅ 순응 제어 활성화됨 → 강성 설정: {STIFFNESS}")

                target_pos = [
                    current_pos[0] + 100.0,
                    current_pos[1],
                    current_pos[2],
                    current_pos[3],
                    current_pos[4],
                    current_pos[5]
                ]

                node.get_logger().info("🤖 순응제어 상태로 x축 +100mm 이동 실행 (amovej)")
                amovej(target_pos, vel=100, acc=100)

                node.get_logger().info("⏳ 이동 후 대기 중 (10초)...")
                wait(10.0)

                release_compliance_ctrl()
                wait(3.0)
                node.get_logger().info("🛑 순응 제어 해제 및 로봇 정지 완료")

                break  # 루프 종료

            node.get_logger().info(f"[{i+1}/{len(joint_path)}] 🚗 movej로 관절 이동 중...")
            try:
                amovej(joints.tolist(), v=10, a=10)
            except DR_Error as e:
                node.get_logger().warn(f"⚠️ movej 오류 발생: {e}")

    except DR_Error as e:
        node.get_logger().error(f"❗ DSR API 오류 발생: {e}")

    finally:
        node.get_logger().info("🔚 노드 종료 및 순응 제어 비활성화")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
