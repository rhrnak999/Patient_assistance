import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("rokey_stacking", namespace=ROBOT_ID)
DR_init.__dsr__node = node

ON, OFF = 1, 0
HOME_READY = [0, 0, 90, 0, 90, 0]

import time

try:
    from DSR_ROBOT2 import (
        get_digital_input,
        set_digital_output,
        check_force_condition,
        release_compliance_ctrl,
        task_compliance_ctrl,
        set_tool,
        set_tcp,
        movej,
        movel,
        wait,
        mwait,
        DR_AXIS_Z, DR_TOOL,
    )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2 : {e}")
    exit()

set_tool("Tool Weight_2FG")
set_tcp("2FG_TCP")

def wait_digital_input(sig_num):
    while not get_digital_input(sig_num):
        wait(0.5)
        print("Wait for digital input")
        pass

def release():
    set_digital_output(2, ON)
    set_digital_output(1, OFF)
    wait_digital_input(2)

def grip():
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait_digital_input(1)

# 박스 ID에 따른 사전 정의된 좌표 (저장 위치)
PREDEFINED_POSITIONS = {
    "1f": [644.69, -432.91, 316.32, 86.79, -92.00, -94.05],
    "2f": [605.97, -415.97, 507.91, 92.01, -88.80, 93.95],
    # 필요 시 더 추가 가능
}

# 박스 ID별 홈 좌표 맵
HOME_POS_MAP = {
    "1f": [19.17, -1.25, 46.47, 66.66, 94.91, 104.80],
    "2f": [367.96, 6.88, 198.58, 0.86, 179.76, 0.73],
    # 추가 ID가 생기면 여기에 정의
}

# 환자 위치 (공통)
PATIENT_PATH = [361.75, 378.45, 159.51, 1.94, 99.14, -96.28]

# HOME_POSE_HOME (returning 명령 시 출발/종료 지점)
HOME_POSE_HOME = [367.96, 6.88, 198.58, 0.86, 179.76, 0.73]

class Box:
    def __init__(self, id, pos_id, position):
        self.id = id
        self.pos_id = pos_id
        self.position = position
        self.target_offset = 100
        self.stacked = False

    def set_pos_id(self, pos_id):
        self.pos_id = pos_id

    def set_box_id(self, id):
        self.id = id
    
    def set_position(self, pos_list):
        self.position = pos_list
    
    def info(self):
        return f"id : {self.id}\nposition : {self.pos_id} -> {self.position}\nstacked : {self.stacked}\n=====\n"

    def __move_to_pos(self, target_pos, action=None):
        """
        1) BOX ID에 맞춘 HOME_POS로 이동
        2) target_pos 위 안전 지점(ready_pos)으로 이동
        3) target_pos로 내려가서 action(grip/release) 수행
        4) 다시 ready_pos로 복귀
        5) HOME_READY로 복귀
        """
        # ① id에 맞춰 HOME_POS 선택
        if self.id in HOME_POS_MAP:
            HOME_POS = HOME_POS_MAP[self.id]
        else:
            # 정의되지 않은 ID인 경우 HOME_READY를 기본값으로 사용
            HOME_POS = HOME_READY.copy()

        # 1) 홈 좌표로 이동
        movej(HOME_POS, vel=VELOCITY, acc=ACC)
        mwait()
        
        # 2) target_pos 위 안전 지점(ready_pos) 계산
        ready_pos = target_pos.copy()
        ready_pos[1] += 150   # y축으로 150mm 앞쪽
        ready_pos[2] += 20    # z축으로 20mm 위쪽
        mwait()
        
        # 3) ready_pos로 이동 → 안전 확보
        movel(ready_pos, vel=VELOCITY, acc=ACC, mod=0)
        mwait()
        
        # 4) target_pos로 정확히 내려감
        movel(target_pos, vel=VELOCITY, acc=ACC, mod=0)
        mwait()

        # 5) action 수행 (grip 혹은 release)
        if action == 'grip':
            grip()
            mwait()
        elif action == 'release':
            release()
            mwait()
        
        # 6) 다시 ready_pos로 복귀
        movel(ready_pos, vel=VELOCITY, acc=ACC, mod=0)
        mwait()
        
        # 7) HOME_READY로 복귀
        movej(HOME_READY, vel=VELOCITY, acc=ACC)
        return target_pos

    def __move_to_patient(self):
        """
        환자 위치로 이동만 수행(그리핑/릴리스는 하지 않음)
        """
        # 1) HOME_READY로 먼저 복귀
        movej(HOME_READY, vel=VELOCITY, acc=ACC)
        mwait()
        
        # 2) 환자 위치(PATIENT_PATH)로 이동
        movel(PATIENT_PATH, vel=VELOCITY, acc=ACC, mod=0)
        mwait()
        
        return PATIENT_PATH

    def stack(self):
        if self.stacked:
            print(f"Box {self.id} is already stacked!")
            return
        if not self.position:
            print("No position set for stacking.")
            return
        
        # 스택 위치로 → 안전 지점 → target_pos → 릴리스 → ready_pos → HOME_READY
        self.__move_to_pos(self.position, action='release')
        self.stacked = True

    def unstack(self):
        if not self.stacked:
            print(f"Box {self.id} is already unstacked!")
            return
        if not self.position:
            print("No position set for stacking.")
            return

        # 1) 스택된 위치로 이동(ready_pos → target_pos) → 그리핑 → ready_pos → HOME_READY
        self.__move_to_pos(self.position, action='grip')
        self.stacked = False

        # 2) 환자 위치로 이동 (그리핑/릴리스 없이 이동만)
        self.__move_to_patient()

        # 3) 환자 위치에서 풀 감지(pull)하기 위한 순응 제어 시작
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        time.sleep(0.5)

        start_time = time.time()
        timeout = 10.0
        pulled = False
        consecutive_hits = 0

        node.get_logger().info("Checking for pull at patient...")

        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            if check_force_condition(DR_AXIS_Z, max=5, ref=DR_TOOL):
                consecutive_hits += 1
            else:
                consecutive_hits = 0

            # 3회 연속 감지되면 pulled=True
            if consecutive_hits >= 3:
                pulled = True
                node.get_logger().info("Pull detected (3 consecutive).")
                break

            if time.time() - start_time > timeout:
                node.get_logger().error("Pull not detected within timeout; aborting.")
                break

        # 4) 순응 제어 해제
        release_compliance_ctrl()

        if not pulled:
            node.get_logger().error("No pull detected; exiting without release.")
            return

        # 5) 풀 감지되었으면 릴리스 → HOME_READY로 복귀
        release()
        movej(HOME_READY, vel=VELOCITY, acc=ACC)
        node.get_logger().info("Box released to patient and returned to HOME_READY.")


def main():
    # 프로그램 시작 시 HOME_READY로 먼저 복귀
    movej(HOME_READY, vel=VELOCITY, acc=ACC)
    mwait()

    box_dict = {}
    while rclpy.ok():
        print("choose target")
        print("==========")
        print("new       : add new box")
        print("delete    : delete box")
        print("grip      : go to grip position and grip something")
        print("release   : go to release position and release something")
        print("returning : patient에게 갔다가 물건 받아서 다시 저장 위치에 놓고 홈으로 복귀")
        print("=====")
        for box in box_dict.values():
            print(box.info())
        user_input = input(">> ").strip()
        if not user_input:
            continue

        # ── “new box_id pos_id” 한 줄로 받은 경우 우선 처리 ──
        parts = user_input.split()
        if parts[0] == "new" and len(parts) == 3:
            box_id = parts[1]
            pos_id = parts[2]
            if box_id not in PREDEFINED_POSITIONS:
                print(f"[ERROR] No predefined position for box ID '{box_id}'")
                continue
            box_dict[box_id] = Box(box_id, pos_id, PREDEFINED_POSITIONS[box_id])
            print(f"[staking] box '{box_id}' is saved with position: {PREDEFINED_POSITIONS[box_id]}")
            continue

        # ── 기존 방식(“new”만 입력하고 다음 줄에 box_id → 다음 줄에 pos_id)도 지원 ──
        if user_input == "new":
            box_id = input("input box id (e.g., 1f, 2f)\n>> ").strip()
            if box_id not in PREDEFINED_POSITIONS:
                print(f"[ERROR] No predefined position for box ID '{box_id}'")
                continue
            position_id = input("input position id\n>> ").strip()
            box_dict[box_id] = Box(box_id, position_id, PREDEFINED_POSITIONS[box_id])
            print(f"[staking] box '{box_id}' is saved with position: {PREDEFINED_POSITIONS[box_id]}")
            continue

        elif user_input == "delete":
            print("input box id")
            box_id = input(">> ")
            print()
            if box_id not in box_dict:
                print("id is not matched : ", box_id)
                continue
            print(f"do you want to delete {box_id}?")
            print("1 : continue\n2 : cancel")
            answer = input(">> ")
            print()
            if answer == "2":
                print("delete is canceled")
                continue
            box_dict.pop(box_id)
            print(f"{box_id} is deleted")

        elif user_input == "grip":
            grip()

        elif user_input == "release":
            release()

        elif user_input == "returning":
            # 1) HOME_POSE_HOME → 환자 위치 이동
            movej(HOME_POSE_HOME, vel=VELOCITY, acc=ACC)
            mwait()
            movel(PATIENT_PATH, vel=VELOCITY, acc=ACC, mod=0)
            mwait()

            # 2) 환자에게 release() → 환자가 물건을 가져가고, 
            #    우리 쪽으로 물건을 줄 때까지 순응 제어로 풀 감지
            release()

            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            time.sleep(0.5)

            start_time = time.time()
            timeout = 10.0
            pulled = False
            consecutive_hits = 0

            node.get_logger().info("Waiting for patient to push (external force) for returning...")

            while True:
                rclpy.spin_once(node, timeout_sec=0.1)
                if check_force_condition(DR_AXIS_Z, max=5, ref=DR_TOOL):
                    consecutive_hits += 1
                else:
                    consecutive_hits = 0

                # 3회 연속 감지되면 pulled=True
                if consecutive_hits >= 3:
                    pulled = True
                    node.get_logger().info("External push detected (3 consecutive).")
                    break

                if time.time() - start_time > timeout:
                    node.get_logger().error("No external push within timeout; aborting returning.")
                    break

            # 3) 순응 제어 해제
            release_compliance_ctrl()

            if not pulled:
                node.get_logger().error("Returning canceled: no external push detected.")
                # 환자가 물건을 주지 않았으므로 바로 HOME_POSE_HOME으로 복귀
                movej(HOME_POSE_HOME, vel=VELOCITY, acc=ACC)
                continue

            # 4) 환자가 물건을 주었으므로 그리핑
            grip()

            # 5) box_dict 안에서 stacked==False인 박스(가장 최근 언스택된 것) 선택
            unstacked_boxes = [b for b in box_dict.values() if b.stacked == False]
            if not unstacked_boxes:
                node.get_logger().error("No unstacked box found to return the item to.")
                # 물건을 들고 홈으로 돌아가긴 하지만, 놓을 위치가 없으므로 그냥 HOME_POSE_HOME으로 복귀
                movej(HOME_POSE_HOME, vel=VELOCITY, acc=ACC)
                continue

            # 만약 여러 개가 있으면, 가장 마지막에 언스택된 박스를 사용하거나
            # 단순히 첫 번째를 사용하도록 설계할 수 있음
            box_to_return = unstacked_boxes[0]

            # 6) box_to_return.position으로 이동하여 release() → 재적재
            #    __move_to_pos()는 내부적으로 HOME_POS_MAP[box.id] → ready_pos → target_pos → release → ready_pos → HOME_READY 순서로 동작
            box_to_return._Box__move_to_pos(box_to_return.position, action='release')
            box_to_return.stacked = True

            # 7) 마지막으로 HOME_POSE_HOME으로 복귀
            movej(HOME_POSE_HOME, vel=VELOCITY, acc=ACC)
            node.get_logger().info(f"Returning completed: Box {box_to_return.id} has been restacked.")
        

        # ── “box_id” 하나만 입력되면 stack/unstack 메뉴로 들어가는 경우 ──
        if user_input in box_dict:
            target = box_dict[user_input]
            subcmd = input("1 : stack\n2 : unstack\n>> ").strip()
            if subcmd == "1":
                target.stack()
            elif subcmd == "2":
                target.unstack()
            else:
                print("----- invalid option -----")
            continue

        print("----- invalid command -----")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
