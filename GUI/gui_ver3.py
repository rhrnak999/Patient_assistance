import tkinter as tk
from tkinter import messagebox, ttk
import subprocess
from PIL import Image, ImageTk, ImageDraw, ImageFont  # Pillow 라이브러리 필요
import threading
import os
import numpy as np  # numpy 추가
import time
import sys

#------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------

# 전역 상태 변수
route_generation_done = False
move_button = None  # 경로이동 버튼 참조용

# 박스 데이터 저장용 딕셔너리 (물건 전달 GUI에서 사용)
#   key: box_id (string), value: dict { 'pos_id': string, 'coords': [x,y], 'stacked': bool }
box_dict = {}
selected_box_id = None

# 이 변수들이 호출되는 시점에 유효하도록 전역에 선언만 해 둡니다.
staking_proc = None    # ros2 run rokey staking 프로세스 (stdin 파이프 확보용)
staking_proc_lock = threading.Lock()  # 멀티스레드에서 stdin write 시 동기화

#------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------

PREDEFINED_POSITIONS = {
    "1f": [644.69, -432.91, 316.32, 86.79, -92.00, -94.05],
    "2f": [605.97, -415.97, 507.91, 92.01, -88.80, 93.95],
    # 필요하면 여기에 더 늘려줍니다
}

#------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------

def start_staking_process():
    """
    ‘물건 전달’ GUI가 처음 뜰 때 딱 한 번 호출하여,
    ros2 run rokey staking 노드를 백그라운드 프로세스로 실행하고 stdin/stdout 파이프를 연결합니다.
    이후 GUI 버튼 클릭 시 staking_proc.stdin.write(...) 로 명령을 보낼 수 있습니다.
    """
    global staking_proc
    if staking_proc is not None:
        return  # 이미 실행 중이면 재실행하지 않음

    try:
        # ROS2 환경이 이미 소싱된 상태라면 아래처럼 간단히 호출해도 되지만,
        # 일반적으로는 bash -c "source ... && ros2 run rokey staking" 으로 실행해야 합니다.
        staking_proc = subprocess.Popen(
            ["bash", "-c", "source ~/ros2_ws/install/setup.bash && ros2 run rokey s3"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
    except Exception as e:
        messagebox.showerror("staking 실행 오류", f"staking 노드 실행 중 오류 발생:\n{e}")
        staking_proc = None


def send_to_staking(line: str):
    """
    staking_proc에 문자열(line)을 쓰고 flush() 합니다. 
    멀티스레드 환경에서 동시 쓰기를 대비해 mutex로 감싸두었습니다.
    """
    global staking_proc
    if staking_proc is None:
        # 프로세스가 실행되지 않은 상태라면 새로 실행
        start_staking_process()

    if staking_proc is not None and staking_proc.stdin:
        with staking_proc_lock:
            try:
                staking_proc.stdin.write(line + "\n")
                staking_proc.stdin.flush()
            except Exception as e:
                messagebox.showerror("stdin 쓰기 오류", f"staking stdin에 쓰는 중 오류 발생:\n{e}")

#------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------

# 식수 제공 기능
def run_water_script():
    try:
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "source ~/ros2_ws/install/setup.bash && ros2 run rokey cup"])
        print("실행 성공", "//", "식수 제공 스크립트를 실행했습니다.")
    except Exception as e:
        messagebox.showerror("실행 오류", f"스크립트 실행 중 오류 발생:\n{e}")

# 환자 자세 교정 기능
def patient_move_script():
    try:
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "source ~/ros2_ws/install/setup.bash && ros2 run rokey mp"])
        print("실행 성공", "//", "환자 자세 교정 스크립트를 실행했습니다.")
    except Exception as e:
        messagebox.showerror("실행 오류", f"스크립트 실행 중 오류 발생:\n{e}")

# 청결 보조
def clean_patient():
    def show_error_with_buttons():
        popup = tk.Toplevel()
        popup.title("환자 청결보조")
        popup.geometry("300x200")
        tk.Label(popup, text="어떤 작업을 하시겠습니까?").pack(pady=10)

        gen_btn = tk.Button(popup, text="닦기 경로 생성", command=lambda: teach_move(gen_btn, popup))
        gen_btn.pack(pady=5)

        global move_button
        move_button = tk.Button(popup, text="경로이동", state=tk.DISABLED, command=do_route_move)
        move_button.pack(pady=5)

        popup.update_idletasks()
        w = popup.winfo_width()
        h = popup.winfo_height()
        sw = popup.winfo_screenwidth()
        sh = popup.winfo_screenheight()
        x = (sw // 2) - (w // 2)
        y = (sh // 2) - (h // 2)
        popup.geometry(f"{w}x{h}+{x}+{y}")
    show_error_with_buttons()

def teach_move(button, popup):
    def run_teach():
        proc = subprocess.Popen(
            ["gnome-terminal", "--", "bash", "-c", "source ~/ros2_ws/install/setup.bash && ros2 run rokey tc2"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        proc.wait()
        if move_button:
            move_button.config(state=tk.NORMAL)
    button.config(state=tk.DISABLED)
    threading.Thread(target=run_teach).start()

def show_path_image():
    try:
        subprocess.Popen(["gnome-terminal", "--", "bash", "-c", "source ~/ros2_ws/install/setup.bash && ros2 run rokey open_fin"])
        print("실행 성공", "//", "환자 자세 교정 스크립트를 실행했습니다.")
    except Exception as e:
        messagebox.showerror("실행 오류", f"스크립트 실행 중 오류 발생:\n{e}")
def do_route_move():
    def run_play_back():
        try:
            proc = subprocess.Popen(
                ["gnome-terminal", "--", "bash", "-c", "source ~/ros2_ws/install/setup.bash && ros2 run rokey pb31"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            proc.wait()
            show_path_image()
        except Exception as e:
            messagebox.showerror("실행 오류", f"경로 이동 실행 중 오류 발생:\n{e}")
    threading.Thread(target=run_play_back).start()

#------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------

# 물건 전달 기능용 GUI
def move_obj_script():
    """
    ‘물건 전달’ 버튼을 누르면 Toplevel 창을 띄우고,
    동시에 처음 한 번만 ros2 run rokey staking 프로세스를 시작합니다.
    이후 Toplevel 내에서 ‘등록’, ‘삭제’, ‘Stack’, ‘Unstack’, ‘Grip’, ‘Release’, ‘Returning’ 버튼을 누를 때
    staking 노드의 stdin으로 문자열을 흘려보냅니다.
    """
    # staking_proc을 한 번만 실행하도록 보장
    start_staking_process()

    def update_box_list():
        # Treeview에 현재 박스 정보를 모두 지우고 다시 삽입
        for item in tree.get_children():
            tree.delete(item)
        for box_id, info in box_dict.items():
            tree.insert("", "end", iid=box_id, values=(
                box_id, info['pos_id'], " ".join(map(str, info['coords'])), '✔' if info['stacked'] else '✘'
            ))

    def new_box():
        box_id = entry_box_id.get().strip()
        pos_id = entry_pos_id.get().strip()
        if not box_id or not pos_id:
            messagebox.showwarning("입력 오류", "Box ID와 Position ID를 모두 입력해주세요.")
            return

        # (1) PREDEFINED_POSITIONS에 box_id가 존재하는지 체크
        if box_id not in PREDEFINED_POSITIONS:
            messagebox.showerror("오류", f"Box ID '{box_id}'에 대한 좌표가 정의돼 있지 않습니다.")
            return

        if box_id in box_dict:
            messagebox.showwarning("중복 오류", f"Box ID '{box_id}'가 이미 등록되어 있습니다.")
            return

        # (2) 실제 좌표를 꺼내 와서 coords에 저장
        coords = PREDEFINED_POSITIONS[box_id]

        box_dict[box_id] = {
            'pos_id': pos_id,
            'coords': coords.copy(),   # 또는 coords[:] 로 복사
            'stacked': False
        }
        update_box_list()
        status_label.config(text=f"[GUI] Box '{box_id}' 등록 완료 (PosID: {pos_id}, Coords: {coords})")

        # (3) staking 코드에도 “new box_id pos_id” 한 줄로 전달
        send_to_staking(f"new {box_id} {pos_id}")

        entry_box_id.delete(0, tk.END)
        entry_pos_id.delete(0, tk.END)


    def delete_box():
        """
        1) Treeview에서 선택 박스 삭제 → box_dict에서 제거 → Treeview 갱신
        2) staking 노드 stdin에 “delete” → box_id 순으로 전송
        """
        global selected_box_id
        if not selected_box_id or selected_box_id not in box_dict:
            return

        bid = selected_box_id
        del box_dict[bid]
        update_box_list()
        status_label.config(text=f"[GUI] Box '{bid}' 삭제 완료")

        send_to_staking("delete")
        send_to_staking(bid)

        # 선택 해제 및 버튼 상태 갱신
        tree.selection_remove(tree.selection())
        on_select_inner(None)

    def on_select_inner(event):
        nonlocal_selected = tree.selection()
        global selected_box_id
        selected_box_id = nonlocal_selected[0] if nonlocal_selected else None
        update_button_states()

    def update_button_states():
        """
        선택된 박스가 있을 때만 Stack/Unstack/Delete 버튼을 활성화
        """
        if selected_box_id and selected_box_id in box_dict:
            box = box_dict[selected_box_id]
            btn_stack.config(state='normal' if not box['stacked'] else 'disabled')
            btn_unstack.config(state='normal' if box['stacked'] else 'disabled')
            btn_delete_box.config(state='normal')
        else:
            btn_stack.config(state='disabled')
            btn_unstack.config(state='disabled')
            btn_delete_box.config(state='disabled')

    def grip_btn_cb():
        """
        Grip 버튼 클릭 시 staking 노드에 “grip” 전송
        """
        send_to_staking("grip")
        status_label.config(text="[GUI] staking → grip 전송 완료")

    def release_btn_cb():
        """
        Release 버튼 클릭 시 staking 노드에 “release” 전송
        """
        send_to_staking("release")
        status_label.config(text="[GUI] staking → release 전송 완료")

    def do_stack():
        """
        Stack 버튼 클릭 시:
          1) GUI 내부 box_dict 에서 stacked 상태 변경
          2) Treeview 갱신 및 버튼 상태 갱신
          3) staking 노드로 “box_id → 1” 을 보내서 실제 로봇 stack 동작 수행
        """
        if not selected_box_id:
            return
        # ① GUI 내부에서 stacked 상태를 True 로 설정
        box_dict[selected_box_id]['stacked'] = True
        update_box_list()
        update_button_states()
        status_label.config(text=f"[GUI] Box '{selected_box_id}' 상태를 stacked=True 로 변경")
        # ② staking 노드에 실제로 stack 명령을 보냄 (1 = stack)
        send_to_staking(selected_box_id)
        send_to_staking("1")
        status_label.config(text=f"[GUI] staking → Stack('{selected_box_id}') 전송 완료")

    def do_unstack():
        """
        Unstack 버튼 클릭 시:
          1) GUI 내부 box_dict 에서 stacked 상태 변경
          2) Treeview 갱신 및 버튼 상태 갱신
          3) staking 노드로 “box_id → 2” 을 보내서 실제 로봇 unstack 동작 수행
        """
        if not selected_box_id:
            return

      # ① GUI 내부에서 stacked 상태를 False 로 설정
        box_dict[selected_box_id]['stacked'] = False
        update_box_list()
        update_button_states()
        status_label.config(text=f"[GUI] Box '{selected_box_id}' 상태를 stacked=False 로 변경")

        # ② staking 노드에 실제로 unstack 명령을 보냄 (2 = unstack)
        send_to_staking(selected_box_id)
        send_to_staking("2")
        status_label.config(text=f"[GUI] staking → Unstack('{selected_box_id}') 전송 완료")


    def do_returning():
        """
        Returning 버튼 클릭 시:
          staking 노드에 “returning” 전송
        """
        send_to_staking("returning")
        status_label.config(text="[GUI] staking → returning 전송 완료")

    # --- 여기서부터 GUI 레이아웃 생성 ---

    win = tk.Toplevel()
    win.title("로키 스태킹 제어 화면")
    win.geometry("900x500")

    # 좌측 - 리스트 & 상태 표시 영역 (Treeview)
    frame_left = tk.Frame(win)
    frame_left.pack(side="left", fill="both", expand=True, padx=10, pady=10)

    tree = ttk.Treeview(frame_left, columns=("box_id", "pos_id", "coords", "stacked"), show="headings")
    tree.heading("box_id", text="Box ID")
    tree.heading("pos_id", text="Pos ID")
    tree.heading("coords", text="좌표(간략)")
    tree.heading("stacked", text="Stacked")
    tree.pack(fill="both", expand=True)
    tree.bind("<<TreeviewSelect>>", on_select_inner)

    # 우측 - 작업 버튼 & 입력 폼 영역
    frame_right = tk.Frame(win)
    frame_right.pack(side="right", fill="y", padx=10, pady=10)

    # (A) 새 박스 등록 섹션
    tk.Label(frame_right, text="새 박스 ID:").pack(anchor="w")
    entry_box_id = tk.Entry(frame_right)
    entry_box_id.pack(fill="x")
    tk.Label(frame_right, text="Position ID:").pack(anchor="w", pady=(10,0))
    entry_pos_id = tk.Entry(frame_right)
    entry_pos_id.pack(fill="x")
    tk.Button(frame_right, text="등록하기", command=new_box).pack(pady=10, fill="x")

    # (B) 박스 삭제 버튼
    btn_delete_box = tk.Button(frame_right, text="삭제하기", command=delete_box, state='disabled')
    btn_delete_box.pack(pady=5, fill="x")

    # (C) 그리퍼 직접 제어 섹션
    ttk.Separator(frame_right, orient="horizontal").pack(fill="x", pady=15)
    tk.Label(frame_right, text="그리퍼 제어").pack(pady=(0,5))
    tk.Button(frame_right, text="Grip", command=grip_btn_cb).pack(fill="x", pady=2)
    tk.Button(frame_right, text="Release", command=release_btn_cb).pack(fill="x", pady=2)

    # (D) 스택/언스택 섹션
    ttk.Separator(frame_right, orient="horizontal").pack(fill="x", pady=15)
    tk.Label(frame_right, text="스택 제어").pack(pady=(0,5))
    btn_stack = tk.Button(frame_right, text="Stack", command=do_stack, state='disabled')
    btn_stack.pack(fill="x", pady=2)
    btn_unstack = tk.Button(frame_right, text="Unstack", command=do_unstack, state='disabled')
    btn_unstack.pack(fill="x", pady=2)

    # (E) Returning 버튼
    ttk.Separator(frame_right, orient="horizontal").pack(fill="x", pady=15)
    tk.Label(frame_right, text="Returning").pack(pady=(0,5))
    tk.Button(frame_right, text="Returning", command=do_returning).pack(fill="x", pady=2)

    # (F) 상태 메시지 출력창 (전역)
    status_label = tk.Label(win, text="상태 메시지 출력 영역", bd=1, relief="sunken", anchor="w")
    status_label.pack(side="bottom", fill="x")

    # 초기 리스트 갱신
    update_box_list()

#------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------

# 버튼 클릭 시 실행될 함수
def on_button_click(button_name):
    messagebox.showinfo("버튼 클릭", f"{button_name} 버튼이 눌렸습니다.")
    if button_name == "물 옮기기":
        run_water_script()
    elif button_name == "물건 전달":
        move_obj_script()      # 여기를 호출하면 move_obj_script()가 열린다
    elif button_name == "몸세척 보조":
        clean_patient()
    elif button_name == "자세 교정":
        patient_move_script()

# 메인 윈도우 설정
root = tk.Tk()
root.title("요양 보조프로젝트")

# 화면 중앙에 윈도우 배치
window_width = 400
window_height = 450
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
x = (screen_width // 2) - (window_width // 2)
y = (screen_height // 2) - (window_height // 2)
root.geometry(f"{window_width}x{window_height}+{x}+{y}")

# 배경 이미지 불러오기 (이미지 경로 수정)
background_image_path = "/home/rokey/Downloads/images.jpeg"
if os.path.exists(background_image_path):
    bg_image = Image.open(background_image_path)
    bg_image = bg_image.resize((400, 450))
    bg_photo = ImageTk.PhotoImage(bg_image)
    background_label = tk.Label(root, image=bg_photo)
    background_label.place(x=0, y=0, relwidth=1, relheight=1)

# 라벨 (제목)
title_label = tk.Label(root, text="요양 보조프로젝트", font=("Arial", 18))
title_label.pack(pady=20)

# 버튼 4개 생성
button_texts = ["물 옮기기", "물건 전달", "몸세척 보조", "자세 교정"]
for text in button_texts:
    btn = tk.Button(root, text=text, width=20, height=2, command=lambda t=text: on_button_click(t))
    btn.pack(pady=5)

# GUI 실행
root.mainloop()

# GUI 종료 시 staking 프로세스도 정리
if staking_proc:
    try:
        staking_proc.stdin.close()
        staking_proc.terminate()
        staking_proc.wait(timeout=2)
    except Exception:
        pass
