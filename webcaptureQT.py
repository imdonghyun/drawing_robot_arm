import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton,QVBoxLayout,QHBoxLayout,QWidget,QDialog, QCheckBox
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QFont
from PyQt5.QtCore import Qt, QPoint, QTimer, QPropertyAnimation, QEasingCurve
import cv2, time
import numpy as np
import backimg_cut
import stylization
import line_makingFn
import os
import socket
import robot_functions
import json
import math
import datetime

save_image_path = "captured_image.jpg"
output_file = "captured_image_rembg"
style = ['style_weights/celeba_distill.pt',
        'style_weights/face_paint_512_v1.pt',
        'style_weights/face_paint_512_v2.pt']

server_address = "127.0.0.1"  # 서버의 실제 IP 주소 또는 도메인 이름
server_port = 12345 

with open('calibration_config.json') as json_file:
   parameter = json.load(json_file)

pen_height = parameter["pen_height"]
center_x = parameter["center_x"]
center_y = parameter["center_y"]
width = parameter["width"]
height = parameter["height"]
origin_x = parameter["origin_x"]
origin_y = parameter["origin_y"]

width_pixel = parameter["width_pixel"]
height_pixel = parameter["height_pixel"]


class calibWindow(QWidget):
    def __init__(self, big_button_text='0/4\n다음 포인트로 >>', parent=None):
        super().__init__(parent)
        self.setWindowTitle('Button Layout')
        self.setGeometry(100, 100, 640, 480)
        self.pen_height_list = [0,0,0,0]
        self.init_point = 0 # 네 모서리 중 몇 번째인지 나타내는 값.
        main_layout = QHBoxLayout(self)
        layout = QVBoxLayout(self)
        left_layout = QVBoxLayout()
        # 작은 버튼 4개 추가
        
        button = QPushButton(f'위로 크게 이동')
        button.setFixedSize(640 // 4, 480 // 6)
        button.clicked.connect(self.up_wide)
        left_layout.addWidget(button)
        button = QPushButton(f'위로 작게 이동')
        button.setFixedSize(640 // 4, 480 // 6)
        button.clicked.connect(self.up_narrow)
        left_layout.addWidget(button)
        button = QPushButton(f'아래로 작게 이동')
        button.setFixedSize(640 // 4, 480 // 6)
        button.clicked.connect(self.down_narrow)
        left_layout.addWidget(button)
        button = QPushButton(f'아래로 크게 이동')
        button.setFixedSize(640 // 4, 480 // 6)
        button.clicked.connect(self.down_wide)
        left_layout.addWidget(button)

        main_layout.addLayout(left_layout)

        # 큰 버튼 추가
        right_layout = QVBoxLayout()

        self.big_button = QPushButton(f'{self.init_point}/4\n다음 포인트로 >>')
        self.big_button.setFixedSize(640 // 5, 480 // 5)
        self.big_button.clicked.connect(self.go_next_point)
        # right_layout.addStretch(1)
        right_layout.addWidget(self.big_button)

        save_button = QPushButton(f'해당 위치 저장')
        save_button.setFixedSize(640 // 5, 480 // 5)
        save_button.clicked.connect(self.save_point)
        # right_layout.addStretch(1)
        right_layout.addWidget(save_button)

        # right_layout.addStretch(1)
        main_layout.addLayout(right_layout)

        self.setLayout(main_layout)

    def save_point(self):
        if self.init_point >= 1:
            print('save_point')
            
            self.pen_height_list[self.init_point-1] = self.p_goal[2]
            if self.init_point == 1:
                self.p_goal = [center_x-width/2, center_y+height/2, pen_height, 0,180,0]
            elif self.init_point == 2:
                self.p_goal = [center_x+width/2, center_y+height/2, pen_height, 0,180,0]
            elif self.init_point == 3:
                self.p_goal = [center_x+width/2, center_y-height/2, pen_height, 0,180,0]
            elif self.init_point == 4:
                self.p_goal = [center_x-width/2, center_y-height/2, pen_height, 0,180,0]
            robot_functions.indy.movetelel_abs(self.p_goal, vel_ratio=0.05, acc_ratio=1)
            time.sleep(2)
        pass

    def go_next_point(self):
        print('next_point')
        self.init_point +=1
        if self.init_point == 1:
            self.big_button.setText(f'{self.init_point}/4\n다음 포인트로 >>')
            # 첫 번째 포인트로 접근.
            robot_functions.indy.start_teleop(0)
            time.sleep(2)
            self.p_goal = [center_x, center_y, pen_height, 0,180,0]

            for i in range(200):
                self.p_goal[0] -= width/2/200
                self.p_goal[1] += height/2/200
                robot_functions.indy.movetelel_abs(self.p_goal, vel_ratio=0.1, acc_ratio=1)
                time.sleep(0.01)
            while True:
                p_now = robot_functions.indy.get_control_data()["p"]
                if (max(np.array(self.p_goal)-np.array(p_now))<2):
                    break
                time.sleep(0.01)
            robot_functions.wait_indy()
            time.sleep(2)


        elif self.init_point == 2:
            self.big_button.setText(f'{self.init_point}/4\n다음 포인트로 >>')
            # 두 번째 포인트로 접근
            self.p_goal = [center_x-width/2, center_y+height/2, pen_height, 0,180,0]
            for i in range(200):
                self.p_goal[0] += width/200
                robot_functions.indy.movetelel_abs(self.p_goal, vel_ratio=0.1, acc_ratio=1)
                time.sleep(0.01)
            while True:
                p_now = robot_functions.indy.get_control_data()["p"]
                if (max(np.array(self.p_goal)-np.array(p_now))<2):
                    break
                time.sleep(0.01)
            robot_functions.wait_indy()
            time.sleep(2)

            pass
        elif self.init_point == 3:  
            self.big_button.setText(f'{self.init_point}/4\n다음 포인트로 >>')
            self.p_goal = [center_x+width/2, center_y+height/2, pen_height, 0,180,0]
            for i in range(200):
                self.p_goal[1] -= height/200
                robot_functions.indy.movetelel_abs(self.p_goal, vel_ratio=0.2, acc_ratio=1)
                time.sleep(0.02)
            while True:
                p_now = robot_functions.indy.get_control_data()["p"]
                if (max(np.array(self.p_goal)-np.array(p_now))<2):
                    break
                time.sleep(0.01)
            robot_functions.wait_indy()
            time.sleep(2)

            pass
        elif self.init_point == 4:
            self.big_button.setText(f'칼리브레이션 완료')
            self.p_goal = [center_x+width/2, center_y-height/2, pen_height, 0,180,0]
            for i in range(200):
                self.p_goal[0] -= width/200
                robot_functions.indy.movetelel_abs(self.p_goal, vel_ratio=0.1, acc_ratio=1)
                time.sleep(0.01)
            while True:
                p_now = robot_functions.indy.get_control_data()["p"]
                if (max(np.array(self.p_goal)-np.array(p_now))<2):
                    break
                time.sleep(0.01)
            robot_functions.wait_indy()
            time.sleep(2)
            pass
        elif self.init_point == 5:
            self.init_point = 0
            self.big_button.setText(f'{self.init_point}/4\n다음 포인트로 >>')
            parameter["pen_height_1"] = self.pen_height_list[0]
            parameter["pen_height_2"] = self.pen_height_list[1]
            parameter["pen_height_3"] = self.pen_height_list[2]
            parameter["pen_height_4"] = self.pen_height_list[3]
            with open('calibration_config.json', 'w') as make_file:
                json.dump(dict(parameter), make_file)
            self.p_goal = [center_x-width/2, center_y-height/2, pen_height, 0,180,0]
            for i in range(200):
                self.p_goal[0] += width/2/200
                self.p_goal[1] += height/2/200
                robot_functions.indy.movetelel_abs(self.p_goal, vel_ratio=0.1, acc_ratio=1)
                time.sleep(0.01) 
            time.sleep(3) 
            robot_functions.indy.stop_teleop()
            time.sleep(0.5)
            robot_functions.move_to_base()
            self.end_calib()
            #TODO 윈도우 닫고, 이동하기
        
        pass

    def up_wide(self):
        print('up_wide')
        self.p_goal[2] += 5
        robot_functions.indy.movetelel_abs(self.p_goal, vel_ratio=0.05, acc_ratio=1)
        pass

    def up_narrow(self):
        print('up_narrow')
        self.p_goal[2] += 1
        robot_functions.indy.movetelel_abs(self.p_goal, vel_ratio=0.05, acc_ratio=1)
        pass

    def down_wide(self):
        print('down_wide')
        self.p_goal[2] -= 5
        robot_functions.indy.movetelel_abs(self.p_goal, vel_ratio=0.05, acc_ratio=1)
        pass

    def down_narrow(self):
        print('down_narrow')
        self.p_goal[2] -= 1
        robot_functions.indy.movetelel_abs(self.p_goal, vel_ratio=0.05, acc_ratio=1)
        pass

    def end_calib(self):
        self.close()

class WebcamWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("Webcam Viewer")
        self.setGeometry(100, 100, 660, 580)
        self.following_face = 0
        self.label = QLabel(self)
        self.label.setGeometry(10, 10, 640, 480)

        self.start_button = QPushButton("Start", self)
        self.start_button.setGeometry(10, 500, 100, 30)
        self.start_button.clicked.connect(self.start_webcam)

        self.home_button = QPushButton("Home", self)
        self.home_button.setGeometry(10, 535, 100, 30)
        self.home_button.clicked.connect(self.robot_move_to_home)

        self.home_button = QPushButton("Photo", self)
        self.home_button.setGeometry(120, 535, 100, 30)
        self.home_button.clicked.connect(self.robot_move_to_photo)

        self.home_button = QPushButton("Draw_base", self)
        self.home_button.setGeometry(230, 535, 100, 30)
        self.home_button.clicked.connect(self.robot_move_to_draw)

        self.calib_button = QPushButton("Calibration", self)
        self.calib_button.setGeometry(340, 535, 100, 30)
        self.calib_button.clicked.connect(self.robot_calibration)

        self.capture_button = QPushButton("Capture", self)
        self.capture_button.setGeometry(120, 500, 100, 30)
        self.capture_button.clicked.connect(self.capture_image)

        self.stop_button = QPushButton("Stop", self)
        self.stop_button.setGeometry(230, 500, 100, 30)
        self.stop_button.clicked.connect(self.stop_webcam)

        self.stop_button = QPushButton("Close", self)
        self.stop_button.setGeometry(550, 500, 100, 30)
        self.stop_button.clicked.connect(self.close_view)

        self.checkbox = QCheckBox('얼굴 따라가기', self)
        self.checkbox.setGeometry(400, 500, 100, 30)
        self.checkbox.stateChanged.connect(self.checkbox_following_face_onoff)

        self.timer = QTimer(self)
        self.timer.start(int(1000/30))
        self.webcam = cv2.VideoCapture(1)
        self.webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.timer.timeout.connect(self.update_frame)

        self.calib_win = calibWindow()
        # 얼굴 따라가기
        self.cascade_filename = 'haarcascade_frontalface_alt.xml'
        self.cascade = cv2.CascadeClassifier(self.cascade_filename)
        #얼굴 인식
        # self.recognizer = cv2.face.LBPHFaceRecognizer_create()
        # self.recognizer.read('trainer/trainer.yml')
        # self.names = ['None', 'kkh']
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        # Vision detection parameters #
        robot_functions.indy.stop_teleop()
        self.detect_radius = 160
        self.xc = 320
        self.yc = 160
        self.dx = 0
        self.dy = 0
        self.p_goal = [-0.2*1000, 0.3*1000, 0.5*1000, 0,180,105] 
        ###
        
    def robot_calibration(self):
        #TODO calibration window 새로 만들기
        robot_functions.indy.stop_teleop()
        self.stop_webcam()
        self.robot_move_to_home()
        self.robot_move_to_draw()
        self.calib_win.init_point = 0
        self.calib_win.show()
        print('show end')
        pass

    def read_integers_from_notepad(self, file_path):
            integers = []
            with open(file_path, 'r') as file:
                for line in file:
                    # 스페이스를 기준으로 문자열을 분리하여 정수로 변환한 후 리스트에 추가
                    integers.append(list(map(int, line.strip().split())))
            return integers
    
    def robot_drawing(self):
        # self.lined_img.close()
        
        robot_functions.move_to_draw_base()
        time.sleep(0.5)
        robot_functions.indy.start_teleop(0)
        time.sleep(2)

        start_T = time.time()

        # 메모장 파일 경로
        notepad_file_path = self.final_image_path[:-9] + "line.txt"
        print(notepad_file_path)
        # # 한 줄씩 읽어오기
        lines = self.read_integers_from_notepad(notepad_file_path)
        # print(lines)
        # lines = [[320, 168, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 7, 7, 7, 7, 6, 6, 5, 5, 5, 5, 7, 7, 7, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 3, 3, 4, 4]]
        # lines = [[320, 168, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 4, 5, 5, 4, 5, 5, 4, 5, 5, 4, 5, 5, 4, 5, 5, 4, 5, 5, 4, 5, 5, 4, 5, 5, 4, 5, 5, 4, 5, 5, 4, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 7, 7, 7, 7, 6, 6, 5, 5, 5, 5, 7, 7, 7, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1]]
        lines.append([0, 190, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7,7,6,6, 5, 5, 4, 5, 5, 4, 5, 5, 4, 5, 5, 4, 5, 5, 4, 5, 5, 4, 5, 5, 4, 5, 5, 4, 4,3,3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 7, 7, 7, 7, 6, 6, 5, 5, 5, 5, 7, 7, 7, 7, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 1, 1])
        now = 0
        total = len(lines)
        x_pixel_pre = width_pixel/2
        y_pixel_pre = height_pixel/2
        x_coord_pre = origin_x + x_pixel_pre/width_pixel*width
        y_coord_pre = origin_y - y_pixel_pre/height_pixel*height
        h_coord_pre = pen_height
        sleep_rate = 0.03
        # sleep_rate = 0.02
        
        for index, line in enumerate(lines):
            if index == total-1:
                sleep_rate = 0.07
            else:
                sleep_rate = 0.03
            # dir_list = lines[i]
            x_pixel = int(line[1])
            y_pixel = int(line[0])
            dir_list = line[2:]
        #     dir_list = ["4"]*400 + ["2"]*400 + ["7"]*400

            x_coord = origin_x + x_pixel/width_pixel*width
            y_coord = origin_y - y_pixel/height_pixel*height
            h_coord = robot_functions.height_interpolation(x_pixel, y_pixel) + 7
            
            dt = math.sqrt((x_pixel-x_pixel_pre)**2 + (y_pixel-y_pixel_pre)**2)*0.01
            
            for k in range(int(dt*100)):
                s = robot_functions.polynomial_time_scaling(k, int(dt*100))
                robot_functions.indy.movetelel_abs([x_coord_pre + (x_coord - x_coord_pre)*s, y_coord_pre + (y_coord - y_coord_pre)*s, h_coord_pre + 7 + (h_coord - h_coord_pre)*s, 0,180,0], vel_ratio=0.1, acc_ratio=1)
                time.sleep(0.01) # 다음으로 넘어가는 딜레이
            
            #print(dt)
            time.sleep(0.25)
            h_coord = robot_functions.height_interpolation(x_pixel, y_pixel)

            for i in range(7):
                robot_functions.indy.movetelel_abs([x_coord, y_coord, h_coord + 6 - i, 0,180,0], vel_ratio=0.03, acc_ratio=1)
                time.sleep(0.05)

            for i in range(len(dir_list)):
                if dir_list[i] == 2:
                    x_pixel += 1
                elif dir_list[i] == 3:
                    y_pixel += 1
                    x_pixel += 1
                elif dir_list[i] == 4:
                    y_pixel += 1
                elif dir_list[i] == 5:
                    y_pixel += 1
                    x_pixel -= 1
                elif dir_list[i] == 6:
                    x_pixel -= 1
                elif dir_list[i] == 7:
                    y_pixel -= 1
                    x_pixel -= 1
                elif dir_list[i] == 0:
                    y_pixel -= 1
                elif dir_list[i] == 1:
                    y_pixel -= 1
                    x_pixel += 1

                x_coord = origin_x + x_pixel/width_pixel*width
                y_coord = origin_y - y_pixel/height_pixel*height
                h_coord = robot_functions.height_interpolation(x_pixel, y_pixel)
                robot_functions.indy.movetelel_abs([x_coord, y_coord, h_coord, 0,180,0], vel_ratio=0.1, acc_ratio=1)
                time.sleep(sleep_rate) # 한 칸 한 칸 딜레이

            h_coord = robot_functions.height_interpolation(x_pixel, y_pixel)
            for i in range(7):
                robot_functions.indy.movetelel_abs([x_coord, y_coord, h_coord + i + 1, 0,180,0], vel_ratio=0.03, acc_ratio=1)
                time.sleep(0.05)
            time.sleep(0.1)
            now += 1
            print("now = ", now, "/",total)
            x_pixel_pre = x_pixel
            y_pixel_pre = y_pixel
            
            x_coord_pre = x_coord
            y_coord_pre = y_coord
            h_coord_pre = h_coord
        robot_functions.indy.stop_teleop()

        time.sleep(2)

        robot_functions.move_to_base()

        print("run time : ", time.time() - start_T)
        pass

    def checkbox_following_face_onoff(self, state):
        if state == Qt.Checked:
            robot_functions.indy.start_teleop(0)
            self.following_face = 1
            
            print('얼굴 따라가기 모드 켜기')
        else:
            
            self.following_face = 0
            robot_functions.indy.stop_teleop()
            print('얼굴 따라가기 모드 끄기')

    def robot_move_to_home(self):
        robot_functions.move_to_base()
    
    def robot_move_to_photo(self):
        robot_functions.move_to_photo_base()

    def robot_move_to_draw(self):
        robot_functions.move_to_draw_base()
    
    def robot_move_to_home_soc(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((server_address, server_port))
        request = 'home'
        client_socket.send(request.encode("utf-8"))
        response = client_socket.recv(1024).decode("utf-8")
        print(response)
    def robot_move_to_photo_soc(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((server_address, server_port))
        request = 'photo'
        client_socket.send(request.encode("utf-8"))
        response = client_socket.recv(1024).decode("utf-8")
        print(response)
    def robot_move_to_draw_soc(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((server_address, server_port))
        request = 'draw_base'
        client_socket.send(request.encode("utf-8"))
        response = client_socket.recv(1024).decode("utf-8")
        print(response)

    def close_view(self):
        self.close()

    def start_webcam(self):
        # self.show_lined_img("captured_image_rembg.jpg")
        self.timer.start(int(1000/30))  # 30 fps

    def stop_webcam(self):
        self.timer.stop()

    def update_frame(self):
        ret, frame = self.webcam.read()
        xd = 160
        yd = 120
        
        if ret:
            frame = frame[360-yd:360+yd, 640-xd:640+xd]
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_CUBIC)
            self.frame = frame
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # frame = cv2.resize(frame,dsize=None,fx=0.75,fy=0.75) 
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = self.cascade.detectMultiScale(gray,scaleFactor=1.1,minNeighbors=5,minSize=(30,30))
            if len(results)>0:
                x, y, w, h = results[0]
                self.xf = int(x+w/2)
                self.yf = int(y+h/3)
                
                vec = np.array([self.xf-self.xc, self.yf-self.yc])
 
                if self.following_face == 1:
                    if  np.linalg.norm(vec) < self.detect_radius:
                        
                        err_x = vec[0]/40
                        err_y = vec[1]/40

                        self.p_goal[1] += err_x
                        self.p_goal[2] -= err_y
                        if self.following_face == 1:
                            robot_functions.indy.movetelel_abs(self.p_goal, vel_ratio=0.05, acc_ratio=1)
                    # print("following face on !")
                else:
                    pass

                if (np.linalg.norm(vec)<5): # 범위안에 들어오면 사진 찍기
                    print("cheese")
                    # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    robot_functions.indy.stop_teleop()
                    self.capture_image()
                    time.sleep(1)

                    # cv2.imwrite('cheese.jpg', frame)
                
            #     cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
            #     id, confidence = self.recognizer.predict(gray[y:y+h,x:x+w])
            #     # Check if confidence is less them 100 ==> "0" is perfect match 
            #     if (confidence < 100):
            #         id = self.names[id]
            #         confidence = "  {0}%".format(round(100 - confidence))
            #     else:
            #         id = "unknown"
            #         confidence = "  {0}%".format(round(100 - confidence))
            #     cv2.putText(frame, str(id), (x+5,y-5), self.font, 1, (255,255,255), 2)
            #     cv2.putText(frame, str(confidence), (x+5,y+h-5), self.font, 1, (255,255,255), 1)    
                cv2.arrowedLine(frame, (self.xc,self.yc),(self.xf,self.yf),(0,255,0), 2)

            cv2.circle(frame, (self.xc,self.yc), self.detect_radius, (0,255,0), 2)
            # 가이드라인
            cv2.line(frame, (160, 0), (160, 480), (0,255,0))
            cv2.line(frame, (480, 0), (480, 480), (0,255,0))
            # cv2.imshow('camera', img)
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            q_image = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            self.label.setPixmap(pixmap)

    def capture_image(self):
        frame = self.frame
        start_x, start_y, width, height = 160, 0, 320, 480
        end_x, end_y = start_x + width, start_y + height
        cropped_image = frame[start_y:end_y, start_x:end_x]
        # TODO 날짜 받기
        # date_now = str(datetime.datetime.now())
        # cv2.putText(cropped_image, date_now, (0, 400), fontFace=self.font, fontScale=1, color=(0,0,0), thickness=1, )#lineType[, bottomLeftOrigin]]])
        cv2.imwrite(save_image_path, cropped_image)
        self.show_captured_image()
        self.stop_webcam()
        
        
        # ret, frame = self.webcam.read()
        
        # if ret:
        #     frame = frame[240:480, 480:800]
        #     frame = cv2.bilateralFilter(frame, d=-1, sigmaColor=10, sigmaSpace=10)

        #     # #face crop
        #     # if self.following_face:
        #     #     w, h = 80, 120
        #     #     start_x, start_y, width, height = self.xf-w, self.yf-h, 2*w, 2*h
        #     #     end_x, end_y = start_x + width, start_y + height
        #     #     cropped_image = frame[start_y:end_y, start_x:end_x]
        #     #     cropped_image = cv2.resize(cropped_image, (320, 480), interpolation=cv2.INTER_CUBIC)
        #     # else:
        #     #     #big crop
        #     #     start_x, start_y, width, height = 160, 0, 320, 480
        #     #     end_x, end_y = start_x + width, start_y + height
        #     #     cropped_image = frame[start_y:end_y, start_x:end_x]

        #     start_x, start_y, width, height = 160, 0, 320, 480
        #     end_x, end_y = start_x + width, start_y + height
        #     cropped_image = frame[start_y:end_y, start_x:end_x]

        #     cv2.imwrite(save_image_path, cropped_image)
        #     self.show_captured_image()
        #     self.stop_webcam()
            

    def show_captured_image(self):
        if hasattr(self, 'captured_window') and self.captured_window is not None:
            self.captured_window.close()
            self.captured_window = None

        self.captured_window = QMainWindow()
        self.captured_window.setWindowTitle("Captured Image")
        self.captured_window.setGeometry(250, 200, 326, 560)
        
        label = QLabel()
        pixmap = QPixmap("captured_image.jpg")
        label.setPixmap(pixmap)
        
        layout = QVBoxLayout()
        layout.addWidget(label)
        
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.captured_window.setCentralWidget(central_widget)

        proceed_button = QPushButton("진행시켜", self.captured_window)
        proceed_button.setGeometry(6, 525, 161, 30)
        proceed_button.clicked.connect(self.proceed_action)

        reset_button = QPushButton("다시 찍기", self.captured_window)
        reset_button.setGeometry(170, 525, 161, 30)
        reset_button.clicked.connect(self.retry_action)

        self.captured_window.show()


    def show_wait_window(self):
        if hasattr(self, 'wait_window') and self.wait_window is not None:
            self.wait_window.close()
            self.wait_window = None
            
        self.wait_window = QDialog()
        self.wait_window.setWindowTitle('작업 진행 중')
        self.wait_window.setGeometry(100, 100, 300, 200)
        
        # QLabel을 생성하고, 중앙 정렬 설정
        wait_label = QLabel(self.wait_window)
        wait_label.setAlignment(Qt.AlignCenter)
        
        # QLabel에 표시할 텍스트 설정
        wait_label.setText("작업을 진행 중입니다. 잠시만 기다려주세요...")
        
        layout = QVBoxLayout()
        layout.addWidget(wait_label)
        self.wait_window.setLayout(layout)
        
        self.wait_window.show()

    def show_lined_img(self, image_path):
        self.lined_img = QMainWindow()
        self.lined_img.setWindowTitle("lined Image")
        self.lined_img.setGeometry(1000, 300, 220, 370)
        
        image_name = image_path[:-4]
        print(image_name)
        self.final_image_path = image_name +"\\" + image_name +"_final.jpg"
        
        layout = QHBoxLayout()

        
        image_label = QLabel()
        pixmap = QPixmap(self.final_image_path)
        image_label.setPixmap(pixmap)
        image_label.setAlignment(Qt.AlignTop)
        layout.addWidget(image_label)

        # QVBoxLayout을 메인 윈도우에 설정
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.lined_img.setCentralWidget(central_widget)

        start = 6
        d = 326
    
        button1 = QPushButton("그리기", self.lined_img)
        button1.setGeometry(start, 334, 105, 30)
        button1.clicked.connect(self.robot_drawing)

        button2 = QPushButton("취소", self.lined_img)
        button2.setGeometry(118, 334, 105, 30)
        button2.clicked.connect(self.restart)

        self.lined_img.show()

    def show_stylized_img(self):
        self.stylized_window = QMainWindow()
        self.stylized_window.setWindowTitle("stylized Image")
        self.stylized_window.setGeometry(250, 200, 320, 550)

        image_path = ["captured_image_rembg.jpg",
                      "captured_image_stylize_0_.jpg",
                      "captured_image_stylize_1_.jpg",
                      "captured_image_stylize_2_.jpg"]
        
        layout = QHBoxLayout()

        for path in image_path:
            image_label = QLabel()
            pixmap = QPixmap(path)
            image_label.setPixmap(pixmap)
            image_label.setAlignment(Qt.AlignTop)
            layout.addWidget(image_label)

        # QVBoxLayout을 메인 윈도우에 설정
        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.stylized_window.setCentralWidget(central_widget)

        start = 118
        d = 326
    
        button1 = QPushButton("선택", self.stylized_window)
        button1.setGeometry(start, 500, 100, 30)
        button1.clicked.connect(self.choose1)
        button2 = QPushButton("선택", self.stylized_window)
        button2.setGeometry(start+d, 500, 100, 30)
        button2.clicked.connect(self.choose2)
        button3 = QPushButton("선택", self.stylized_window)
        button3.setGeometry(start+2*d, 500, 100, 30)
        button3.clicked.connect(self.choose3)
        button4 = QPushButton("선택", self.stylized_window)
        button4.setGeometry(start+3*d, 500, 100, 30)
        button4.clicked.connect(self.choose4)

        self.stylized_window.show()


    def proceed_action(self): # 배경 제거
        robot_functions.indy.stop_teleop()
        ret = backimg_cut.remove_bg(save_image_path, output_file)
        if ret:
            print('bg removed')
            
            # image_rembg = cv2.imread(output_file+'.jpg', cv2.IMREAD_COLOR)
            # cv2.imshow("bk", image_rembg)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            # TODO 배경 제거 후 할 일
            for idx, checkpoint in enumerate(style):
                img = stylization.style_change(output_file+'.jpg', checkpoint, idx)
                # img_rmbg_st = cv2.imread('captured_image_stylize_' + str(idx) + '_.jpg', cv2.IMREAD_COLOR)
                # cv2.imshow("style"+str(idx), img_rmbg_st)

            self.show_stylized_img()



        else:
            print('bg remove error !')


    def choose1(self):
        self.stylized_window.close()
        self.captured_window.close()
        choose = 1
        self.drawing(choose)

    def choose2(self):
        self.stylized_window.close()
        self.captured_window.close()
        choose = 2
        self.drawing(choose)

    def choose3(self):
        self.stylized_window.close()
        self.captured_window.close()
        choose = 3
        self.drawing(choose)

    def choose4(self):
        self.stylized_window.close()
        self.captured_window.close()
        choose = 4
        self.drawing(choose)

    def drawing(self, choose):
        image_path = ["captured_image_rembg.jpg",
                      "captured_image_stylize_0_.jpg",
                      "captured_image_stylize_1_.jpg",
                      "captured_image_stylize_2_.jpg"]
        isDrawSucc, num_line = line_makingFn.line_makingFn(image_path[choose-1])
        if isDrawSucc: # 선 따는 함수가 성공했을 때
            print('Drawing success, number of line = ', num_line)
            self.show_lined_img(image_path[choose-1])
            # self.wait_window.close()
            # 드로우 결과 띄우고 최종 확인 버튼 누르는 창
        else:
            print('\033[31m' + '[ERROR] Drawing Error' + '\033[0m') # 31 빨간색 글씨
            print('\033[31m' + '[ERROR] Program Restart in 3 sec ...' + '\033[0m') # 31 빨간색 글씨
            print('\033[31m' + '[ERROR] 해당 오류가 발생하기 까지의 과정을 복기해보세요 영철 호출하기 ...' + '\033[0m') # 31 빨간색 글씨
            time.sleep(3)
            # self.restart()
            pass
        
        

    def retry_action(self):
        self.captured_window.close()
        self.start_webcam()
        pass

    def restart(self):
        self.lined_img.close()
        # os.execl(sys.executable, sys.executable, *sys.argv)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WebcamWindow()
    window.show()
    sys.exit(app.exec_())
