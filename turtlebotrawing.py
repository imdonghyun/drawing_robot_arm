import turtle
import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
def draw_lines_with_turtle(line_coordinates):
    # 초기화

    turtle.home()
    turtle.speed(10)

    turtle.penup()
    print(line_coordinates)
    # time.sleep(1)
    turtle.goto(int(line_coordinates[0]), int(line_coordinates[1]))
    
   #print(line_coordinates)
    # 좌표 리스트를 따라 그리기
    turtle.pendown()
    for line in line_coordinates[2:]:
        # print(line)
        x,y = turtle.pos()
        if line == 0:
            turtle.goto(int(x-1), int(y))
        elif line == 1:
            turtle.goto(int(x-1), int(y+1))
        elif line == 2:
            turtle.goto(int(x), int(y+1))
        elif line == 3:
            turtle.goto(int(x+1), int(y+1))
        elif line == 4:
            turtle.goto(int(x+1), int(y))
        elif line == 5:
            turtle.goto(int(x+1), int(y-1))
        elif line == 6:
            turtle.goto(int(x), int(y-1))
        elif line == 7:
            turtle.goto(int(x-1), int(y-1))
    turtle.penup()

    # 화면 유지
    # turtle.done()

# 이미지 경로 설정





def read_integers_from_notepad(file_path):
    integers = []
    with open(file_path, 'r') as file:
        for line in file:
            # 스페이스를 기준으로 문자열을 분리하여 정수로 변환한 후 리스트에 추가
            integers.append(list(map(int, line.strip().split())))
    return integers

# 메모장 파일 경로
notepad_file_path = 'image_line.txt'

# 한 줄씩 읽어오기
lines = read_integers_from_notepad(notepad_file_path)
print(lines)
# 결과 출력
for line in lines:
    draw_lines_with_turtle(line)

turtle.done()

