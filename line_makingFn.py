import cv2
import numpy as np
import matplotlib.pyplot as plt
from drawfunc import *
import os
from arguments import *
import traceback

def line_makingFn(img_path):
    try:
        #1 edge_img
        # img_path = 'cheese_re7.jpg'
        if img_path.split('.')[-1] != 'jpg' and img_path.split('.')[-1] != 'png':
            assert 0, f"only jpg and png avilable, {img_path.split('.')[-1]} is wrong type!"

        scale = 320
        img = cv2.imread(img_path)
        if (img.shape[0] < img.shape[1]):
            img = cv2.resize(img, (scale, scale * img.shape[0]//img.shape[1]))
        else:
            img = cv2.resize(img, (scale * img.shape[1]//img.shape[0], scale))
        print(img.shape)
        threshold1 = 130
        threshold2 = 150
        edge_img = cv2.Canny(img, threshold1, threshold2)
        # cv2.imshow('frame', edge_img)
        img_name = img_path.split('.')[-2].split('/')[-1]
        save_path = os.path.join(os.path.split(img_path)[0], img_name)
        print(save_path)
        if not os.path.isdir(save_path):
            os.mkdir(save_path)

        # edge_img = 255 - edge_img
        cv2.imwrite(os.path.join(save_path, f'{img_name}_edge.jpg'), edge_img)
        # plt.imshow(edge_img)

        #2 line_info[[sp1, sp2], d1, d2, ...]
        line_info = []

        #3 check layer: ckVec
        ckVec = np.zeros(edge_img.shape) # 비면 0, 선 시작은 2, 선 중간은 1, 단일픽셀로 지워진 경우 3
        line_info = []


        # STEP2 이미지를 그리기 위한 선의 궤적을 추출한다. 
        #1 이미지의 모든 픽셀이 선으로 표현 가능하도록 한다.
        progress_num = 0
        while True:
            for j in range(50):
                line_info, ckVec = lining(edge_img, line_info, ckVec)
            
            ckVec = erase_noise(edge_img, ckVec)
            # print_img(ckVec, line_info, f"img_{progress_num}", True) 중간 과정 출력하고 싶을 때 사용
            progress_num+=1
            if imgIsEmpty(edge_img, ckVec) or progress_num==1000:
                break

        #2 길이가 짧은 선(10 이하)를 노이즈로 간주하고 제거한다. 
        noise_len = 10 # 필수적인 선이 지워지는 경우 크기를 줄여본다. 

        line_clear = noise_del(line_info, noise_len)

        #3 총 선의 개수 출력
        line_num=0
        for i in line_clear:
            if i == 100:
                continue
            line_num+=1
        print(f" Total Line number: {line_num}")

        # STEP3 얻은 데이터를 출력한다. 
        #1 최종 이미지 출력
        final_img = print_img(ckVec, line_clear, os.path.join(save_path, f"{img_name}_final") ,True)

        # 2 선 길이 분포표
        line_len = []
        for i in line_clear:
            if i == 100:
                continue
            line_len.append(len(i) - 1)
            
        # plt.hist(line_len)

        # plt.show()


        #3 line_clear를 txt 파일로 내보내기
        with open(os.path.join(save_path, f"{img_name}_line.txt"), "w") as f:
            for line in line_clear:
                if line == 100:
                    continue
                data = str(line[0][0]) + " " + str(line[0][1]) 
                for j in line[1: ]:
                    data = data + " " + str(j)
                
                f.write(data)
                f.write("\n")

        return (1, line_num)
    except Exception as e:
        print(e)
        print(traceback.format_exc())



        return (0, 0)