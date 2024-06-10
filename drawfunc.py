import cv2
import numpy as np
import matplotlib.pyplot as plt


# now_pos와 num을 입력해 주면 대응하는 다음 좌표 반환
def next_pixel(now_pos, vec):
    
    if vec == 7:
        return [now_pos[0]-1, now_pos[1]-1]
    elif vec == 0:
        return [now_pos[0]-1, now_pos[1]  ]
    elif vec == 1:
        return [now_pos[0]-1, now_pos[1]+1]
    elif vec == 2:
        return [now_pos[0]  , now_pos[1]+1]
    elif vec == 3:
        return [now_pos[0]+1, now_pos[1]+1]
    elif vec == 4:
        return [now_pos[0]+1, now_pos[1]  ]
    elif vec == 5:
        return [now_pos[0]+1, now_pos[1]-1]
    elif vec == 6:
        return [now_pos[0]  , now_pos[1]-1]
    else: 

        assert 0, "Wrong input on function 'next_pixel' "



# start point serch function

def spSerch(edge_img, ckVec):
    img_shape = edge_img.shape

    spExist = False

    # 테두리를 모두 비우는 과정
    for k in range(edge_img.shape[0]):
        edge_img[k][ 0] = 0
        edge_img[k][-1] = 0
    for l in range(edge_img.shape[1]):
        edge_img[ 0][l] = 0
        edge_img[-1][l] = 0
    
    # find the initial point
    for i in range(img_shape[0]-1, 0, -1):
        for j in range(img_shape[1]):
            if (edge_img[i][j] != 0 and ckVec[i][j] == 0) :
                sp = [i, j]
                ckVec[i][j] = 2
                spExist = True
                break

                
        if edge_img[i][j] != 0:
            break
            
    # print(sp)
    if spExist:
        return sp, ckVec
    else:
        return [-1, -1], ckVec

# lining function
def lining(edge_img, line_info, ckVec):
    line = [] # 새로운 선 성분을 저장하는 list
    
    # start point를 line에 추가, ckVec 업데이트 수행
    sp, ckVec = spSerch(edge_img, ckVec)
    if sp[0]==-1:
        assert "No sp ERROR!"
    line.append(sp)
    
    vec = 0 # 진행 방향 지정 변수
    pixel = edge_img[sp[0]][sp[1]] # 현재 탐색 중인 픽셀 지정 변수
    checkCross = 0 # 4 이상이면 교차점, 4 미만이면 순환궤도
    now_pos = sp # 검증할 점의 위치
    a=0
    while(1):
        a+=1
        checkCross = 0 # 값 초기화
        # 주변에 점이 있는 경우
        i=0
        for i in range(8):
            ckpt = next_pixel(now_pos, vec) # 검증할 점의 좌표
            if ckpt[0] >= edge_img.shape[0] or ckpt[1] >= edge_img.shape[1]: #########################################################################################################
                continue
            ckPix = edge_img[ckpt[0]][ckpt[1]]
            
            
            # print(ckPix)
            if ((ckVec[ckpt[0]][ckpt[1]] == 0) and (ckPix != 0)): #체크벡터가 0이고, 검증할 점에 색이 있으면
                line.append(vec) # 선에 이동 방향 저장
                ckVec[ckpt[0]][ckpt[1]] = 1 # ckVec 업데이트
                now_pos = ckpt
                # print('1')
                i = 0
                break
            
            elif ((ckVec[ckpt[0]][ckpt[1]] == 0) and (ckPix == 0)): # ckVec이 0이고    , 점에 색이 없는 경우
                vec += 1
            elif (ckPix == 0): # ckvec이 0이 아니고, 점에 색이 없는 경우: error
                assert 0, "ckvec이 0이 아니고, 점에 색이 없는 경우: error"
            else: # ckVec이 0이 아니고, 점에 색이 있는 경우
                checkCross += 1
                vec += 1
            
            vec = vec % 8
        
        
        assert vec <= 7, "CheckNextPixelError: Too big variable 'vec'!"
        
        if (i == 7 and checkCross < 3): # 선이 종결된 경우
            if (len(line_info)+1) % 10 == 0:
                print("Line " + str(len(line_info)+1) + " done!")
           
            break
            
        elif (i == 7): # 교차 확인 시 진행
            buffer = 0
            for k in range(2):
                now_pos = next_pixel(now_pos, vec) # 진행 방향으로 일단 한 픽셀 이동하여 그 주위를 검토
                line.append(vec)
                for j in range(8):

                    ckpt = next_pixel(now_pos, vec) # 검증할 점의 좌표
                    if ckpt[0] >= edge_img.shape[0] or ckpt[1] >= edge_img.shape[1]: #########################################################################################################
                        continue
                    ckPix = edge_img[ckpt[0]][ckpt[1]]



                    if ((ckVec[ckpt[0]][ckpt[1]] == 0) and (ckPix != 0)): #체크벡터가 0이고, 검증할 점에 색이 있으면
                        line.append(vec) # 선에 이동 방향 저장
                        ckVec[ckpt[0]][ckpt[1]] = 1 # ckVec 업데이트
                        now_pos = ckpt
                        buffer = 1
                        
                        break

                    elif ((ckVec[ckpt[0]][ckpt[1]] == 0) and (ckPix == 0)): # ckVec이 0이고    , 점에 색이 없는 경우
                        vec += 1
                    elif (ckPix == 0): # ckvec이 0이 아니고, 점에 색이 없는 경우: error
                        assert 0, "ckvec이 0이 아니고, 점에 색이 없는 경우: error"
                    else: # ckVec이 0이 아니고, 점에 색이 있는 경우
                        vec += 1

                    vec = vec % 8
                    
                if buffer == 1:
                    break
            
        else: # 그냥 이어지는 경우
            # print(vec)
            pass
        
    line_info.append(line)       
    return line_info, ckVec


def noise_del(line_info, threshold):
    for i in range(len(line_info)):
        if line_info[i] == 100:
            continue
        if len(line_info[i]) <= threshold + 1: # sp 정보가 한 칸 차지함
            line_info[i] = 100
        
    return line_info
            



# line 읽어서 이미지 만드는 과정
def print_img(ckVec, line_info, img_name, printImg=False):
    img_vec = np.zeros(list(ckVec.shape), dtype=np.uint8)
    frame_array = []

    for i in line_info:
        if i == 100: # noise_del 함수에서 지워진 부분이 100으로 치환됨
            continue

        sp = i[0] # start point 변수로 받음
        for j in range(1, len(i)): # 선의 길이만큼
            img_vec[sp[0], sp[1]] = 255
            sp = next_pixel(sp, i[j])
    if printImg:
        print(f"{img_name} print!")
        cv2.imwrite(f'{img_name}.jpg',img_vec)
        
    return img_vec


def imgIsEmpty(edge_img, ckVec):
    # ckVec 변환
    for a in range(ckVec.shape[0]):
        for b in range(ckVec.shape[1]):
            if ckVec[a][b] > 0:
                ckVec[a][b] = 255
    # 둘의 차를 입력
    img = edge_img-ckVec    

    # 각 픽셀에 대해...        
    for i in img:
        for j in i:
            if j!=0:
                return False
    print("Image is empty!")
    return True

def erase_noise(edge_img, ckVec):
    num=0
    for row in range(1, edge_img.shape[0]-1):
        for col in range(1, edge_img.shape[1]-1):
            if edge_img[row][col]!=0 and ckVec[row][col] == 0:
                buf = 0
                for vec in range(8):
                    ckPix = next_pixel([row, col], vec)
                    if edge_img[ckPix[0]][ckPix[1]]==0:
                        buf += 1
                if buf==8:
                    num+=1
                    #print(f"{row}, {col} erased!")
                    ckVec[row][col] = 3
    print(f"erased {num} pixels!")
    return ckVec

def check():
    return 10