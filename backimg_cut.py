from rembg import remove
from PIL import Image
import os

# input_file = 'cheese.jpg'
# output_file = 'cheese_rembg'

# 

# # 배경 제거하기
# out = remove(img) 

# # 변경된 이미지 저장하기
# 
# os.rename(output_file+".png",output_file+'.jpg')


def remove_bg(input_file, output_file):
    try:
        img = Image.open(input_file) 
        out = remove(img)
        out.save(output_file + ".png")

        if not os.path.isfile(output_file+'.jpg'):
            os.rename(output_file+".png",output_file+'.jpg')
        else:
            os.remove(output_file+'.jpg')
            os.rename(output_file+".png",output_file+'.jpg')
        return 1
    except Exception as e:
        print('Background remove error !', e)
        return 0