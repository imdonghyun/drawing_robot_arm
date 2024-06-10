# 배경 제거와 스타일 변경을 한꺼번에 진행하는 모듈이다.

from rembg import remove
from PIL import Image
import os

input_file = 'cheese.jpg'
output_file = 'cheese_rembg'

img = Image.open(input_file) 

# 배경 제거하기
out = remove(img) 

# 변경된 이미지 저장하기
out.save(output_file+".png") 
os.rename(output_file+".png",output_file+'.jpg')

