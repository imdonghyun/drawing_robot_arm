# import json
# from easydict import EasyDict


# json_data = [] # your list with json objects (dicts)

# with open('calibration_config.json') as json_file:
#    pen_height_data = json.load(json_file)



# print(pen_height_data['pen_height_1'])
# print(pen_height_data['pen_height_2'])
# print(pen_height_data['pen_height_3'])
# print(pen_height_data['pen_height_4'])
# # json_data['p_goal'][0] = 200  # 200 -> 1

# with open('calibration_config.json', 'w') as make_file:
#     json.dump(dict(json_data), make_file)

	
# pen_height = 1000*0.300
# center_x = 1000*(0.2)
# center_y = 1000*(0.5)
# width = 1000*(0.21)
# height = 1000*(0.297)
# origin_x = center_x - width/2
# origin_y = center_y + height/2

# print(origin_x)
# print(origin_y)
# # width_pixel = 213
# # height_pixel = 320



name = "captured_image_stylize_2__final.jpg"
# captured_image_stylize_2__line.txt
print(name[:-9] + "line.txt")