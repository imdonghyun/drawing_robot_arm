# print('\033[31m' + '[ERROR] Drawing Error' + '\033[0m') # 31 빨간색 글씨
# Indy robot이 움직임 명령을 받고 움직이는 서버 스크립트.

import socket
import sys
import robot_functions
# 서버 설정

class SERVER:
    def __init__(self):
        self.host_ip = "127.0.0.1"      # 서버의 IP 주소 또는 도메인 이름
        self.port = 12345               # 포트 번호
        
        # 서버 소켓 생성
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host_ip, self.port))
        self.server_socket.listen(5)
        print(f"서버가 {self.host_ip}:{self.port}에서 대기 중입니다...")
        self.wait_client()

    def wait_client(self):
        while True:
        # 클라이언트 연결 대기
            client_socket, client_address = self.server_socket.accept()
            print(f"클라이언트 {client_address}가 연결되었습니다.")
        
            try:
                # 클라이언트로부터 요청 받기
                data = client_socket.recv(1024).decode("utf-8")
                
                if not data:
                    continue

                # 요청 파싱
                print("recieved msg = ",data)
                if data == 'home':
                    # print("homoe!!!!!!!!!!!!")
                    #TODO go home position
                    robot_functions.move_to_base()
                    response = "yes you go home"
                
                elif data == 'photo':
                    # print('going to photo')
                    #TODO go to photo spot
                    robot_functions.move_to_photo_base()
                    response = "yes you go to photo"

                elif data == 'draw_base':
                    robot_functions.move_to_draw_base()
                    response = "yes you go to photo"
                else:
                    response = "there is some error you should send home or photo"
                client_socket.send(response.encode("utf-8"))
                

            except Exception as e:
                print(f"오류 발생: {e}")
            except KeyboardInterrupt:
                print ('Keyboard Interrupted')
                sys.exit(0)
            
            finally:
                # 클라이언트 소켓 닫기
                print("연결종료")
                client_socket.close()



if __name__ == "__main__":
    try:
        indy_robot=SERVER()
    except KeyboardInterrupt:
        print ('Interrupted')
        sys.exit(0)
