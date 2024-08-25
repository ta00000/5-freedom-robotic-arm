import socket
import json

HOST = '192.168.43.123'     # 绑定所有网络接口
PORT = 5000      # 监听的端口号

while True:
    # 创建套接字并开始监听
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            print('Connected by', addr)
            while True:
                # 接收数据
                data = conn.recv(1024)
                if not data:
                    break
                # 将接收到的数据解码为 JSON 格式
                json_data = data.decode()
                data = json.loads(json_data)
                print(data)
