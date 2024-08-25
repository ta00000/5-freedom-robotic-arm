# encoding: utf-8
import socket
import json
import time

HOST = "192.168.23.178" # 接收数据的树莓派的 IP 地址
PORT = 1080         # 接收数据的树莓派的端口号

while True:
    # 构建要发送的数据
    data = {
        'key1': 'value1',
        'key2': 2,
        'key3': [3, 4, 5]
    }
    # 将数据编码为 JSON 格式
    json_data = json.dumps(data)

    # 创建套接字并发送数据
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(json_data.encode())

    # 等待1秒
    time.sleep(0.5)
