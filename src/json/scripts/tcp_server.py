# encoding: utf-8
import socket

tcpServer = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # 创建socket对象，走tcp通道
# host = socket.gethostname() # 获取本地主机名
host = '192.168.23.112'
port = 1080 # 端口号
addr = (host, port)
tcpServer.bind(addr) # 绑定地址
tcpServer.listen(5) # 设置最大连接数，超过后排队

# while True:
conn,addr = tcpServer.accept() # 建立客户端连接
print(conn)
# data = conn.recv(1024) # 接收来自客户端的数据，小于1024字节
# print(data)
# msg = 'Hello Client'.encode('utf-8')
# conn.send(msg) # 发送数据给客户端

with open('/home/tbs/le_arm/src/json/scripts/test.txt', 'rb') as file:
    data = file.read(1024)
    while data:
        conn.send(data)
        data = file.read(1024)

print("文件传输完成")

conn.close() # 关闭连接

