# encoding: utf-8
import socket               
 
tcpClient = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # 创建socket对象
# host = socket.gethostname() 
host = '127.0.0.1'
port = 1080 
addr = (host, port)
tcpClient.connect(addr) # 连接服务，指定主机和端口号
data = b'\x01\x64\xff' # 报文数据，bytes类型
tcpClient.send(data) # 发送数据给服务端
msg = tcpClient.recv(1024) # 接收来自服务端的数据，小于1024字节
print(msg.decode('utf-8'))
tcpClient.close()
