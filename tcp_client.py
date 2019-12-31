import socket
import time


client = socket.socket()
client.connect(('192.168.0.114', 60002))
msg = 'xxxxxxx'
client.send(msg.encode('utf-8'))
time.sleep(3)
client.close()