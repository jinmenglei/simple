"""
这个是平移的穿山甲socket server
"""
import socketserver
socket_dict = {}


class SocketServer(socketserver.BaseRequestHandler):
    def handle(self):
        print('get socket')
        while True:
            try:
                data = self.request.recv(1024).strip()
                if len(data) > 0:
                    print('data : ' + str(data))
                    print(self.client_address)
            except ConnectionResetError as e:
                print('error : ' + str(e))


if __name__ == '__main__':
    host, port = '0.0.0.0', 60002
    server = socketserver.ThreadingTCPServer((host, port), SocketServer)
    server.serve_forever()