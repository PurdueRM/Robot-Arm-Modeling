import socket

class TCP_Server:
    def __init__(self, host='localhost', port=65432):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket = None

    def start(self):
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        print(f"Server listening on {self.host}:{self.port}")
        self.client_socket, addr = self.server_socket.accept()
        print(f"Connected by {addr}")

    def send(self, data):
        if self.client_socket:
            self.client_socket.sendall(data.encode())

    def stop(self):
        if self.client_socket:
            self.client_socket.close()
        self.server_socket.close()

class TCP_Client:
    def __init__(self, host='localhost', port=65432):
        self.host = host
        self.port = port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        self.client_socket.connect((self.host, self.port))
        print(f"Connected to server at {self.host}:{self.port}")

    def send(self, data):
        self.client_socket.sendall(data.encode())

    def disconnect(self):
        self.client_socket.close()

class UDP_Server:
    def __init__(self, host='localhost', port=65432, broadcast_enabled=False):
        self.host = host
        self.port = port
        self.broadcast_enabled = broadcast_enabled
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if self.broadcast_enabled:
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def start(self):
        self.server_socket.bind((self.host, self.port))
        print(f"UDP Server listening on {self.host}:{self.port}")

    def receive(self):
        data, addr = self.server_socket.recvfrom(1024)
        print(f"Received message from {addr}: {data.decode()}")

    def stop(self):
        self.server_socket.close()

    def broadcast(self, data):
        if self.broadcast_enabled:
            self.server_socket.sendto(data.encode(), ('<broadcast>', self.port))
        else:
            print("Broadcasting is not enabled for this server.")

class UDP_Client:
    def __init__(self, host='localhost', port=65432, broadcast_enabled=False):
        self.host = host
        self.port = port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if broadcast_enabled:
            self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.client_socket.bind(('', self.port))
        self.client_socket.settimeout(1.0)

    def receive(self):
        # print(f"UDP Client listening on port {self.port}...")
        data, addr = self.client_socket.recvfrom(1024)
        # print(f"Received message from {addr}: {data.decode()}")
        return data.decode()

    def send(self, data):
        self.client_socket.sendto(data.encode(), (self.host, self.port))

    def disconnect(self):
        self.client_socket.close()

