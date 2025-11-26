import socket

class SimpleSocketServer:
    def __init__(self, host="0.0.0.0", port=9000):
        self.host = host
        self.port = port

        self.server_sock = None
        self.client_sock = None
        self.client_addr = None

    
    def start(self):
        """Create, bind, and listen."""
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.settimeout(1.0)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(1)
        print(f"Listening on {self.host}:{self.port}")

    def accept(self):
        """Accept a single client connection."""
        self.client_sock, self.client_addr = self.server_sock.accept()
        self.client_sock.settimeout(1.0)
        print(f"Client connected: {self.client_addr}")

    def send_bytes(self, data: bytes):
        """Send raw bytes to the client."""
        if not self.client_sock:
            raise RuntimeError("No client connected")
        self.client_sock.sendall(data)

    def recv_exact(self, n: int) -> bytes:
        if not self.client_sock:
            raise RuntimeError("No client connected")
        buf = bytearray()
        while len(buf) < n:
            chunk = self.client_sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("Socket closed before receiving enough data")
            buf.extend(chunk)
        return bytes(buf)

    def receive_bytes(self,bytes) -> bytes:
        """Receive raw bytes. Blocks until some are received."""
        if not self.client_sock:
            raise RuntimeError("No client connected")
        return self.client_sock.recv(bytes)

    def close_client(self):
        """Close client connection only."""
        if self.client_sock:
            self.client_sock.close()
            self.client_sock = None
            print("Client disconnected")

    def stop(self):
        """Close everything."""
        self.close_client()
        if self.server_sock:
            self.server_sock.close()
            self.server_sock = None
            print("Server stopped")
