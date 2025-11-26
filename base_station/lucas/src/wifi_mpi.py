from SimpleSocketServer import SimpleSocketServer

_socket_server: SimpleSocketServer = None

TEXT_STREAM_OUT = 0xFF
PING = 0xFE

class Message:
    code: int
    data: bytes

function_ptrs = {}

def setup(socket_server: SimpleSocketServer):
    global _socket_server
    _socket_server = socket_server

def send_bytes(code: int, data: bytes):    
    
    payload_length = len(data)
    bytes_to_send = code.to_bytes(1, "little") + payload_length.to_bytes(4, "little") + data
    _socket_server.send_bytes(bytes_to_send) 

def send(code: int):
    payload_length = 0
    header = code.to_bytes(1, "little") + payload_length.to_bytes(4, "little")
    _socket_server.send_bytes(header) 
    
def register_message_handler(code: int, function):
    function_ptrs[code] = function

def handle_message(m: Message):
    if m.code in function_ptrs:
        function_ptrs[m.code](m)
        return True
    return False

def process_message(code: int):
    m = Message()
    m.code = code
    length_bytes = _socket_server.recv_exact(4)
    length = int.from_bytes(length_bytes, "little")
    m.data = _socket_server.recv_exact(length)
    handle_message(m)

def process_messages():
    messages = 10
    while messages > 0:
        code_bytes = _socket_server.recv_exact(1)
        if code_bytes is None:
            break
        process_message(code_bytes[0])
        messages = messages - 1
