import ctypes


class Sample(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
    ]

    def __repr__(self):
        return f"Sample(x={self.x:.3f}, y={self.y:.3f})"

    @classmethod
    def from_bytes(cls, data: bytes | bytearray | memoryview, offset: int = 0):
        size = ctypes.sizeof(cls)
        if len(data) - offset < size:
            raise ValueError(f"Expected at least {size} bytes, got {len(data) - offset}")
        obj = cls()
        ctypes.memmove(ctypes.addressof(obj), (ctypes.c_char * size).from_buffer_copy(data[offset:offset+size]), size)
        return obj