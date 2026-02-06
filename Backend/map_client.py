import socket
import json

TCP_IP, TCP_PORT = "127.0.0.1", 5005

def request_map():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect((TCP_IP, TCP_PORT))

    s.sendall(json.dumps({"command": "GET_MAP"}).encode())

    data = b""
    while True:
        try:
            packet = s.recv(4096)
            if not packet:
                break
            data += packet
        except socket.timeout:
            break

    s.close()

    if not data:
        raise RuntimeError("No data received from supervisor")

    return json.loads(data.decode())

if __name__ == "__main__":
    from pprint import pprint
    pprint(request_map())
