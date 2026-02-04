import socket
import json

TCP_IP = "127.0.0.1"
TCP_PORT = 5005

def request_map():
    """
    Request map data (obstacle, start, goal)
    from Webots Supervisor via TCP
    """
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect((TCP_IP, TCP_PORT))

    request = {
        "command": "GET_MAP"
    }

    client.send(json.dumps(request).encode())

    data = client.recv(8192)
    client.close()

    return json.loads(data.decode())


# ================= TEST MANUAL =================
if __name__ == "__main__":
    world_map = request_map()
    print("=== MAP RECEIVED ===")
    print(json.dumps(world_map, indent=2))
