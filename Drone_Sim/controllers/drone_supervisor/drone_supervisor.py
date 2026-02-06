from controller import Supervisor
import socket
import json
import math

# --- Konfigurasi TCP ---
HOST = '127.0.0.1'
PORT = 65432

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

drone_node = robot.getSelf()
target_node = robot.getFromDef("TARGET")
root_group = robot.getRoot()
children_field = root_group.getField("children")

def fly_to_target(current_pos, target_pos, speed=0.15):
    dx, dy = target_pos[0] - current_pos[0], target_pos[1] - current_pos[1]
    dist = math.sqrt(dx**2 + dy**2)
    if dist < 0.1: return True, current_pos 
    
    vx, vy = (dx / dist) * speed, (dy / dist) * speed
    new_pos = [current_pos[0] + vx, current_pos[1] + vy, 1.0]
    drone_node.getField("translation").setSFVec3f(new_pos)
    
    angle = math.atan2(dy, dx)
    drone_node.getField("rotation").setSFRotation([0, 0, 1, angle])
    return False, new_pos

# Setup Server Socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
server_socket.setblocking(False)

print(f"Webots Server listening on {PORT}...")
client_conn = None
waypoints, wp_index, is_flying = [], 0, False

while robot.step(timestep) != -1:
    if client_conn is None:
        try:
            client_conn, addr = server_socket.accept()
            client_conn.setblocking(False)
            print(f"Connected to GUI: {addr}")
        except BlockingIOError: pass
    else:
        try:
            data = client_conn.recv(4096)
            if data:
                msg = json.loads(data.decode('utf-8'))
                cmd = msg.get("command")
                
                if cmd == "GET_MAP":
                    obs_list = []
                    for i in range(children_field.getCount()):
                        node = children_field.getMFNode(i)
                        if node.getDef() and "OBSTACLE" in node.getDef():
                            pos = node.getPosition()
                            size = node.getField("size").getSFVec3f()
                            rot = node.getField("rotation").getSFRotation()
                            # Ambil rotasi sumbu Z
                            angle_z = rot[3] if rot[2] == 1.0 else (-rot[3] if rot[2] == -1.0 else 0.0)
                            obs_list.append({"x": pos[0], "y": pos[1], "w": size[0], "h": size[1], "rot": angle_z})
                    
                    resp = {
                        "start": drone_node.getPosition()[:2],
                        "goal": target_node.getPosition()[:2],
                        "obstacles": obs_list
                    }
                    client_conn.sendall(json.dumps(resp).encode('utf-8'))

                elif cmd == "START_SIM":
                    waypoints = msg.get("path")
                    if waypoints:
                        wp_index, is_flying = 0, True
        except (BlockingIOError, ConnectionResetError): pass

    if is_flying and wp_index < len(waypoints):
        arrived, _ = fly_to_target(drone_node.getPosition(), waypoints[wp_index])
        if arrived: wp_index += 1
    elif wp_index >= len(waypoints):
        is_flying = False