import tkinter as tk
from tkinter import messagebox, ttk
import socket, json, threading, math, random, time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.patches as patches
from matplotlib.collections import LineCollection
import numpy as np

# --- Helper Math ---
def is_point_in_rect(px, py, obs, margin=0.45):
    tx, ty = px - obs['x'], py - obs['y']
    c, s = math.cos(-obs['rot']), math.sin(-obs['rot'])
    lx, ly = tx * c - ty * s, tx * s + ty * c
    return - (obs['w']/2 + margin) <= lx <= (obs['w']/2 + margin) and \
           - (obs['h']/2 + margin) <= ly <= (obs['h']/2 + margin)

def get_corners(obs):
    c, s = math.cos(obs['rot']), math.sin(obs['rot'])
    hw, hh = obs['w']/2, obs['h']/2
    pts = [(-hw,-hh), (hw,-hh), (hw,hh), (-hw,hh)]
    return [[obs['x'] + (x*c - y*s), obs['y'] + (x*s + y*c)] for x, y in pts]

# --- Planning Engine Optimized ---
class PlanningEngine:
    def __init__(self, start, goal, obstacles):
        self.start = {"x": start[0], "y": start[1], "parent": None, "cost": 0.0}
        self.goal = {"x": goal[0], "y": goal[1]}
        self.obstacles = obstacles
        self.node_list = []
        self.expand_dis = 0.7 
        self.search_radius = 1.8 

    def check_collision(self, x, y):
        for o in self.obstacles:
            if is_point_in_rect(x, y, o): return True
        return False

    def is_line_safe(self, p1, p2):
        dist = math.dist(p1, p2)
        steps = int(dist / 0.3) # Optimasi step size
        for i in range(steps + 1):
            t = i / max(1, steps)
            if self.check_collision(p1[0] + t*(p2[0]-p1[0]), p1[1] + t*(p2[1]-p1[1])):
                return False
        return True

    def solve_rrt_star(self, max_iter=2500):
        self.node_list = [self.start]
        for _ in range(max_iter):
            rnd = [random.uniform(-7.5, 7.5), random.uniform(-7.5, 7.5)]
            nearest = min(self.node_list, key=lambda n: (n["x"]-rnd[0])**2 + (n["y"]-rnd[1])**2)
            
            theta = math.atan2(rnd[1] - nearest["y"], rnd[0] - nearest["x"])
            new_node = {
                "x": nearest["x"] + self.expand_dis * math.cos(theta),
                "y": nearest["y"] + self.expand_dis * math.sin(theta),
                "parent": nearest,
                "cost": nearest["cost"] + self.expand_dis
            }

            if not self.check_collision(new_node["x"], new_node["y"]):
                # Optimization: Find near nodes efficiently
                near_nodes = [n for n in self.node_list if (n["x"]-new_node["x"])**2 + (n["y"]-new_node["y"])**2 <= self.search_radius**2]
                
                # Choose Parent
                for near in near_nodes:
                    d = math.dist([near["x"], near["y"]], [new_node["x"], new_node["y"]])
                    if near["cost"] + d < new_node["cost"]:
                        if self.is_line_safe([near["x"], near["y"]], [new_node["x"], new_node["y"]]):
                            new_node["cost"] = near["cost"] + d
                            new_node["parent"] = near
                
                self.node_list.append(new_node)

                # Rewire
                for near in near_nodes:
                    d = math.dist([new_node["x"], new_node["y"]], [near["x"], near["y"]])
                    if new_node["cost"] + d < near["cost"]:
                        if self.is_line_safe([new_node["x"], new_node["y"]], [near["x"], near["y"]]):
                            near["parent"] = new_node
                            near["cost"] = new_node["cost"] + d
        
        # Cari node terdekat ke goal setelah iterasi selesai
        last_node = min(self.node_list, key=lambda n: math.dist([n["x"], n["y"]], [self.goal["x"], self.goal["y"]]))
        return self.extract_path(last_node)

    def solve_multibias(self, max_iter=2500):
        self.node_list = [self.start]
        for _ in range(max_iter):
            p = random.random()
            if p < 0.15: rnd = [self.goal["x"], self.goal["y"]]
            elif p < 0.60: # Gaussian
                sigma = 2.0
                x1, y1 = random.uniform(-7.5, 7.5), random.uniform(-7.5, 7.5)
                x2, y2 = random.gauss(x1, sigma), random.gauss(y1, sigma)
                rnd = [(x1+x2)/2, (y1+y2)/2] if self.check_collision(x1, y1) != self.check_collision(x2, y2) else [x1, y1]
            else: rnd = [random.uniform(-7.5, 7.5), random.uniform(-7.5, 7.5)]

            nearest = min(self.node_list, key=lambda n: (n["x"]-rnd[0])**2 + (n["y"]-rnd[1])**2)
            theta = math.atan2(rnd[1]-nearest["y"], rnd[0]-nearest["x"])
            new_node = {"x": nearest["x"] + self.expand_dis*math.cos(theta), "y": nearest["y"] + self.expand_dis*math.sin(theta), "parent": nearest, "cost": nearest["cost"] + self.expand_dis}

            if not self.check_collision(new_node["x"], new_node["y"]):
                self.node_list.append(new_node)
                if math.dist([new_node["x"], new_node["y"]], [self.goal["x"], self.goal["y"]]) < self.expand_dis:
                    return self.extract_path(new_node)
        return None

    def extract_path(self, node):
        path = [[self.goal["x"], self.goal["y"]]]
        curr = node
        while curr:
            path.append([curr["x"], curr["y"]])
            curr = curr["parent"]
        return path[::-1]

    def smooth_path(self, path):
        if not path or len(path) < 3: return path
        smoothed = [path[0]]
        curr = 0
        while curr < len(path) - 1:
            for test in range(len(path) - 1, curr, -1):
                if self.is_line_safe(path[curr], path[test]):
                    smoothed.append(path[test])
                    curr = test
                    break
        return smoothed

# --- GUI Application ---
class DroneApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Path Planner (Fast Engine)")
        self.sock = None
        self.data = {"start": None, "goal": None, "obs": [], "raw": [], "smooth": []}

        # UI Setup
        side = tk.Frame(root, width=220, bg="#f3f4f6")
        side.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        
        tk.Label(side, text="ALGO SELECTION", font=("Arial", 9, "bold")).pack(pady=5)
        self.algo_choice = tk.StringVar(value="multibias")
        ttk.Radiobutton(side, text="Standard RRT*", variable=self.algo_choice, value="rrtstar").pack(anchor="w")
        ttk.Radiobutton(side, text="Multi-Bias (Gaussian)", variable=self.algo_choice, value="multibias").pack(anchor="w")

        ttk.Separator(side, orient='horizontal').pack(fill='x', pady=10)
        
        self.btn_map = ttk.Button(side, text="1. Load Map", command=self.get_map)
        self.btn_map.pack(fill='x', pady=2)
        self.btn_rrt = ttk.Button(side, text="2. Run Planning", command=self.start_thread, state="disabled")
        self.btn_rrt.pack(fill='x', pady=2)
        self.btn_sim = ttk.Button(side, text="3. Fly Drone", command=self.fly, state="disabled")
        self.btn_sim.pack(fill='x', pady=2)
        
        self.lbl_status = tk.Label(side, text="Status: Ready", fg="gray")
        self.lbl_status.pack(pady=20)

        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        self.init_ax()
        
        # Connect automatic
        self.connect()

    def init_ax(self):
        self.ax.clear()
        self.ax.set_xlim(-8, 8); self.ax.set_ylim(-8, 8)
        self.canvas.draw_idle()

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect(('127.0.0.1', 65432))
            self.lbl_status.config(text="Status: Connected", fg="green")
        except: self.lbl_status.config(text="Status: Webots Offline", fg="red")

    def get_map(self):
        try:
            self.sock.sendall(json.dumps({"command": "GET_MAP"}).encode())
            resp = json.loads(self.sock.recv(16384).decode())
            self.data.update({"start": resp['start'], "goal": resp['goal'], "obs": resp['obstacles']})
            self.draw_world()
            self.btn_rrt.config(state="normal")
        except: messagebox.showerror("Error", "Gagal ambil data map!")

    def draw_world(self):
        self.ax.clear()
        for o in self.data['obs']:
            self.ax.add_patch(patches.Polygon(get_corners(o), color='#2c3e50', alpha=0.8))
        self.ax.plot(*self.data['start'], 'go')
        self.ax.plot(*self.data['goal'], 'r*', markersize=15)
        self.ax.set_xlim(-8, 8); self.ax.set_ylim(-8, 8)
        self.canvas.draw_idle()

    def start_thread(self):
        self.lbl_status.config(text="Status: Calculating...", fg="orange")
        self.root.config(cursor="watch")
        threading.Thread(target=self.solve, daemon=True).start()

    def solve(self):
        engine = PlanningEngine(self.data['start'], self.data['goal'], self.data['obs'])
        raw = engine.solve_rrt_star() if self.algo_choice.get() == "rrtstar" else engine.solve_multibias()
        
        if raw:
            smooth = engine.smooth_path(raw)
            # Ambil semua garis tree sekaligus
            tree_lines = []
            for node in engine.node_list:
                if node["parent"]:
                    tree_lines.append([(node["x"], node["y"]), (node["parent"]["x"], node["parent"]["y"])])
            
            self.root.after(0, self.planning_done, raw, smooth, tree_lines)
        else:
            self.root.after(0, lambda: self.lbl_status.config(text="Status: Failed", fg="red"))

    def planning_done(self, raw, smooth, tree_lines):
        self.root.config(cursor="")
        self.data['raw'], self.data['smooth'] = raw, smooth
        
        # GUNAKAN LINECOLLECTION (Sangat Cepat!)
        lc = LineCollection(tree_lines, colors='#2ecc71', linewidths=0.5, alpha=0.3)
        self.ax.add_collection(lc)

        # Plot Path
        r_np, s_np = np.array(raw), np.array(smooth)
        self.ax.plot(r_np[:,0], r_np[:,1], '--', color='gray', alpha=0.5)
        self.ax.plot(s_np[:,0], s_np[:,1], 'm-o', linewidth=2)
        
        self.canvas.draw_idle()
        self.btn_sim.config(state="normal")
        self.lbl_status.config(text="Status: Path Found", fg="green")

    def fly(self):
        if self.data['smooth']:
            self.sock.sendall(json.dumps({"command": "START_SIM", "path": self.data['smooth']}).encode())

if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1000x750")
    app = DroneApp(root)
    root.mainloop()