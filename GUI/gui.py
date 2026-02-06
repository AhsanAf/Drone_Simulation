# ==========================================================
# FIX PYTHON PATH (AGAR IMPORT BACKEND TIDAK ERROR)
# ==========================================================
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

# ==========================================================
# IMPORT
# ==========================================================
import math
import tkinter as tk
import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from Backend.map_client import request_map

# ==========================================================
# GUI CONFIG
# ==========================================================
CANVAS_W = 900
CANVAS_H = 600

GRID_SIZE = 25          # ukuran grid (px)
WALL_MIN_THICK = 16     # ketebalan minimum obstacle
PADDING = 60            # margin canvas

# ==========================================================
# GUI CLASS
# ==========================================================
class DronePlannerGUI:

    def __init__(self, root):
        self.root = root
        self.root.title("Drone Path Planning GUI")
        self.root.geometry("1200x700")

        main = ttk.Frame(root)
        main.pack(fill=BOTH, expand=True)

        # ================= LEFT PANEL =================
        left = ttk.Frame(main, width=300, padding=20)
        left.pack(side=LEFT, fill=Y)
        left.pack_propagate(False)

        ttk.Label(
            left, text="Drone Planner",
            font=("Segoe UI", 18, "bold")
        ).pack(pady=10)

        ttk.Button(
            left,
            text="Start Mapping",
            bootstyle=SUCCESS,
            command=self.start_mapping
        ).pack(fill=X, pady=10)

        ttk.Label(
            left, text="World Coordinates",
            font=("Segoe UI", 12, "bold")
        ).pack(anchor=W, pady=(10, 5))

        self.text = tk.Text(
            left, height=32,
            font=("Consolas", 9)
        )
        self.text.pack(fill=BOTH, expand=True)

        # ================= RIGHT PANEL =================
        right = ttk.Frame(main, padding=10)
        right.pack(side=RIGHT, fill=BOTH, expand=True)

        self.canvas = tk.Canvas(
            right,
            bg="white",
            width=CANVAS_W,
            height=CANVAS_H
        )
        self.canvas.pack(fill=BOTH, expand=True)

    # ======================================================
    # GRID
    # ======================================================
    def draw_grid(self):
        for x in range(0, CANVAS_W, GRID_SIZE):
            self.canvas.create_line(x, 0, x, CANVAS_H, fill="#eeeeee")
        for y in range(0, CANVAS_H, GRID_SIZE):
            self.canvas.create_line(0, y, CANVAS_W, y, fill="#eeeeee")

    # ======================================================
    # START MAPPING
    # ======================================================
    def start_mapping(self):
        self.canvas.delete("all")
        self.text.delete("1.0", tk.END)

        self.draw_grid()

        world = request_map()

        # ===== TEXT INFO =====
        self.text.insert(tk.END, "OBSTACLES:\n")
        for i, o in enumerate(world["obstacles"], 1):
            self.text.insert(
                tk.END,
                f"{i}. x={o['x']:.2f}, y={o['y']:.2f}, "
                f"w={o['width']}, h={o['height']}, "
                f"yaw={o['yaw']:.2f}\n"
            )

        self.text.insert(
            tk.END,
            f"\nSTART: x={world['start']['x']:.2f}, "
            f"y={world['start']['y']:.2f}\n"
        )
        self.text.insert(
            tk.END,
            f"GOAL : x={world['goal']['x']:.2f}, "
            f"y={world['goal']['y']:.2f}\n"
        )

        self.draw_world(world)

    # ======================================================
    # DRAW ROTATED RECTANGLE
    # ======================================================
    def draw_rotated_rect(self, cx, cy, w, h, yaw):
        hw, hh = w / 2, h / 2

        corners = [
            (-hw, -hh),
            ( hw, -hh),
            ( hw,  hh),
            (-hw,  hh)
        ]

        points = []
        for x, y in corners:
            rx = x * math.cos(yaw) - y * math.sin(yaw)
            ry = x * math.sin(yaw) + y * math.cos(yaw)
            points.append(cx + rx)
            points.append(cy + ry)

        self.canvas.create_polygon(points, fill="black", outline="")

    # ======================================================
    # DRAW WORLD (TOP-DOWN X–Y)
    # ======================================================
    def draw_world(self, world):
        xs = [o["x"] for o in world["obstacles"]] + \
             [world["start"]["x"], world["goal"]["x"]]
        ys = [o["y"] for o in world["obstacles"]] + \
             [world["start"]["y"], world["goal"]["y"]]

        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)

        scale = min(
            (CANVAS_W - 2 * PADDING) / (maxx - minx + 1e-6),
            (CANVAS_H - 2 * PADDING) / (maxy - miny + 1e-6)
        )

        def wx(x):
            return PADDING + (x - minx) * scale

        def wy(y):
            # BALIK Y (TOP-DOWN)
            return CANVAS_H - (PADDING + (y - miny) * scale)

        def snap(v):
            return round(v / GRID_SIZE) * GRID_SIZE

        # ===== OBSTACLES =====
        for o in world["obstacles"]:
            cx = snap(wx(o["x"]))
            cy = snap(wy(o["y"]))

            w = max(o["width"] * scale, WALL_MIN_THICK)
            h = max(o["height"] * scale, WALL_MIN_THICK)

            self.draw_rotated_rect(
                cx, cy,
                w, h,
                o["yaw"]
            )

        # ===== START =====
        sx = snap(wx(world["start"]["x"]))
        sy = snap(wy(world["start"]["y"]))

        self.canvas.create_oval(
            sx - 7, sy - 7,
            sx + 7, sy + 7,
            fill="blue"
        )
        self.canvas.create_text(sx, sy - 14, text="START")

        # ===== GOAL =====
        gx = snap(wx(world["goal"]["x"]))
        gy = snap(wy(world["goal"]["y"]))

        self.canvas.create_oval(
            gx - 7, gy - 7,
            gx + 7, gy + 7,
            fill="red"
        )
        self.canvas.create_text(gx, gy - 14, text="GOAL")


# ==========================================================
# MAIN
# ==========================================================
if __name__ == "__main__":
    app = DronePlannerGUI(ttk.Window(themename="flatly"))
    app.root.mainloop()
