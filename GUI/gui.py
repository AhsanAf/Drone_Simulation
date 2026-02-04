import tkinter as tk
import ttkbootstrap as ttk
from ttkbootstrap.constants import *


class DronePlannerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Path Planning GUI")
        self.root.geometry("1100x650")
        self.root.minsize(1000, 600)

        # ================= MAIN CONTAINER =================
        main_frame = ttk.Frame(root)
        main_frame.pack(fill=BOTH, expand=True)

        # ================= LEFT CONTROL PANEL =================
        control_frame = ttk.Frame(
            main_frame,
            padding=20,
            width=280
        )
        control_frame.pack(side=LEFT, fill=Y)
        control_frame.pack_propagate(False)

        ttk.Label(
            control_frame,
            text="Drone Planner",
            font=("Segoe UI", 18, "bold")
        ).pack(pady=(0, 30))

        # ---------- Algorithm Selection ----------
        ttk.Label(
            control_frame,
            text="Path Planning Algorithm",
            font=("Segoe UI", 10)
        ).pack(anchor="w", pady=(0, 5))

        self.algorithm_var = ttk.StringVar(value="RRT")

        algo_combo = ttk.Combobox(
            control_frame,
            textvariable=self.algorithm_var,
            values=["RRT", "RRT*"],
            state="readonly"
        )
        algo_combo.pack(fill=X, pady=(0, 20))

        # ---------- Start Mapping ----------
        self.btn_mapping = ttk.Button(
            control_frame,
            text="Start Mapping",
            bootstyle=SUCCESS,
            command=self.start_mapping
        )
        self.btn_mapping.pack(fill=X, pady=6)

        # ---------- Start Simulation ----------
        self.btn_simulation = ttk.Button(
            control_frame,
            text="Start Simulation",
            bootstyle=WARNING,
            command=self.start_simulation
        )
        self.btn_simulation.pack(fill=X, pady=6)

        # ---------- Status ----------
        ttk.Separator(control_frame).pack(fill=X, pady=25)

        self.status_label = ttk.Label(
            control_frame,
            text="Status: Idle",
            foreground="#0078D7",
            font=("Segoe UI", 10)
        )
        self.status_label.pack(anchor="w")

        # ================= RIGHT VISUALIZATION PANEL =================
        visual_frame = ttk.Frame(main_frame)
        visual_frame.pack(side=RIGHT, fill=BOTH, expand=True)

        self.canvas = tk.Canvas(
            visual_frame,
            bg="white",
            highlightthickness=0
        )
        self.canvas.pack(fill=BOTH, expand=True)

        # Draw initial grid
        self.draw_grid()

    # ================= PLACEHOLDER CALLBACKS =================
    def start_mapping(self):
        algorithm = self.algorithm_var.get()
        self.status_label.config(
            text=f"Status: Mapping using {algorithm}"
        )

        # TODO (future):
        # - Request obstacle data via TCP
        # - Request drone & goal position
        # - Run RRT / RRT*
        # - Draw 2D path result

    def start_simulation(self):
        self.status_label.config(
            text="Status: Simulating"
        )

        # TODO (future):
        # - Send path to backend
        # - Trigger Webots execution

    # ================= CANVAS GRID =================
    def draw_grid(self):
        self.canvas.delete("grid")

        width = 2000
        height = 2000
        grid_size = 50

        for x in range(0, width, grid_size):
            self.canvas.create_line(
                x, 0, x, height,
                fill="#e5e5e5",
                tags="grid"
            )

        for y in range(0, height, grid_size):
            self.canvas.create_line(
                0, y, width, y,
                fill="#e5e5e5",
                tags="grid"
            )


# ================= MAIN ENTRY =================
if __name__ == "__main__":
    root = ttk.Window(themename="flatly")
    app = DronePlannerGUI(root)
    root.mainloop()
