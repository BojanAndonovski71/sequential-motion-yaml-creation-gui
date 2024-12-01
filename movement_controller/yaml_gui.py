import tkinter as tk
from tkinter import filedialog
import yaml

class YamlGui:
    def __init__(self, root):
        self.root = root
        self.root.title("YAML Configuration GUI")
        self.movements = []

        tk.Button(root, text="Add Rotate", command=self.add_rotate).pack()
        tk.Button(root, text="Add Straight", command=self.add_straight).pack()
        tk.Button(root, text="Save YAML", command=self.save_yaml).pack()

    def add_rotate(self):
        rotate_window = tk.Toplevel(self.root)
        rotate_window.title("Add Rotate")

        tk.Label(rotate_window, text="Direction:").pack()
        direction = tk.StringVar(value="right")
        tk.OptionMenu(rotate_window, direction, "right", "left").pack()

        tk.Label(rotate_window, text="Speed (deg/s):").pack()
        speed = tk.DoubleVar(value=90.0)
        tk.Entry(rotate_window, textvariable=speed).pack()

        tk.Label(rotate_window, text="Angle (deg):").pack()
        angle = tk.DoubleVar(value=90.0)
        tk.Entry(rotate_window, textvariable=angle).pack()

        def add():
            self.movements.append({
                'moving_type': 'rotate',
                'direction': direction.get(),
                'speed': speed.get(),
                'target': {'angle': angle.get()}
            })
            rotate_window.destroy()

        tk.Button(rotate_window, text="Add", command=add).pack()

    def add_straight(self):
        straight_window = tk.Toplevel(self.root)
        straight_window.title("Add Straight")

        tk.Label(straight_window, text="Direction:").pack()
        direction = tk.StringVar(value="forward")
        tk.OptionMenu(straight_window, direction, "forward", "backward").pack()

        tk.Label(straight_window, text="Speed (m/s):").pack()
        speed = tk.DoubleVar(value=0.3)
        tk.Entry(straight_window, textvariable=speed).pack()

        tk.Label(straight_window, text="Distance (m):").pack()
        distance = tk.DoubleVar(value=1.0)
        tk.Entry(straight_window, textvariable=distance).pack()

        def add():
            self.movements.append({
                'moving_type': 'straight',
                'direction': direction.get(),
                'speed': speed.get(),
                'distance': distance.get()
            })
            straight_window.destroy()

        tk.Button(straight_window, text="Add", command=add).pack()

    def save_yaml(self):
        file_path = filedialog.asksaveasfilename(defaultextension=".yaml", filetypes=[("YAML files", "*.yaml")])
        if file_path:
            with open(file_path, 'w') as file:
                yaml.dump({'unit_moves': self.movements}, file)

def main():
    root = tk.Tk()
    app = YamlGui(root)
    root.mainloop()

if __name__ == "__main__":
    main()
