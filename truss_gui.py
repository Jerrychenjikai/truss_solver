import tkinter as tk
from tkinter import messagebox
import math
from basic import Truss, Joint, Member, Force, Vector2D

def point_to_segment_dist(px, py, ax, ay, bx, by):
    l2 = (bx - ax)**2 + (by - ay)**2
    if l2 == 0: return ((px - ax)**2 + (py - ay)**2)**0.5
    t = max(0, min(1, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / l2))
    proj_x = ax + t * (bx - ax)
    proj_y = ay + t * (by - ay)
    return ((px - proj_x)**2 + (py - proj_y)**2)**0.5

class TrussGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("2D Truss Visualizer & Solver")
        
        self.truss = Truss()
        
        self.current_tool = 'SELECT'
        self.selected_item = None
        self.temp_joint = None
        
        self.disp_scale = 100.0  
        self.zoom = 10.0
        self.pan_x = 400
        self.pan_y = 300
        
        self.setup_ui()
        self.redraw()
        
    def setup_ui(self):
        # Toolbar (Left)
        toolbar = tk.Frame(self.root, width=120, bg='#e8e8e8', relief=tk.RAISED, bd=2)
        toolbar.pack(side=tk.LEFT, fill=tk.Y)
        
        self.tool_lbl = tk.Label(toolbar, text="Tool: SELECT", bg='#e8e8e8', font=("Arial", 10, "bold"), fg='blue')
        self.tool_lbl.pack(pady=10)
        
        tools = [("Select (S)", 'SELECT'), ("Add Joint (J)", 'JOINT'), 
                 ("Add Member (M)", 'MEMBER'), ("Add Force (F)", 'FORCE')]
        
        for txt, val in tools:
            tk.Button(toolbar, text=txt, command=lambda v=val: self.set_tool(v), width=12).pack(pady=5)
        
        tk.Label(toolbar, text="Disp. Scale:", bg='#e8e8e8').pack(pady=(20,0))
        self.scale_var = tk.StringVar(value=str(self.disp_scale))
        tk.Entry(toolbar, textvariable=self.scale_var, width=10, justify='center').pack()
        tk.Button(toolbar, text="Update Scale", command=self.update_scale).pack(pady=5)

        tk.Button(toolbar, text="SOLVE\nDISPLACEMENT", command=self.solve_truss, bg='#a2fca2', font=("Arial", 9, "bold")).pack(pady=20, fill=tk.X, padx=5)
        
        # Info Panel (Right)
        self.info_panel = tk.Frame(self.root, width=250, bg='#f4f4f4', relief=tk.RAISED, bd=2)
        self.info_panel.pack(side=tk.RIGHT, fill=tk.Y)
        tk.Label(self.info_panel, text="Properties", font=("Arial", 12, "bold"), bg='#f4f4f4').pack(pady=10)
        
        self.prop_frame = tk.Frame(self.info_panel, bg='#f4f4f4')
        self.prop_frame.pack(fill=tk.BOTH, expand=True, padx=10)
        
        # Canvas (Center)
        self.canvas = tk.Canvas(self.root, bg='white')
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<B2-Motion>", self.on_pan)
        self.canvas.bind("<Button-2>", self.start_pan)
        self.canvas.bind("<MouseWheel>", self.on_zoom) 
        self.canvas.bind("<Button-4>", self.on_zoom) 
        self.canvas.bind("<Button-5>", self.on_zoom) 
        
        self.last_pan_x = 0
        self.last_pan_y = 0

        # Shortcuts bindings
        self.root.bind('<s>', lambda e: self.set_tool('SELECT'))
        self.root.bind('<j>', lambda e: self.set_tool('JOINT'))
        self.root.bind('<m>', lambda e: self.set_tool('MEMBER'))
        self.root.bind('<f>', lambda e: self.set_tool('FORCE'))

    def update_scale(self):
        try:
            self.disp_scale = float(self.scale_var.get())
            self.redraw()
        except ValueError:
            pass

    def set_tool(self, tool):
        self.current_tool = tool
        self.temp_joint = None
        self.tool_lbl.config(text=f"Tool: {tool}")
        if tool == 'SELECT':
            self.selected_item = None
            self.update_info_panel()
        self.redraw()
        
    def screen_to_world(self, sx, sy):
        x = (sx - self.pan_x) / self.zoom
        y = (self.pan_y - sy) / self.zoom
        return x, y
        
    def world_to_screen(self, x, y, disp_x=0.0, disp_y=0.0):
        # Includes current displacements (amplified by disp_scale) to visualize deformation
        sx = self.pan_x + (x + disp_x * self.disp_scale) * self.zoom
        sy = self.pan_y - (y + disp_y * self.disp_scale) * self.zoom
        return sx, sy
        
    def find_closest_joint(self, wx, wy, threshold=15):
        closest = None
        min_d = float('inf')
        for j in self.truss.joints:
            d = ((j.pos.x - wx)**2 + (j.pos.y - wy)**2)**0.5
            if d * self.zoom < threshold and d < min_d:
                closest = j
                min_d = d
        return closest

    def find_closest_item(self, wx, wy):
        # 1. Check joints
        j = self.find_closest_joint(wx, wy)
        if j: return j
        
        # 2. Check forces
        closest_f = None
        min_fd = float('inf')
        for j in self.truss.joints:
            for f in j.forces:
                fx, fy = f.value.x, f.value.y
                if fx == 0 and fy == 0: continue
                mx, my = j.pos.x + fx, j.pos.y + fy
                d = ((mx - wx)**2 + (my - wy)**2)**0.5
                if d * self.zoom < 30 and d < min_fd:
                    closest_f = f
                    min_fd = d
        if closest_f: return closest_f

        # 3. Check members
        closest_m = None
        min_md = float('inf')
        for m in self.truss.members:
            j1, j2 = m.joints[0], m.joints[1]
            d = point_to_segment_dist(wx, wy, j1.pos.x, j1.pos.y, j2.pos.x, j2.pos.y)
            if d * self.zoom < 10 and d < min_md:
                closest_m = m
                min_md = d
        return closest_m

    def on_click(self, event):
        wx, wy = self.screen_to_world(event.x, event.y)
        
        if self.current_tool == 'SELECT':
            self.selected_item = self.find_closest_item(wx, wy)
            self.update_info_panel()
            
        elif self.current_tool == 'JOINT':
            new_j = Joint([wx, wy])
            self.truss.add_joint(new_j)
            self.selected_item = new_j
            self.set_tool('SELECT')  
            self.update_info_panel()
            
        elif self.current_tool == 'MEMBER':
            j = self.find_closest_joint(wx, wy)
            if j:
                if self.temp_joint is None:
                    self.temp_joint = j
                else:
                    if self.temp_joint != j:
                        new_m = Member(self.temp_joint, j)
                        self.temp_joint.add_member(new_m)
                        j.add_member(new_m)
                        self.truss.add_member(new_m)
                        self.selected_item = new_m
                        self.set_tool('SELECT')
                        self.update_info_panel()
                    self.temp_joint = None
            else:
                self.temp_joint = None
                
        elif self.current_tool == 'FORCE':
            if self.temp_joint is None:
                j = self.find_closest_joint(wx, wy)
                if j:
                    self.temp_joint = j
            else:
                fx = wx - self.temp_joint.pos.x
                fy = wy - self.temp_joint.pos.y
                new_f = Force(Vector2D(fx, fy))
                self.temp_joint.add_force(new_f)
                self.selected_item = new_f
                self.temp_joint = None
                self.set_tool('SELECT')
                self.update_info_panel()
                
        self.redraw()

    def start_pan(self, event):
        self.last_pan_x = event.x
        self.last_pan_y = event.y

    def on_pan(self, event):
        dx = event.x - self.last_pan_x
        dy = event.y - self.last_pan_y
        self.pan_x += dx
        self.pan_y += dy
        self.last_pan_x = event.x
        self.last_pan_y = event.y
        self.redraw()
        
    def on_zoom(self, event):
        if event.num == 4 or event.delta > 0:
            self.zoom *= 1.1
        elif event.num == 5 or event.delta < 0:
            self.zoom *= 0.9
        self.redraw()

    def update_info_panel(self):
        for widget in self.prop_frame.winfo_children():
            widget.destroy()
            
        if self.selected_item is None:
            tk.Label(self.prop_frame, text="No item selected", fg='gray', bg='#f4f4f4').pack(pady=20)
            return
            
        if isinstance(self.selected_item, Joint):
            tk.Label(self.prop_frame, text="JOINT PROPERTIES", font=("Arial", 10, "bold"), bg='#f4f4f4').pack(pady=10)
            
            f_x = tk.Frame(self.prop_frame, bg='#f4f4f4')
            f_x.pack(fill=tk.X, pady=2)
            tk.Label(f_x, text="X:", width=5, bg='#f4f4f4').pack(side=tk.LEFT)
            ent_x = tk.Entry(f_x)
            ent_x.pack(side=tk.LEFT, fill=tk.X, expand=True)
            ent_x.insert(0, f"{self.selected_item.pos.x:.4f}")
            
            f_y = tk.Frame(self.prop_frame, bg='#f4f4f4')
            f_y.pack(fill=tk.X, pady=2)
            tk.Label(f_y, text="Y:", width=5, bg='#f4f4f4').pack(side=tk.LEFT)
            ent_y = tk.Entry(f_y)
            ent_y.pack(side=tk.LEFT, fill=tk.X, expand=True)
            ent_y.insert(0, f"{self.selected_item.pos.y:.4f}")
            
            def save_joint():
                try:
                    self.selected_item.pos = Vector2D(float(ent_x.get()), float(ent_y.get()))
                    self.redraw()
                except ValueError:
                    pass
            tk.Button(self.prop_frame, text="Apply Changes", command=save_joint).pack(pady=10)

            tk.Label(self.prop_frame, text="Displacements:", font=("Arial", 9, "bold"), bg='#f4f4f4').pack(pady=(10,2))
            tk.Label(self.prop_frame, text=f"dx = {self.selected_item.displacement.x:.4e}", bg='#f4f4f4').pack()
            tk.Label(self.prop_frame, text=f"dy = {self.selected_item.displacement.y:.4e}", bg='#f4f4f4').pack()

        elif isinstance(self.selected_item, Member):
            tk.Label(self.prop_frame, text="MEMBER PROPERTIES", font=("Arial", 10, "bold"), bg='#f4f4f4').pack(pady=10)
            
            f_E = tk.Frame(self.prop_frame, bg='#f4f4f4')
            f_E.pack(fill=tk.X, pady=2)
            tk.Label(f_E, text="E (Modulus):", width=12, bg='#f4f4f4').pack(side=tk.LEFT)
            ent_E = tk.Entry(f_E)
            ent_E.pack(side=tk.LEFT, fill=tk.X, expand=True)
            ent_E.insert(0, str(self.selected_item.E))
            
            f_A = tk.Frame(self.prop_frame, bg='#f4f4f4')
            f_A.pack(fill=tk.X, pady=2)
            tk.Label(f_A, text="A (Area):", width=12, bg='#f4f4f4').pack(side=tk.LEFT)
            ent_A = tk.Entry(f_A)
            ent_A.pack(side=tk.LEFT, fill=tk.X, expand=True)
            ent_A.insert(0, str(self.selected_item.A))
            
            def save_member():
                try:
                    self.selected_item.E = float(ent_E.get())
                    self.selected_item.A = float(ent_A.get())
                    self.redraw()
                except ValueError:
                    pass
            tk.Button(self.prop_frame, text="Apply Changes", command=save_member).pack(pady=10)
            
            if self.selected_item.force is not None:
                tk.Label(self.prop_frame, text="Calculated Force:", font=("Arial", 9, "bold"), bg='#f4f4f4').pack(pady=(10,2))
                f_val = self.selected_item.force
                state = "Tension" if f_val >= 0 else "Compression"
                color = "blue" if f_val >= 0 else "red"
                tk.Label(self.prop_frame, text=f"{abs(f_val):.4f} ({state})", fg=color, bg='#f4f4f4').pack()
                
        elif isinstance(self.selected_item, Force):
            tk.Label(self.prop_frame, text="FORCE PROPERTIES", font=("Arial", 10, "bold"), bg='#f4f4f4').pack(pady=10)
            
            f_x = tk.Frame(self.prop_frame, bg='#f4f4f4')
            f_x.pack(fill=tk.X, pady=2)
            tk.Label(f_x, text="Fx:", width=8, bg='#f4f4f4').pack(side=tk.LEFT)
            ent_x = tk.Entry(f_x)
            ent_x.pack(side=tk.LEFT, fill=tk.X, expand=True)
            ent_x.insert(0, f"{self.selected_item.value.x:.4f}")
            
            f_y = tk.Frame(self.prop_frame, bg='#f4f4f4')
            f_y.pack(fill=tk.X, pady=2)
            tk.Label(f_y, text="Fy:", width=8, bg='#f4f4f4').pack(side=tk.LEFT)
            ent_y = tk.Entry(f_y)
            ent_y.pack(side=tk.LEFT, fill=tk.X, expand=True)
            ent_y.insert(0, f"{self.selected_item.value.y:.4f}")

            def save_force_cartesian():
                try:
                    self.selected_item.set_value([float(ent_x.get()), float(ent_y.get())])
                    self.truss.forces_solved = False
                    self.redraw()
                    self.update_info_panel()
                except ValueError:
                    pass
            tk.Button(self.prop_frame, text="Apply Fx/Fy", command=save_force_cartesian).pack(pady=5)
            
            tk.Frame(self.prop_frame, height=2, bg='gray').pack(fill=tk.X, pady=10)
            
            mag = self.selected_item.value.get_magnitude()
            ang = self.selected_item.value.get_angle_degree()
            
            f_m = tk.Frame(self.prop_frame, bg='#f4f4f4')
            f_m.pack(fill=tk.X, pady=2)
            tk.Label(f_m, text="Length:", width=8, bg='#f4f4f4').pack(side=tk.LEFT)
            ent_m = tk.Entry(f_m)
            ent_m.pack(side=tk.LEFT, fill=tk.X, expand=True)
            ent_m.insert(0, f"{mag:.4f}")
            
            f_a = tk.Frame(self.prop_frame, bg='#f4f4f4')
            f_a.pack(fill=tk.X, pady=2)
            tk.Label(f_a, text="Angle (°):", width=8, bg='#f4f4f4').pack(side=tk.LEFT)
            ent_a = tk.Entry(f_a)
            ent_a.pack(side=tk.LEFT, fill=tk.X, expand=True)
            ent_a.insert(0, f"{ang:.4f}")

            def save_force_polar():
                try:
                    m = float(ent_m.get())
                    a = math.radians(float(ent_a.get()))
                    fx = m * math.cos(a)
                    fy = m * math.sin(a)
                    self.selected_item.set_value([fx, fy])
                    self.truss.forces_solved = False
                    self.redraw()
                    self.update_info_panel()
                except ValueError:
                    pass
            tk.Button(self.prop_frame, text="Apply Length/Angle", command=save_force_polar).pack(pady=5)

    def solve_truss(self):
        try:
            self.truss.solve_displacement()
            self.update_info_panel()
            self.redraw()
            messagebox.showinfo("Success", "Truss forces and displacements solved successfully!")
        except Warning as w:
            messagebox.showwarning("Warning", str(w))
        except Exception as e:
            messagebox.showerror("Solve Error", f"Could not solve truss: {str(e)}")

    def redraw(self):
        self.canvas.delete("all")
        
        # Grid/Axes could be added here
        
        for m in self.truss.members:
            j1, j2 = m.joints[0], m.joints[1]
            x1, y1 = self.world_to_screen(j1.pos.x, j1.pos.y, j1.displacement.x, j1.displacement.y)
            x2, y2 = self.world_to_screen(j2.pos.x, j2.pos.y, j2.displacement.x, j2.displacement.y)
            
            color = "black"
            width = 2
            
            if m == self.selected_item:
                color = "orange"
                width = 4
            elif m.force is not None:
                color = "blue" if m.force > 0 else "red"
                width = 3
                
            self.canvas.create_line(x1, y1, x2, y2, fill=color, width=width, tags="member")
            
        for j in self.truss.joints:
            x1, y1 = self.world_to_screen(j.pos.x, j.pos.y, j.displacement.x, j.displacement.y)
            for f in j.forces:
                fx, fy = f.value.x, f.value.y
                if fx == 0 and fy == 0: continue
                f_scale = 1.0 
                x2 = x1 + fx * self.zoom * f_scale
                y2 = y1 - fy * self.zoom * f_scale
                
                color = "green"
                width = 2
                if f == self.selected_item: 
                    color = "orange"
                    width = 4
                
                self.canvas.create_line(x1, y1, x2, y2, fill=color, width=width, arrow=tk.LAST, tags="force")
                
        for j in self.truss.joints:
            cx, cy = self.world_to_screen(j.pos.x, j.pos.y, j.displacement.x, j.displacement.y)
            r = 6
            
            color = "white"
            if j == self.selected_item: color = "orange"
            elif j == self.temp_joint: color = "yellow"
            
            self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill=color, outline="black", width=2, tags="joint")
            
        if self.current_tool == 'MEMBER' and self.temp_joint is not None:
            cx, cy = self.world_to_screen(self.temp_joint.pos.x, self.temp_joint.pos.y, 
                                          self.temp_joint.displacement.x, self.temp_joint.displacement.y)
            self.canvas.create_oval(cx-8, cy-8, cx+8, cy+8, outline="blue", width=2, dash=(2,2))
            
        if self.current_tool == 'FORCE' and self.temp_joint is not None:
            cx, cy = self.world_to_screen(self.temp_joint.pos.x, self.temp_joint.pos.y, 
                                          self.temp_joint.displacement.x, self.temp_joint.displacement.y)
            self.canvas.create_oval(cx-8, cy-8, cx+8, cy+8, outline="green", width=2, dash=(2,2))
            
if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1100x700")
    app = TrussGUI(root)
    root.mainloop()
