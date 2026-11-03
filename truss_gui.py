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

def parse_force_component(text: str):
    """Fx / Fy 输入。None（不区分大小写）表示该分量大小未定。"""
    raw = text.strip()
    if raw.lower() == "none":
        return None
    return float(raw)

def format_force_component(value):
    if value is None:
        return "None"
    return f"{value:.4f}"

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

        tk.Label(
            toolbar,
            text="Green: load\nPurple: reaction\nBlue/Red: T/C\n\nFx or Fy = None\nmeans unknown",
            bg='#e8e8e8',
            fg='#333',
            justify=tk.LEFT,
        ).pack(side=tk.BOTTOM, pady=8, padx=6)
        
        # Info Panel (Right)
        self.info_panel = tk.Frame(self.root, width=280, bg='#f4f4f4', relief=tk.RAISED, bd=2)
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

        # Shortcuts bindings. Ignore them while typing in the property fields.
        self.root.bind('<s>', lambda e: self._shortcut('SELECT', e))
        self.root.bind('<j>', lambda e: self._shortcut('JOINT', e))
        self.root.bind('<m>', lambda e: self._shortcut('MEMBER', e))
        self.root.bind('<f>', lambda e: self._shortcut('FORCE', e))

    def _shortcut(self, tool, event):
        if isinstance(event.widget, (tk.Entry, tk.Text)):
            return
        self.set_tool(tool)

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
        self.redraw()

    def show_item(self, item):
        """选中并立刻打开属性栏。不改变当前工具。"""
        self.selected_item = item
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

    def _reaction_marker_length(self):
        return 28.0 / max(self.zoom, 1e-6)

    def force_pick_points(self, joint, force):
        """用于点选的世界坐标。已知/已解出的力用箭头尖端；未定分量用短双向箭头的两端。"""
        jx, jy = joint.pos.x, joint.pos.y
        points = []
        fx, fy = force.effective_x(), force.effective_y()
        if abs(fx) > 1e-12 or abs(fy) > 1e-12:
            points.append((jx + fx, jy + fy))
        length = self._reaction_marker_length()
        if force.fx_prescribed is None and force.fx_solved is None:
            points.extend(((jx + length, jy), (jx - length, jy)))
        if force.fy_prescribed is None and force.fy_solved is None:
            points.extend(((jx, jy + length), (jx, jy - length)))
        return points

    def find_closest_item(self, wx, wy):
        # 1. Check joints
        j = self.find_closest_joint(wx, wy)
        if j: return j
        
        # 2. Check forces (arrow tip, or the ends of an unsolved reaction marker)
        closest_f = None
        min_fd = float('inf')
        for j in self.truss.joints:
            for f in j.forces:
                for mx, my in self.force_pick_points(j, f):
                    d = ((mx - wx)**2 + (my - wy)**2)**0.5
                    if d * self.zoom < 18 and d < min_fd:
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
            self.show_item(new_j)
            
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
                        self.temp_joint = None
                        self.show_item(new_m)
                    else:
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
                self.temp_joint = None
                self.show_item(new_f)
                
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
            tk.Label(self.prop_frame, text="No item selected.\nPlace or click an item\nto edit it here.", fg='gray', bg='#f4f4f4', justify=tk.LEFT).pack(pady=20)
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
                    self.truss.unsolve()
                    self.update_info_panel()
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
                    self.truss.unsolve()
                    self.update_info_panel()
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
            force = self.selected_item
            tk.Label(self.prop_frame, text="FORCE PROPERTIES", font=("Arial", 10, "bold"), bg='#f4f4f4').pack(pady=10)

            kind = "Reaction" if force.is_reaction else "Loading"
            kind_color = "#6d28d9" if force.is_reaction else "#15803d"
            tk.Label(self.prop_frame, text=f"Type: {kind}", fg=kind_color, bg='#f4f4f4', font=("Arial", 9, "bold")).pack()
            tk.Label(
                self.prop_frame,
                text="Type None for an unknown\ncomponent. Any None makes\nthis a reaction.",
                fg='#555',
                bg='#f4f4f4',
                justify=tk.LEFT,
            ).pack(pady=(2, 8))
            
            f_x = tk.Frame(self.prop_frame, bg='#f4f4f4')
            f_x.pack(fill=tk.X, pady=2)
            tk.Label(f_x, text="Fx:", width=8, bg='#f4f4f4').pack(side=tk.LEFT)
            ent_x = tk.Entry(f_x)
            ent_x.pack(side=tk.LEFT, fill=tk.X, expand=True)
            ent_x.insert(0, format_force_component(force.fx_prescribed))
            
            f_y = tk.Frame(self.prop_frame, bg='#f4f4f4')
            f_y.pack(fill=tk.X, pady=2)
            tk.Label(f_y, text="Fy:", width=8, bg='#f4f4f4').pack(side=tk.LEFT)
            ent_y = tk.Entry(f_y)
            ent_y.pack(side=tk.LEFT, fill=tk.X, expand=True)
            ent_y.insert(0, format_force_component(force.fy_prescribed))

            error_lbl = tk.Label(self.prop_frame, text="", fg='red', bg='#f4f4f4')

            def save_force_cartesian():
                try:
                    force.set_value([
                        parse_force_component(ent_x.get()),
                        parse_force_component(ent_y.get()),
                    ])
                    self.truss.unsolve()
                    self.update_info_panel()
                    self.redraw()
                except ValueError:
                    error_lbl.config(text="Enter a number or None")
            tk.Button(self.prop_frame, text="Apply Fx/Fy", command=save_force_cartesian).pack(pady=5)
            error_lbl.pack()

            if force.is_reaction:
                tk.Label(self.prop_frame, text="Solved reaction:", font=("Arial", 9, "bold"), bg='#f4f4f4').pack(pady=(8, 2))

                def solved_text(prescribed, solved):
                    if prescribed is not None:
                        return f"{prescribed:.4f} (specified)"
                    if solved is None:
                        return "unsolved"
                    return f"{solved:.4f}"

                tk.Label(self.prop_frame, text=f"Fx = {solved_text(force.fx_prescribed, force.fx_solved)}", bg='#f4f4f4').pack()
                tk.Label(self.prop_frame, text=f"Fy = {solved_text(force.fy_prescribed, force.fy_solved)}", bg='#f4f4f4').pack()

    def solve_truss(self):
        try:
            self.truss.solve_displacement()
            self.update_info_panel()
            self.redraw()
            messagebox.showinfo("Success", "Truss forces and displacements solved successfully!")
        except Warning as w:
            self.update_info_panel()
            self.redraw()
            messagebox.showwarning("Warning", str(w))
        except Exception as e:
            self.update_info_panel()
            self.redraw()
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
                color = "#7c3aed" if f.is_reaction else "green"
                width = 2
                if f == self.selected_item:
                    color = "orange"
                    width = 4

                fx, fy = f.effective_x(), f.effective_y()
                if abs(fx) > 1e-12 or abs(fy) > 1e-12:
                    x2 = x1 + fx * self.zoom
                    y2 = y1 - fy * self.zoom
                    self.canvas.create_line(x1, y1, x2, y2, fill=color, width=width, arrow=tk.LAST, tags="force")

                length = self._reaction_marker_length()
                if f.fx_prescribed is None and f.fx_solved is None:
                    ax1, ay1 = self.world_to_screen(j.pos.x - length, j.pos.y, j.displacement.x, j.displacement.y)
                    ax2, ay2 = self.world_to_screen(j.pos.x + length, j.pos.y, j.displacement.x, j.displacement.y)
                    self.canvas.create_line(ax1, ay1, ax2, ay2, fill=color, width=width, arrow=tk.BOTH, dash=(4, 2), tags="force")
                if f.fy_prescribed is None and f.fy_solved is None:
                    ax1, ay1 = self.world_to_screen(j.pos.x, j.pos.y - length, j.displacement.x, j.displacement.y)
                    ax2, ay2 = self.world_to_screen(j.pos.x, j.pos.y + length, j.displacement.x, j.displacement.y)
                    self.canvas.create_line(ax1, ay1, ax2, ay2, fill=color, width=width, arrow=tk.BOTH, dash=(4, 2), tags="force")
                
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