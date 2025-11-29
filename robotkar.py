import tkinter as tk
from tkinter import ttk
import math



# --- 1. MATEMATIKAI MODELL ---

class RobotArmModel:
    def __init__(self):
        # Adatok
        self.l3 = 1.2
        self.l4 = 0.7
        self.min_phi32 = 25.0
        self.max_phi32 = 80.0
        self.min_phi43 = 45.0
        self.max_phi43 = 135.0
        self.tgy = 0.5
        self.v = 0.5
        
        self.points = []
        self.calc_rad_limits()

    def calc_rad_limits(self):
        self.rad_min32 = math.radians(self.min_phi32)
        self.rad_max32 = math.radians(self.max_phi32)
        self.rad_min43 = math.radians(self.min_phi43)
        self.rad_max43 = math.radians(self.max_phi43)

    def forward_kinematics(self, phi32, phi43):
        ax, ay = 0, 0
        bx = self.l3 * math.cos(phi32)
        by = self.l3 * math.sin(phi32)
        
        # Könyök jobbra/lefelé hajlik
        abs_angle = phi32 - phi43
        
        cx = bx + self.l4 * math.cos(abs_angle)
        cy = by + self.l4 * math.sin(abs_angle)
        
        return (ax, ay), (bx, by), (cx, cy)

    def inverse_kinematics(self, x, y):
        dist_sq = x**2 + y**2
        dist = math.sqrt(dist_sq)
        
        if dist > (self.l3 + self.l4) or dist < abs(self.l3 - self.l4) or dist == 0:
            return None

        val = (self.l3**2 + self.l4**2 - dist_sq) / (2 * self.l3 * self.l4)
        val = max(-1.0, min(1.0, val))
        
        gamma = math.acos(val)
        phi43 = math.pi - gamma

        beta = math.atan2(y, x)
        
        val_alpha = (self.l3**2 + dist_sq - self.l4**2) / (2 * self.l3 * dist)
        val_alpha = max(-1.0, min(1.0, val_alpha))
        alpha = math.acos(val_alpha)
        
        phi32 = beta + alpha

        eps = 0.001
        if not (self.rad_min32 - eps <= phi32 <= self.rad_max32 + eps):
            return None
        if not (self.rad_min43 - eps <= phi43 <= self.rad_max43 + eps):
            return None

        return phi32, phi43

# --- 2. GRAFIKUS FELÜLET ---

class RobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robotkar Szimuláció")
        self.root.geometry("1280x720")

        self.model = RobotArmModel()
        self.is_moving = False
        self.is_paused = False
        self.anim_queue = []

        # Layout
        self.left_frame = tk.Frame(root, width=320, bg="#f0f0f0", padx=10, pady=10)
        self.left_frame.pack(side="left", fill="y")
        self.left_frame.pack_propagate(False)

        self.right_frame = tk.Frame(root, bg="white")
        self.right_frame.pack(side="right", fill="both", expand=True)

        self.setup_controls()

        self.canvas = tk.Canvas(self.right_frame, bg="white")
        self.canvas.pack(fill="both", expand=True)

        # --- ORIGÓ ÉS SKÁLA BEÁLLÍTÁSA ---
        self.origin_x = 100 
        self.origin_y = 650
        self.scale = 100.0

        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.root.bind("<Configure>", self.on_resize)
        
        self.update_model_from_ui()
        self.draw()

    def setup_controls(self):
        # 1. Változók 
        vf = tk.LabelFrame(self.left_frame, text="Változók", bg="#f0f0f0")
        vf.pack(fill="x", pady=5)
        headers = ["L3(m)", "L4(m)", "φ32min(fok)", "φ32max(fok)", "φ43min(fok)", "φ43max(fok)", "tgy(s)", "v(m/s)"]
        defaults = [self.model.l3, self.model.l4, self.model.min_phi32, self.model.max_phi32, 
                    self.model.min_phi43, self.model.max_phi43, self.model.tgy, self.model.v]
        self.entries = []
        for i, h in enumerate(headers):
            tk.Label(vf, text=h, bg="#f0f0f0").grid(row=i, column=0, sticky="e")
            v = tk.StringVar(value=str(defaults[i]))
            e = tk.Entry(vf, textvariable=v, width=8, state="readonly")
            e.grid(row=i, column=1, padx=5, pady=2)
            self.entries.append(v)
        

        # 2. Adattáblázat
        df = tk.LabelFrame(self.left_frame, text="Adatok", bg="#f0f0f0")
        df.pack(fill="x", pady=10) 
        
        tree_frame = tk.Frame(df)
        tree_frame.pack(fill="x", expand=False)

        cols = ("n", "ti", "zi", "ξi", "φ32", "φ43")
        self.tree = ttk.Treeview(tree_frame, columns=cols, show="headings", height=8)
        
        for col in cols: 
            self.tree.heading(col, text=col)
            w = 50 if col == "Time" else 40
            self.tree.column(col, width=w, anchor="center")
            
        sb = ttk.Scrollbar(tree_frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        
        self.tree.pack(side="left", fill="x", expand=True)
        sb.pack(side="right", fill="y")
        
        tk.Button(df, text="Lista Törlése", command=self.clear_tbl).pack(side="bottom", fill="x", pady=5)

        # 3. Gombok
        bf = tk.Frame(self.left_frame, bg="#f0f0f0")
        bf.pack(side="bottom", fill="x", pady=10)
        tk.Button(bf, text="Pontok Törlése", command=self.del_pts).pack(fill="x")
        sub = tk.Frame(bf, bg="#f0f0f0")
        sub.pack(fill="x")
        tk.Button(sub, text="Start", command=self.start, bg="#cfc").grid(row=0, column=0)
        tk.Button(sub, text="Stop", command=self.stop, bg="#fcc").grid(row=0, column=1)
        tk.Label(sub, text="n:", bg="#f0f0f0").grid(row=1, column=0)
        self.div_var = tk.StringVar(value="12")
        tk.Entry(sub, textvariable=self.div_var, width=5).grid(row=1, column=1)

    def update_model_from_ui(self, e=None):
        # Ez a függvény most már csak inicializáláskor fut le, 
        # mivel a mezők readonly-k és a gomb nincs ott.
        try:
            v = [float(x.get()) for x in self.entries]
            self.model.l3, self.model.l4 = v[0], v[1]
            self.model.min_phi32, self.model.max_phi32 = v[2], v[3]
            self.model.min_phi43, self.model.max_phi43 = v[4], v[5]
            self.model.tgy, self.model.v = v[6], v[7]
            self.model.calc_rad_limits()
            self.draw()
        except: pass

    def on_canvas_click(self, e):
        if self.is_moving: return
        wx = (e.x - self.origin_x) / self.scale
        wy = (self.origin_y - e.y) / self.scale
        
        res = self.model.inverse_kinematics(wx, wy)
        if res:
            self.model.points.append((wx, wy, res[0], res[1]))
            self.draw()
        else:
            print("Hiba: A pont kívül esik a munkaterületen!")

    def to_scr(self, x, y):
        return self.origin_x + x*self.scale, self.origin_y - y*self.scale

    def draw(self):
        self.canvas.delete("all")
        bx, by = self.to_scr(0, 0)

        # --- 1. KOORDINÁTA RENDSZER (0-5) ---
        self.canvas.create_line(bx, by, bx + 6 * self.scale, by, fill="black", arrow="last")
        self.canvas.create_line(bx, by, bx, by - 6 * self.scale, fill="black", arrow="last")
        
        for i in range(1, 6):
            sx = bx + i * self.scale
            self.canvas.create_line(sx, by - 5, sx, by + 5, fill="black") 
            self.canvas.create_text(sx, by + 15, text=str(i))

            sy = by - i * self.scale
            self.canvas.create_line(bx - 5, sy, bx + 5, sy, fill="black") 
            self.canvas.create_text(bx - 15, sy, text=str(i))
            
        self.canvas.create_text(bx - 10, by + 10,)

        # --- 2. MUNKATERÜLET ---
        pts = []
        step = 2
        for d in range(int(self.model.min_phi32), int(self.model.max_phi32)+1, step):
            r32 = math.radians(d)
            _, _, p = self.model.forward_kinematics(r32, self.model.rad_min43)
            sc = self.to_scr(*p)
            pts.extend(sc)
        for d in range(int(self.model.min_phi43), int(self.model.max_phi43)+1, step):
            r43 = math.radians(d)
            _, _, p = self.model.forward_kinematics(self.model.rad_max32, r43)
            sc = self.to_scr(*p)
            pts.extend(sc)
        for d in range(int(self.model.max_phi32), int(self.model.min_phi32)-1, -step):
            r32 = math.radians(d)
            _, _, p = self.model.forward_kinematics(r32, self.model.rad_max43)
            sc = self.to_scr(*p)
            pts.extend(sc)
        for d in range(int(self.model.max_phi43), int(self.model.min_phi43)-1, -step):
            r43 = math.radians(d)
            _, _, p = self.model.forward_kinematics(self.model.rad_min32, r43)
            sc = self.to_scr(*p)
            pts.extend(sc)
        
        self.canvas.create_polygon(pts, fill="#9BCEEC", outline="black", dash=(5, 2))

        # --- Végpontok (A, B, C, D) ---
        # A pont (0.61, 0.61)
        _, _, pA_end = self.model.forward_kinematics(self.model.rad_max32, self.model.rad_max43)
        sa = self.to_scr(*pA_end)
        self.canvas.create_text(sa[0]-15, sa[1], text="A", font=("bold"))
        
        # B pont (0.78, 1.58)
        _, _, pB_end = self.model.forward_kinematics(self.model.rad_max32, self.model.rad_min43)
        sb = self.to_scr(*pB_end)
        self.canvas.create_text(sb[0], sb[1]-15, text="B", font=("bold"))
        
        # C pont (1.75, 0.27)
        _, _, pC_end = self.model.forward_kinematics(self.model.rad_min32, self.model.rad_min43)
        sc = self.to_scr(*pC_end)
        self.canvas.create_text(sc[0]+15, sc[1], text="C", font=("bold"))
        
        # D pont (0.85, -0.15)
        _, _, pD_end = self.model.forward_kinematics(self.model.rad_min32, self.model.rad_max43)
        sd = self.to_scr(*pD_end)
        self.canvas.create_text(sd[0], sd[1]+15, text="D", font=("bold"))

        # --- 3. KAR RAJZOLÁSA ---
        p32, p43 = self.model.rad_min32, self.model.rad_min43
        if self.model.points: p32, p43 = self.model.points[-1][2], self.model.points[-1][3]
        if hasattr(self, 'anim_st'): p32, p43 = self.anim_st

        _, pB, pC = self.model.forward_kinematics(p32, p43)
        sA = self.to_scr(0,0)
        sB = self.to_scr(*pB)
        sC = self.to_scr(*pC)

        self.canvas.create_line(sA[0], sA[1], sB[0], sB[1], width=4, fill="gray")
        self.canvas.create_text((sA[0]+sB[0])/2, (sA[1]+sB[1])/2 - 15, text="L3", font=("bold"))
        self.canvas.create_line(sB[0], sB[1], sC[0], sC[1], width=3, fill="black")
        self.canvas.create_text((sB[0]+sC[0])/2 + 15, (sB[1]+sC[1])/2 - 15, text="L4", font=("bold"))

        self.canvas.create_oval(sB[0]-3, sB[1]-3, sB[0]+3, sB[1]+3, fill="white", outline="black")
        self.canvas.create_oval(sC[0]-4, sC[1]-4, sC[0]+4, sC[1]+4, fill="red", outline="black")

        d32 = math.degrees(p32)
        self.draw_cs(sB[0], sB[1], d32)

        d43 = math.degrees(p43)
        self.canvas.create_arc(sA[0]-25, sA[1]-25, sA[0]+25, sA[1]+25, start=0, extent=d32, style="arc", outline="blue")
        self.canvas.create_text(sA[0]+35, sA[1]-10, text=f"{d32:.1f}°", fill="blue")
        
        start_angle_b = d32
        self.canvas.create_arc(sB[0]-25, sB[1]-25, sB[0]+25, sB[1]+25, start=start_angle_b, extent=-d43, style="arc", outline="green")
        self.canvas.create_text(sB[0]+20, sB[1]+20, text=f"{d43:.1f}°", fill="green")

        for i, p in enumerate(self.model.points):
            sc = self.to_scr(p[0], p[1])
            self.canvas.create_oval(sc[0]-3, sc[1]-3, sc[0]+3, sc[1]+3, fill="black")
            if i > 0:
                prev = self.to_scr(self.model.points[i-1][0], self.model.points[i-1][1])
                self.canvas.create_line(prev[0], prev[1], sc[0], sc[1], dash=(2,2), fill="gray")

    def draw_cs(self, ox, oy, ang):
        rad = math.radians(ang)
        l = 30
        x2 = ox + l * math.cos(rad)
        y2 = oy - l * math.sin(rad)
        self.canvas.create_line(ox, oy, x2, y2, fill="red", arrow="last")
        
        rad2 = rad - math.pi/2 
        x3 = ox + l * math.cos(rad2)
        y3 = oy - l * math.sin(rad2)
        self.canvas.create_line(ox, oy, x3, y3, fill="green", arrow="last")

    def start(self):
        if len(self.model.points) < 2: return
        self.clear_tbl()
        self.anim_q = []
        try: d = int(self.div_var.get())
        except: d=10
        
        # --- IDŐ SZÁMÍTÁS ---
        for i in range(len(self.model.points)-1):
            p1, p2 = self.model.points[i], self.model.points[i+1]
            
            dist_x = p2[0] - p1[0]
            dist_y = p2[1] - p1[1]
            total_dist = math.sqrt(dist_x**2 + dist_y**2)

            v_max = self.model.v
            t_acc = self.model.tgy
            acc = v_max / t_acc if t_acc > 0 else v_max
            d_acc = 0.5 * acc * (t_acc**2)
            is_triangle = total_dist < 2 * d_acc

            for j in range(d+1):
                t_ratio = j/d
                curr32 = p1[2] + (p2[2]-p1[2])*t_ratio
                curr43 = p1[3] + (p2[3]-p1[3])*t_ratio
                
                current_dist = total_dist * t_ratio
                time_at_step = 0.0
                
                if is_triangle:
                    t_peak = math.sqrt(total_dist / acc)
                    if current_dist <= total_dist / 2:
                        time_at_step = math.sqrt(2 * current_dist / acc)
                    else:
                        rem_dist = total_dist - current_dist
                        time_at_step = (2 * t_peak) - math.sqrt(2 * rem_dist / acc)
                else:
                    d_const = total_dist - 2 * d_acc
                    if current_dist <= d_acc:
                        time_at_step = math.sqrt(2 * current_dist / acc)
                    elif current_dist <= (d_acc + d_const):
                        time_at_step = t_acc + (current_dist - d_acc) / v_max
                    else:
                        rem_dist = total_dist - current_dist
                        total_segment_time = (2 * t_acc) + (d_const / v_max)
                        time_at_step = total_segment_time - math.sqrt(2 * rem_dist / acc)

                # Minden szakasz 0-ról indul az időben
                final_time = time_at_step 
                self.anim_q.append((curr32, curr43, j, final_time))

        self.is_moving = True; self.run_anim()
    
    def run_anim(self):
        if not self.is_moving or not self.anim_q: 
            self.is_moving=False
            if hasattr(self,'anim_st'): del self.anim_st
            self.draw()
            return
        
        data = self.anim_q.pop(0)
        
        self.anim_st = (data[0], data[1])
        _, _, pos = self.model.forward_kinematics(data[0], data[1])
        
        self.tree.insert("","end",values=(
            data[2],
            f"{data[3]:.2f} s",
            f"{pos[0]:.2f}",
            f"{pos[1]:.2f}",
            f"{math.degrees(data[0]):.1f}",
            f"{math.degrees(data[1]):.1f}"
        ))
        self.tree.yview_moveto(1)
        self.draw()
        self.root.after(50, self.run_anim)
    
    def stop(self): self.is_moving=False
    def del_pts(self): self.model.points=[]; self.draw()
    def clear_tbl(self): 
        for i in self.tree.get_children(): self.tree.delete(i)
    def on_resize(self,e): self.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotApp(root)
    root.mainloop()