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

    def inverse_kinematics(self, x, y, check_limits=True):
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

        if check_limits:
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
        self.root.title("Robotkar Szimuláció - Rakodás")
        self.root.geometry("1280x768") 

        self.model = RobotArmModel()
        self.is_moving = False
        self.is_paused = False
        self.anim_queue = []
        self.mouse_ik_angles = None 
        
        # Állapotjelző a rakodáshoz
        self.is_holding_object = False 

        # Gyűjtő koordinátái 
        self.bin_x = 1.2
        self.bin_y = -0.6

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
        self.origin_y = 550 
        self.scale = 100.0

        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<Motion>", self.on_mouse_move) 
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
        
        # 2. Üzemmód Kapcsoló
        mf = tk.LabelFrame(self.left_frame, text="Üzemmód", bg="#f0f0f0")
        mf.pack(fill="x", pady=10)
        
        self.pick_place_var = tk.BooleanVar(value=False)
        cb = tk.Checkbutton(mf, text="Rakodás Mód", 
                            variable=self.pick_place_var, bg="#f0f0f0", 
                            font=("Arial", 10, "bold"), command=self.draw)
        cb.pack(padx=10, pady=5, anchor="w")


        # 3. Adattáblázat
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

        # 4. Gombok
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
        try:
            v = [float(x.get()) for x in self.entries]
            self.model.l3, self.model.l4 = v[0], v[1]
            self.model.min_phi32, self.model.max_phi32 = v[2], v[3]
            self.model.min_phi43, self.model.max_phi43 = v[4], v[5]
            self.model.tgy, self.model.v = v[6], v[7]
            self.model.calc_rad_limits()
            self.draw()
        except: pass

    def on_mouse_move(self, e):
        if self.is_moving: return
        wx = (e.x - self.origin_x) / self.scale
        wy = (self.origin_y - e.y) / self.scale
        res = self.model.inverse_kinematics(wx, wy)
        if res: self.mouse_ik_angles = res
        else: self.mouse_ik_angles = None
        self.draw()

    def on_canvas_click(self, e):
        if self.is_moving: return
        wx = (e.x - self.origin_x) / self.scale
        wy = (self.origin_y - e.y) / self.scale
        res = self.model.inverse_kinematics(wx, wy)
        if res:
            self.model.points.append((wx, wy, res[0], res[1]))
            self.mouse_ik_angles = None 
            self.draw()
        else:
            print("Hiba: A pont kívül esik a munkaterületen!")

    def to_scr(self, x, y):
        return self.origin_x + x*self.scale, self.origin_y - y*self.scale

    def draw(self):
        self.canvas.delete("all")
        bx, by = self.to_scr(0, 0)
        sA = self.to_scr(0,0)

        # --- 1. HÁTTÉR ELEMEK ---
        # Gyűjtőterület (Drop Zone)
        if self.pick_place_var.get():
            bin_w_scr, bin_h_scr = 60, 40
            bin_cx, bin_cy = self.to_scr(self.bin_x, self.bin_y)
            self.canvas.create_rectangle(bin_cx - bin_w_scr, bin_cy, 
                                         bin_cx + bin_w_scr, bin_cy + bin_h_scr,
                                         fill="#DDDDDD", outline="#888888", width=2)
            self.canvas.create_rectangle(bin_cx - bin_w_scr + 5, bin_cy + 5, 
                                         bin_cx + bin_w_scr - 5, bin_cy + bin_h_scr - 5,
                                         fill="#BBBBBB", outline="")
            self.canvas.create_text(bin_cx, bin_cy + bin_h_scr/2, text="GYŰJTŐ", 
                                    fill="#555555", font=("Arial", 9, "bold"))


        # --- 2. KOORDINÁTA RENDSZER ---
        grid_color = "#E0E0E0"
        axis_color = "#444444"
        max_coord = 7 
        axis_length = 6 * self.scale
        
        for i in range(1, max_coord):
            sx = bx + i * self.scale
            self.canvas.create_line(sx, 0, sx, self.canvas.winfo_height(), fill=grid_color) 
            sy = by - i * self.scale
            self.canvas.create_line(0, sy, self.canvas.winfo_width(), sy, fill=grid_color) 
            sy_neg = by + i * self.scale
            self.canvas.create_line(0, sy_neg, self.canvas.winfo_width(), sy_neg, fill=grid_color)

        self.canvas.create_line(bx, by, bx + axis_length, by, fill=axis_color, arrow="last", width=1.5)
        self.canvas.create_line(bx, by, bx, 0, fill=axis_color, arrow="last", width=1.5) 
        
        # Skála
        for i in range(1, 6):
            sx = bx + i * self.scale
            self.canvas.create_line(sx, by - 5, sx, by + 5, fill=axis_color) 
            self.canvas.create_text(sx, by + 15, text=str(i), font=("Arial", 9))
            sy = by - i * self.scale
            self.canvas.create_line(bx - 5, sy, bx + 5, sy, fill=axis_color) 
            self.canvas.create_text(bx - 15, sy, text=str(i), font=("Arial", 9))
            
        self.canvas.create_text(bx + axis_length + 20, by, text="X (m)", font=("Arial", 10, "bold"))
        self.canvas.create_text(bx, 20, text="Y (m)", font=("Arial", 10, "bold"))
        self.canvas.create_text(bx - 10, by + 10, text="O", font=("Arial", 10))

        # --- 3. TALAPZAT ---
        pedestal_w, pedestal_h = 20, 7 
        self.canvas.create_rectangle(sA[0] - pedestal_w, sA[1] - pedestal_h, 
                                     sA[0] + pedestal_w, sA[1] + pedestal_h, 
                                     fill="#666666", outline="#333333")
        self.canvas.create_polygon([sA[0] - pedestal_w, sA[1] - pedestal_h, 
                                    sA[0], sA[1] - pedestal_h - 10, 
                                    sA[0] + pedestal_w, sA[1] - pedestal_h],
                                    fill="#444444", outline="#333333")


        # --- 4. MUNKATERÜLET ---
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
        
        self.canvas.create_polygon(pts, fill="#EFEFEF", outline="gray", dash=(5, 2))

        # --- Végpontok (A, B, C, D) ---
        _, _, pA_end = self.model.forward_kinematics(self.model.rad_max32, self.model.rad_max43)
        sa = self.to_scr(*pA_end)
        self.canvas.create_text(sa[0]-15, sa[1], text="A", font=("Arial", 11, "bold"))
        _, _, pB_end = self.model.forward_kinematics(self.model.rad_max32, self.model.rad_min43)
        sb = self.to_scr(*pB_end)
        self.canvas.create_text(sb[0], sb[1]-15, text="B", font=("Arial", 11, "bold"))
        _, _, pC_end = self.model.forward_kinematics(self.model.rad_min32, self.model.rad_min43)
        sc = self.to_scr(*pC_end)
        self.canvas.create_text(sc[0]+15, sc[1], text="C", font=("Arial", 11, "bold"))
        _, _, pD_end = self.model.forward_kinematics(self.model.rad_min32, self.model.rad_max43)
        sd = self.to_scr(*pD_end)
        self.canvas.create_text(sd[0], sd[1]+15, text="D", font=("Arial", 11, "bold"))

        # --- 5. ROBOTKAR ---
        target_p32, target_p43 = self.model.rad_min32, self.model.rad_min43
        is_tracking = False
        
        if self.mouse_ik_angles and not self.is_moving:
            target_p32, target_p43 = self.mouse_ik_angles
            is_tracking = True
        elif self.model.points and not self.is_moving: 
            target_p32, target_p43 = self.model.points[-1][2], self.model.points[-1][3]
        elif hasattr(self, 'anim_st') and self.is_moving:
            target_p32, target_p43 = self.anim_st

        _, pB, pC = self.model.forward_kinematics(target_p32, target_p43)
        sB = self.to_scr(*pB)
        sC = self.to_scr(*pC)

        fill_L3 = "#C0C0C0" 
        fill_L4 = "#A9A9A9" 
        L3_highlight = "#FFFFFF"
        L4_highlight = "#D3D3D3"
        kar_vastagsag = 8
        shadow_offset = 4
        
        if is_tracking:
            fill_L3 = "#AAAAAA" 
            fill_L4 = "#888888" 
            dash_style = (2, 4) 
            kar_vastagsag = 5
            shadow_offset = 2
        else:
            dash_style = () 

        # Árnyék
        shadow_color = "#AAAAAA"
        self.canvas.create_line(sA[0] + shadow_offset, sA[1] + shadow_offset, 
                                sB[0] + shadow_offset, sB[1] + shadow_offset, 
                                width=kar_vastagsag, fill=shadow_color, 
                                capstyle=tk.ROUND, dash=dash_style)
        self.canvas.create_line(sB[0] + shadow_offset, sB[1] + shadow_offset, 
                                sC[0] + shadow_offset, sC[1] + shadow_offset, 
                                width=kar_vastagsag - 2, fill=shadow_color, 
                                capstyle=tk.ROUND, dash=dash_style)

        # Karok
        self.canvas.create_line(sA[0], sA[1], sB[0], sB[1], 
                                width=kar_vastagsag, fill=fill_L3, 
                                capstyle=tk.ROUND, dash=dash_style)
        if not is_tracking:
            self.canvas.create_line(sA[0]-2, sA[1]-2, sB[0]-2, sB[1]-2,
                                    width=kar_vastagsag/3, fill=L3_highlight, capstyle=tk.ROUND)
        self.canvas.create_text((sA[0]+sB[0])/2, (sA[1]+sB[1])/2 - 15, text="L3", font=("Arial", 10, "bold"))
                                
        self.canvas.create_line(sB[0], sB[1], sC[0], sC[1], 
                                width=kar_vastagsag - 2, fill=fill_L4, 
                                capstyle=tk.ROUND, dash=dash_style)
        if not is_tracking:
            self.canvas.create_line(sB[0]-2, sB[1]-2, sC[0]-2, sC[1]-2,
                                    width=(kar_vastagsag-2)/3, fill=L4_highlight, capstyle=tk.ROUND)
        self.canvas.create_text((sB[0]+sC[0])/2 + 15, (sB[1]+sC[1])/2 - 15, text="L4", font=("Arial", 10, "bold"))

        # Ízületek
        joint_size = 6
        end_effector_size = 8
        self.canvas.create_oval(sA[0]-joint_size, sA[1]-joint_size, sA[0]+joint_size, sA[1]+joint_size, fill="#696969", outline="black")
        self.canvas.create_oval(sA[0]-joint_size+1, sA[1]-joint_size+1, sA[0]+joint_size-3, sA[1]+joint_size-3, fill="#D3D3D3", outline="") 
        self.canvas.create_oval(sB[0]-joint_size, sB[1]-joint_size, sB[0]+joint_size, sB[1]+joint_size, fill="#888888", outline="black")
        self.canvas.create_oval(sB[0]-joint_size+1, sB[1]-joint_size+1, sB[0]+joint_size-3, sB[1]+joint_size-3, fill="#EEEEEE", outline="") 

        # Végpont
        self.canvas.create_oval(sC[0]-end_effector_size, sC[1]-end_effector_size, sC[0]+end_effector_size, sC[1]+end_effector_size, fill="red", outline="black")
        self.canvas.create_oval(sC[0]-end_effector_size+2, sC[1]-end_effector_size+2, sC[0]+end_effector_size-4, sC[1]+end_effector_size-4, fill="#FFCCCC", outline="")
        
        if self.is_holding_object:
            box_sz = 12
            self.canvas.create_rectangle(sC[0]-box_sz, sC[1]-box_sz, sC[0]+box_sz, sC[1]+box_sz, 
                                         fill="#FFFF00", outline="black", width=2)

        # Szögek
        d32 = math.degrees(target_p32)
        self.draw_cs(sB[0], sB[1], d32)
        d43 = math.degrees(target_p43)
        self.canvas.create_arc(sA[0]-25, sA[1]-25, sA[0]+25, sA[1]+25, start=0, extent=d32, style="arc", outline="#FF0000", width=2)
        self.canvas.create_text(sA[0]+35, sA[1]-10, text=f"{d32:.1f}°", fill="#FF0000", font=("Arial", 10, "bold"))
        self.canvas.create_arc(sB[0]-25, sB[1]-25, sB[0]+25, sB[1]+25, start=d32, extent=-d43, style="arc", outline="#FF0000", width=2)
        self.canvas.create_text(sB[0]+20, sB[1]+20, text=f"{d43:.1f}°", fill="#FF0000", font=("Arial", 10, "bold"))

        # Pontok
        for i, p in enumerate(self.model.points):
            sc = self.to_scr(p[0], p[1])
            self.canvas.create_oval(sc[0]-4, sc[1]-4, sc[0]+4, sc[1]+4, fill="#FFA500", outline="black")
            self.canvas.create_text(sc[0] + 10, sc[1] - 10, text=str(i+1), font=("Arial", 8, "bold"), fill="#FFA500")
            if i > 0:
                prev = self.to_scr(self.model.points[i-1][0], self.model.points[i-1][1])
                self.canvas.create_line(prev[0], prev[1], sc[0], sc[1], width=2, fill="gray")

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

    # --- ÚTVONAL GENERÁLÁS ---
    def generate_path_segment(self, start_angles, end_angles, steps, holding_state):
        # Távolság és időbecslés
        p1 = self.model.forward_kinematics(start_angles[0], start_angles[1])[2]
        p2 = self.model.forward_kinematics(end_angles[0], end_angles[1])[2]
        dist = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
        
        velocity = self.model.v if self.model.v > 0 else 0.5
        total_duration = dist / velocity if dist > 0 else 1.0 
        dt = total_duration / steps if steps > 0 else 0

        segment = []
        for j in range(steps + 1):
            t_ratio = j / steps
            curr32 = start_angles[0] + (end_angles[0] - start_angles[0]) * t_ratio
            curr43 = start_angles[1] + (end_angles[1] - start_angles[1]) * t_ratio
            
            current_n = j
            current_t = j * dt
            segment.append((curr32, curr43, current_n, current_t, holding_state, True))
            
        return segment

    def start(self):
        if len(self.model.points) < 1: return
        self.clear_tbl()
        self.anim_q = []
        try: d = int(self.div_var.get())
        except: d=10
        
        current_angles = (self.model.rad_min32, self.model.rad_min43)
        if hasattr(self, 'anim_st'): current_angles = self.anim_st
        
        # --- RAKODÁS MÓD LOGIKA ---
        if self.pick_place_var.get():
            bin_angles = self.model.inverse_kinematics(self.bin_x, self.bin_y, check_limits=False)
            if not bin_angles:
                print("Hiba: A gyűjtő fizikailag nem elérhető!")
                return

            for i, p in enumerate(self.model.points):
                target_angles = (p[2], p[3])
                
                # 1. Mozgás a ponthoz 
                seg = self.generate_path_segment(current_angles, target_angles, d, False)
                self.anim_q.extend(seg)
                
                # 2. Várakozás a ponton
                for _ in range(5): 
                    last = seg[-1]
                    # False flag 
                    self.anim_q.append((last[0], last[1], last[2], last[3], True, False))

                # 3. Mozgás a gyűjtőhöz 
                seg = self.generate_path_segment(target_angles, bin_angles, d, True)
                self.anim_q.extend(seg)
                
                # 4. Várakozás a gyűjtőnél 
                for _ in range(5):
                    last = seg[-1]
                    self.anim_q.append((last[0], last[1], last[2], last[3], False, False))

                current_angles = bin_angles
                
        # --- NORMÁL MÓD LOGIKA ---
        else:
            if len(self.model.points) < 2: return
            for i in range(len(self.model.points)-1):
                p1, p2 = self.model.points[i], self.model.points[i+1]
                seg = self.generate_path_segment((p1[2], p1[3]), (p2[2], p2[3]), d, False)
                self.anim_q.extend(seg)

        self.is_moving = True
        self.run_anim()
    
    def run_anim(self):
        if not self.is_moving or not self.anim_q: 
            self.is_moving=False
            self.is_holding_object = False 
            if hasattr(self,'anim_st'): del self.anim_st
            self.draw()
            self.canvas.delete("current_data") 
            return
        
        data = self.anim_q.pop(0)
        
        self.anim_st = (data[0], data[1])
        self.is_holding_object = data[4]
        should_write = data[5] # Táblázat flag
        
        _, _, pos = self.model.forward_kinematics(data[0], data[1])
        
        # Adatok beírása az n és ti oszlopba
        if should_write:
            self.tree.insert("","end",values=(
                f"{data[2]}",        
                f"{data[3]:.2f} s",  
                f"{pos[0]:.2f}",
                f"{pos[1]:.2f}",
                f"{math.degrees(data[0]):.1f}",
                f"{math.degrees(data[1]):.1f}"
            ))
            self.tree.yview_moveto(1)
            
        self.draw()
        
        sC = self.to_scr(*pos)
        current_data = f"X: {pos[0]:.2f} m\nY: {pos[1]:.2f} m\nT: {data[3]:.2f} s"
        self.canvas.create_text(sC[0] + 50, sC[1] - 20, 
                                text=current_data, 
                                fill="red", font=("Arial", 10, "bold"), 
                                anchor="w", tags="current_data") 
        
        self.root.after(30, self.run_anim) 
    
    def stop(self): self.is_moving=False
    def del_pts(self): self.model.points=[]; self.draw()
    def clear_tbl(self): 
        for i in self.tree.get_children(): self.tree.delete(i)
    def on_resize(self,e): self.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotApp(root)
    root.mainloop()