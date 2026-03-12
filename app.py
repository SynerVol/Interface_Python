import collections
import collections.abc
collections.MutableMapping = collections.abc.MutableMapping
collections.Iterable = collections.abc.Iterable

import customtkinter as ctk
import tkintermapview
from dronekit import connect, Command, LocationGlobalRelative, VehicleMode
from pymavlink import mavutil
import math
import threading
import time

# --- CONFIGURATION ---
CONNECTION_STRING = '/dev/ttyUSB0' 
BAUD_RATE = 57600

class RealDroneBackend:
    def __init__(self):
        self.vehicle = None
        self.connected = False
        self.mission_center = None 

    def connect_drone(self, status_callback):
        try:
            status_callback(f"Connexion {CONNECTION_STRING}...")
            try:
                self.vehicle = connect(CONNECTION_STRING, wait_ready=False, baud=BAUD_RATE, heartbeat_timeout=15)
            except Exception as e:
                if self.vehicle: print(f"Warning: {e}")
                else: raise e

            status_callback("Attente Heartbeat...")
            self.vehicle.wait_heartbeat()
            self.connected = True
            
            # Désactivation sécurités pour test (A remettre pour prod)
            self.vehicle.parameters['ARMING_CHECK'] = 0
            
            status_callback(f"✅ CONNECTÉ")
            return True
        except Exception as e:
            if self.vehicle and self.vehicle.battery:
                self.connected = True
                status_callback("✅ CONNECTÉ (Force)")
                return True
            status_callback(f"Erreur: {e}")
            return False

    def get_telemetry(self):
        data = {'lat': None, 'lon': None, 'batt': 'N/A', 'sats': '0', 'mode': 'N/A', 'alt': '0.0', 'armed': False}
        if self.vehicle:
            try:
                data['armed'] = self.vehicle.armed
                if self.vehicle.location.global_relative_frame.lat:
                    data['lat'] = self.vehicle.location.global_relative_frame.lat
                    data['lon'] = self.vehicle.location.global_relative_frame.lon
                    data['alt'] = f"{self.vehicle.location.global_relative_frame.alt:.1f}"
                if self.vehicle.battery:
                    data['batt'] = f"{self.vehicle.battery.voltage}V"
                if self.vehicle.gps_0:
                    data['sats'] = f"{self.vehicle.gps_0.satellites_visible}"
                if self.vehicle.mode:
                    data['mode'] = self.vehicle.mode.name
            except: pass
        return data

    # --- MATHS & MISSION ---
    def get_location_metres(self, original_location, dNorth, dEast, alt):
        earth_radius = 6378137.0 
        dLat = dNorth/earth_radius * 180/math.pi
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180)) * 180/math.pi
        return LocationGlobalRelative(original_location.lat + dLat, original_location.lon + dLon, alt)

    def upload_scan_mission(self, center_lat, center_lon, radius, altitude, fov=60, overlap=0.2):
        if not self.vehicle: return None
        
        print("Calcul de la trajectoire Boustrophédon...")
        cmds = self.vehicle.commands
        cmds.clear()
        
        # Liste pour l'affichage sur la carte (Frontend)
        path_points = []
        
        # WP1: Takeoff
        cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, altitude))
        
        # Calcul Espacement
        largeur_vue = 2 * altitude * math.tan(math.radians(fov / 2))
        spacing = largeur_vue * (1 - overlap)
        
        center_obj = type('obj', (object,), {'lat': center_lat, 'lon': center_lon, 'alt': altitude})
        y = -radius 
        direction = 1 
        
        while y <= radius:
            if abs(y) >= radius:
                y += spacing
                continue
            x_span = math.sqrt(radius**2 - y**2)
            
            p1 = self.get_location_metres(center_obj, y, -x_span if direction == 1 else x_span, altitude)
            p2 = self.get_location_metres(center_obj, y, x_span if direction == 1 else -x_span, altitude)
            
            # Ajout commande drone
            cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, p1.lat, p1.lon, altitude))
            cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, p2.lat, p2.lon, altitude))
            
            # Ajout pour affichage
            path_points.append((p1.lat, p1.lon))
            path_points.append((p2.lat, p2.lon))
            
            y += spacing
            direction *= -1
            
        # WP Final: RTL
        cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        
        print("Envoi au drone...")
        cmds.upload()
        return path_points # On retourne la liste des points pour l'interface

    def start_auto_mission(self):
        if self.vehicle:
            self.vehicle.mode = VehicleMode("GUIDED")
            self.vehicle.armed = True
            time.sleep(2)
            if self.vehicle.armed:
                self.vehicle.simple_takeoff(5)
                time.sleep(5)
                self.vehicle.mode = VehicleMode("AUTO")

    # --- ACTIONS MANUELLES ---
    def set_arm(self):
        if self.vehicle:
            self.vehicle.mode = VehicleMode("STABILIZE")
            self.vehicle.armed = True
    
    def set_disarm(self):
        if self.vehicle: self.vehicle.armed = False

    def set_land(self):
        if self.vehicle:
            print("Commande: LAND (Atterrissage immédiat)")
            self.vehicle.mode = VehicleMode("LAND")

class RealMissionApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.backend = RealDroneBackend()
        
        self.title("SYNERVOL - Mission Réelle")
        self.geometry("1200x800")
        
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # --- SIDEBAR ---
        self.frame_left = ctk.CTkFrame(self, width=280, corner_radius=0)
        self.frame_left.grid(row=0, column=0, sticky="nswe")

        self.lbl_title = ctk.CTkLabel(self.frame_left, text="DRONE CONTROL", font=ctk.CTkFont(size=20, weight="bold"))
        self.lbl_title.grid(row=0, column=0, padx=20, pady=(20, 10))

        # Connexion
        self.btn_connect = ctk.CTkButton(self.frame_left, text="CONNECTER", command=self.on_connect, fg_color="#107C10")
        self.btn_connect.grid(row=1, column=0, padx=20, pady=10)
        self.lbl_status = ctk.CTkLabel(self.frame_left, text="Déconnecté", text_color="gray")
        self.lbl_status.grid(row=2, column=0, padx=20, pady=(0, 10))

        # Infos Rapides
        self.frame_infos = ctk.CTkFrame(self.frame_left)
        self.frame_infos.grid(row=3, column=0, padx=10, pady=5, sticky="ew")
        self.lbl_batt = ctk.CTkLabel(self.frame_infos, text="Batt: N/A")
        self.lbl_batt.pack(pady=5)
        self.lbl_sats = ctk.CTkLabel(self.frame_infos, text="Sats: 0")
        self.lbl_sats.pack(pady=5)

        # --- SECTION MISSION ---
        self.lbl_mis = ctk.CTkLabel(self.frame_left, text="--- MISSION AUTO ---", text_color="orange", font=ctk.CTkFont(weight="bold"))
        self.lbl_mis.grid(row=4, column=0, padx=20, pady=(20, 5))

        # Inputs avec Labels au dessus
        self.lbl_rad = ctk.CTkLabel(self.frame_left, text="Rayon de recherche (m) :")
        self.lbl_rad.grid(row=5, column=0, padx=20, pady=(5,0), sticky="w")
        self.entry_radius = ctk.CTkEntry(self.frame_left)
        self.entry_radius.grid(row=6, column=0, padx=20, pady=(0,5))
        self.entry_radius.insert(0, "50")

        self.lbl_alt = ctk.CTkLabel(self.frame_left, text="Altitude de vol (m) :")
        self.lbl_alt.grid(row=7, column=0, padx=20, pady=(5,0), sticky="w")
        self.entry_alt = ctk.CTkEntry(self.frame_left)
        self.entry_alt.grid(row=8, column=0, padx=20, pady=(0,5))
        self.entry_alt.insert(0, "15")

        self.btn_upload = ctk.CTkButton(self.frame_left, text="1. GÉNÉRER & VOIR", command=self.on_upload, state="disabled", fg_color="#005FB8")
        self.btn_upload.grid(row=9, column=0, padx=20, pady=(15, 5))

        self.btn_start_auto = ctk.CTkButton(self.frame_left, text="2. DÉCOLLER (AUTO)", command=self.on_start_auto, state="disabled", fg_color="#E0a800", text_color="black")
        self.btn_start_auto.grid(row=10, column=0, padx=20, pady=5)

        # --- SECTION MANUELLE ---
        self.lbl_man = ctk.CTkLabel(self.frame_left, text="--- MANUEL ---", text_color="gray")
        self.lbl_man.grid(row=11, column=0, padx=20, pady=(20, 5))
        
        # Boutons d'urgence
        self.frame_manual = ctk.CTkFrame(self.frame_left, fg_color="transparent")
        self.frame_manual.grid(row=12, column=0, padx=10, pady=5)
        
        self.btn_land = ctk.CTkButton(self.frame_manual, text="LAND", command=self.on_land, width=100, fg_color="#555")
        self.btn_land.pack(side="left", padx=5)
        
        self.btn_disarm = ctk.CTkButton(self.frame_manual, text="DISARM", command=self.on_disarm, width=100, fg_color="#C42B1C")
        self.btn_disarm.pack(side="right", padx=5)

        # --- MAP ---
        self.map_widget = tkintermapview.TkinterMapView(self, corner_radius=0)
        self.map_widget.grid(row=0, column=1, sticky="nswe")
        self.map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
        self.map_widget.set_position(48.8566, 2.3522)
        self.map_widget.set_zoom(18)
        self.map_widget.add_right_click_menu_command(label="Définir Zone ici", command=self.set_target, pass_coords=True)

        # Bouton Recentrer sur la Map
        self.btn_recenter = ctk.CTkButton(self.map_widget, text="🎯", width=40, height=40, command=self.action_recenter, fg_color="#333", font=("Arial", 20))
        self.btn_recenter.place(relx=0.95, rely=0.05, anchor="ne")

        # Variables graphiques
        self.drone_marker = None
        self.target_marker = None
        self.zone_circle = None
        self.planned_path = None # Ligne Jaune
        self.traveled_path = None # Ligne Verte
        self.traveled_coords = [] # Historique des points

        self.update_loop()

    # --- LOGIQUE ---
    def on_connect(self):
        self.btn_connect.configure(state="disabled", text="Connexion...")
        threading.Thread(target=self.thread_connect).start()

    def thread_connect(self):
        def cb(msg): self.lbl_status.configure(text=msg)
        if self.backend.connect_drone(cb):
            self.btn_connect.configure(fg_color="green", text="CONNECTÉ")
            self.btn_upload.configure(state="normal")
            self.btn_disarm.configure(state="normal")
        else:
            self.btn_connect.configure(state="normal", fg_color="red", text="RÉESSAYER")

    def set_target(self, coords):
        self.backend.mission_center = coords
        if self.target_marker: self.target_marker.delete()
        self.target_marker = self.map_widget.set_marker(coords[0], coords[1], text="Cible")
        
        # Nettoyage ancienne mission
        if self.planned_path: self.planned_path.delete()
        
        # Dessin cercle rouge
        if self.zone_circle: self.zone_circle.delete()
        try:
            r = float(self.entry_radius.get())
            points = []
            for angle in range(0, 361, 10):
                dN = r * math.cos(math.radians(angle))
                dE = r * math.sin(math.radians(angle))
                lat_offset = dN / 111111
                lon_offset = dE / (111111 * math.cos(math.radians(coords[0])))
                points.append((coords[0] + lat_offset, coords[1] + lon_offset))
            self.zone_circle = self.map_widget.set_polygon(points, fill_color="red", outline_color="red", border_width=2)
        except: pass

    def on_upload(self):
        if not self.backend.mission_center:
            print("Erreur: Définissez une zone (Clic Droit)")
            return
        
        r = float(self.entry_radius.get())
        alt = float(self.entry_alt.get())
        
        # Le backend renvoie maintenant les points de la mission
        waypoints = self.backend.upload_scan_mission(self.backend.mission_center[0], self.backend.mission_center[1], r, alt)
        
        if waypoints:
            self.btn_start_auto.configure(state="normal")
            self.lbl_status.configure(text="Mission Prête !")
            
            # Affichage du tracé prévisionnel (JAUNE)
            if self.planned_path: self.planned_path.delete()
            self.planned_path = self.map_widget.set_path(waypoints, color="yellow", width=2)

    def on_start_auto(self):
        # Reset du tracé réel
        if self.traveled_path: self.traveled_path.delete()
        self.traveled_coords = []
        threading.Thread(target=self.backend.start_auto_mission).start()

    def on_land(self): threading.Thread(target=self.backend.set_land).start()
    def on_disarm(self): threading.Thread(target=self.backend.set_disarm).start()

    def action_recenter(self):
        data = self.backend.get_telemetry()
        if data['lat']:
            self.map_widget.set_position(data['lat'], data['lon'])
            self.map_widget.set_zoom(19)
        elif self.backend.mission_center:
            self.map_widget.set_position(self.backend.mission_center[0], self.backend.mission_center[1])
            self.map_widget.set_zoom(18)

    def update_loop(self):
        data = self.backend.get_telemetry()
        self.lbl_batt.configure(text=f"Batt: {data['batt']}")
        self.lbl_sats.configure(text=f"Sats: {data['sats']}")
        
        if data['lat']:
            # Marker Drone
            if self.drone_marker: self.drone_marker.set_position(data['lat'], data['lon'])
            else: 
                self.drone_marker = self.map_widget.set_marker(data['lat'], data['lon'], text="Drone")
                self.map_widget.set_position(data['lat'], data['lon']) # Centrage au premier fix
            
            # Tracé Vert (Réel)
            self.traveled_coords.append((data['lat'], data['lon']))
            # On ne redessine pas à chaque frame pour économiser, mais tous les X points
            if len(self.traveled_coords) > 1:
                if self.traveled_path: self.traveled_path.delete()
                self.traveled_path = self.map_widget.set_path(self.traveled_coords, color="#00FF00", width=3)
        
        self.after(500, self.update_loop)

if __name__ == "__main__":
    app = RealMissionApp()
    app.mainloop()
