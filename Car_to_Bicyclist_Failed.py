import carla
import time
import math
import pygame
import sys
import os
import socket
import struct
import matplotlib.pyplot as plt
from collections import deque
import threading

def get_speed(vehicle):
    velocity = vehicle.get_velocity()
    return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

def calculate_ttc(distance, relative_velocity):
    """Calcule le Time-To-Collision"""
    if relative_velocity <= 0:
        return float('inf')  # Pas de collision si vitesse relative <= 0
    return distance / relative_velocity

def setup_tcp_server(port=9001):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('localhost', port))
    sock.listen(1)
    print(f"En attente de connexion Simulink sur le port {port}...")
    try:
        connection, addr = sock.accept()
        connection.settimeout(0.05)  # Timeout plus court
        print(f"Connexion Simulink établie depuis {addr}")
        return connection, sock
    except Exception as e:
        print(f"Erreur connexion TCP: {e}")
        sock.close()
        return None, None

def send_data(conn, data):
    try:
        for key in ['MIO_Distance', 'MIO_Velocity', 'Ego_Velocity']:
            conn.sendall(struct.pack('d', float(data[key])))
        print(f"[SEND] {data}")
    except Exception as e:
        print(f"[ERREUR SEND]: {e}")

def receive_data(conn):
    try:
        data = []
        for _ in range(5):
            packet = conn.recv(8)
            if len(packet) < 8:
                raise ValueError("Packet incomplet")
            data.append(struct.unpack('d', packet)[0])
        sim_data = {
            'egoCarStop': bool(round(data[0])),
            'FCW_Activate': bool(round(data[1])),
            'Deceleration': data[2],
            'AEB_Status': bool(round(data[3])),
            'Emergency_Brake': bool(round(data[4]))
        }
        print(f"[RECV] {sim_data}")
        return sim_data
    except socket.timeout:
        # Timeout normal - retourner les dernières valeurs ou valeurs par défaut
        return {
            'egoCarStop': False,
            'FCW_Activate': False,
            'Deceleration': 0.0,
            'AEB_Status': False,
            'Emergency_Brake': False
        }
    except Exception as e:
        print(f"[ERREUR RECV]: {e}")
        return {
            'egoCarStop': False,
            'FCW_Activate': False,
            'Deceleration': 0.0,
            'AEB_Status': False,
            'Emergency_Brake': False
        }

def initialize_carla():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.load_world('Town03')
    time.sleep(1)
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    return client, world

def setup_weather_night_rain(world):
    """Configuration météo : Nuit + Pluie - Compatible CARLA 0.9.10"""
    weather = carla.WeatherParameters(
        cloudiness=90.0,           # Très nuageux
        precipitation=70.0,        # Pluie forte
        precipitation_deposits=95.0, # Flaques importantes
        wind_intensity=50.0,       # Vent fort
        sun_azimuth_angle=0.0,     # Position du soleil
        sun_altitude_angle=-90.0,  # Soleil sous l'horizon = NUIT
        fog_density=30.0,          # Brouillard nocturne
        fog_distance=30.0,         # Distance de visibilité réduite
        fog_falloff=2.0,           # Transition du brouillard
        wetness=90.0               # Route très mouillée
    )
    world.set_weather(weather)
    print("[INFO] Conditions météo appliquées: NUIT + PLUIE FORTE")

def setup_street_lighting(world):
    """Active l'éclairage public nocturne - Compatible CARLA 0.9.10"""
    try:
        # Tentative d'activation de l'éclairage si disponible
        lights = world.get_lightmanager().get_all_lights()
        
        # Allumage de tous les éclairages publics
        lights_count = 0
        for light in lights:
            if hasattr(light, 'is_on') and not light.is_on:
                light.turn_on()
                lights_count += 1
        
        print(f"[INFO] {lights_count} éclairages publics activés")
    except Exception as e:
        print(f"[WARNING] Impossible d'activer l'éclairage public: {e}")
        print("[INFO] Simulation sans éclairage public automatique")

def spawn_actors(world):
    bp_lib = world.get_blueprint_library()

    # Véhicule ego avec phares
    ego_bp = bp_lib.find('vehicle.audi.tt')
    ego_spawn = carla.Transform(carla.Location(x=8.0, y=-80.0, z=0.3), carla.Rotation(yaw=90))

    # Cycliste : plus reculé, avec équipement nocturne
    cyclist_bp = bp_lib.find('vehicle.bh.crossbike')
    cyclist_spawn = carla.Transform(carla.Location(x=8.0, y=-17.0, z=0.3), carla.Rotation(yaw=-87))

    # Nettoyage
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()
    time.sleep(0.5)

    ego = world.spawn_actor(ego_bp, ego_spawn)
    cyclist = world.spawn_actor(cyclist_bp, cyclist_spawn)

    # Activation des phares du véhicule ego - Compatible CARLA 0.9.10
    try:
        lights = carla.VehicleLightState.NONE
        lights |= carla.VehicleLightState.Position
        lights |= carla.VehicleLightState.LowBeam  # Feux de croisement
        # Note: HighBeam peut ne pas être disponible dans 0.9.10
        if hasattr(carla.VehicleLightState, 'HighBeam'):
            lights |= carla.VehicleLightState.HighBeam
        ego.set_light_state(carla.VehicleLightState(lights))
        print("[INFO] Phares du véhicule ego activés")
    except Exception as e:
        print(f"[WARNING] Impossible d'activer les phares: {e}")
    
    print("[INFO] Véhicules spawned avec tentative d'éclairage nocturne")

    return ego, cyclist

def control_cyclist(cyclist, sim_time):
    if sim_time > 2.0:
        cyclist.apply_control(carla.VehicleControl(throttle=0.4, steer=0.0))
    else:
        cyclist.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))

def control_ego(ego, sim_time, sim_data, collision):
    # Adaptation pour conditions nocturnes et pluvieuses
    # Réduction de la vitesse de base en conditions difficiles
    base_throttle = 0.5  # Réduit par rapport aux 0.6 initiaux
    
    if sim_data['Emergency_Brake'] or sim_data['egoCarStop'] or collision:
        return carla.VehicleControl(throttle=0.0, brake=1.0)
    if sim_time > 2.0:
        # Freinage plus progressif en conditions glissantes
        decel_factor = min(0.8, sim_data['Deceleration'])  # Limité pour éviter le blocage
        return carla.VehicleControl(
            throttle=max(0.6, base_throttle - decel_factor),
            brake=min(0.8, decel_factor),  # Freinage moins agressif sur route mouillée
            steer=0.0
        )
    else:
        return carla.VehicleControl(throttle=0.0, brake=1.0)

class RealTimePlotter:
    def __init__(self, max_points=200):
        self.max_points = max_points
        self.times = deque(maxlen=max_points)
        self.distances = deque(maxlen=max_points)
        self.ttc_values = deque(maxlen=max_points)
        self.ego_speeds = deque(maxlen=max_points)
        self.cyclist_speeds = deque(maxlen=max_points)
        
        # Configuration matplotlib pour éviter les conflits de thread
        plt.switch_backend('TkAgg')  # Backend plus stable
        plt.ion()
        
        try:
            self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 8))
            # Thème sombre pour représenter la nuit
            self.fig.patch.set_facecolor('black')
            self.fig.suptitle('Analyse AEB en Temps Réel - Conditions Nocturnes + Pluie', color='white')
            
            # Configuration des axes avec thème sombre
            for ax in [self.ax1, self.ax2, self.ax3]:
                ax.set_facecolor('black')
                ax.tick_params(colors='white')
                ax.xaxis.label.set_color('white')
                ax.yaxis.label.set_color('white')
                ax.title.set_color('white')
                ax.spines['bottom'].set_color('white')
                ax.spines['top'].set_color('white')
                ax.spines['right'].set_color('white')
                ax.spines['left'].set_color('white')
            
            self.ax1.set_ylabel('Distance (m)')
            self.ax1.set_title('Distance Ego-Cycliste (Nuit + Pluie)')
            self.ax1.grid(True, color='gray', alpha=0.3)
            
            self.ax2.set_ylabel('TTC (s)')
            self.ax2.set_title('Time-To-Collision (Visibilité Réduite)')
            self.ax2.grid(True, color='gray', alpha=0.3)
            self.ax2.set_ylim(0, 10)
            
            self.ax3.set_ylabel('Vitesse (m/s)')
            self.ax3.set_xlabel('Temps (s)')
            self.ax3.set_title('Vitesses des Véhicules (Conditions Difficiles)')
            self.ax3.grid(True, color='gray', alpha=0.3)
            
            plt.tight_layout()
            plt.show(block=False)
            self.plotting_enabled = True
        except Exception as e:
            print(f"[WARNING] Impossible d'initialiser matplotlib: {e}")
            print("[INFO] Continuer sans graphiques temps réel")
            self.plotting_enabled = False
    
    def update(self, time_val, distance, ttc, ego_speed, cyclist_speed):
        if not self.plotting_enabled:
            return
            
        try:
            self.times.append(time_val)
            self.distances.append(distance)
            self.ttc_values.append(min(ttc, 10))
            self.ego_speeds.append(ego_speed)
            self.cyclist_speeds.append(cyclist_speed)
            
            # Mise à jour moins fréquente pour éviter les conflits
            if len(self.times) % 5 == 0:  # Mise à jour tous les 5 points
                self._update_plots()
        except Exception as e:
            print(f"[WARNING] Erreur matplotlib: {e}")
            self.plotting_enabled = False
    
    def _update_plots(self):
        try:
            # Couleurs adaptées au thème nocturne
            # Mise à jour des graphiques
            self.ax1.clear()
            self.ax1.set_facecolor('black')
            self.ax1.plot(list(self.times), list(self.distances), color='cyan', linewidth=2, label='Distance')
            self.ax1.axhline(y=2.5, color='red', linestyle='--', label='Seuil collision')
            self.ax1.set_ylabel('Distance (m)', color='white')
            self.ax1.set_title('Distance Ego-Cycliste (Nuit + Pluie)', color='white')
            self.ax1.legend()
            self.ax1.grid(True, color='gray', alpha=0.3)
            self.ax1.tick_params(colors='white')
            
            self.ax2.clear()
            self.ax2.set_facecolor('black')
            self.ax2.plot(list(self.times), list(self.ttc_values), color='yellow', linewidth=2, label='TTC')
            self.ax2.axhline(y=2.0, color='orange', linestyle='--', label='TTC critique (nuit)')
            self.ax2.set_ylabel('TTC (s)', color='white')
            self.ax2.set_title('Time-To-Collision (Visibilité Réduite)', color='white')
            self.ax2.set_ylim(0, 10)
            self.ax2.legend()
            self.ax2.grid(True, color='gray', alpha=0.3)
            self.ax2.tick_params(colors='white')
            
            self.ax3.clear()
            self.ax3.set_facecolor('black')
            self.ax3.plot(list(self.times), list(self.ego_speeds), color='lime', linewidth=2, label='Ego')
            self.ax3.plot(list(self.times), list(self.cyclist_speeds), color='magenta', linewidth=2, label='Cycliste')
            self.ax3.set_ylabel('Vitesse (m/s)', color='white')
            self.ax3.set_xlabel('Temps (s)', color='white')
            self.ax3.set_title('Vitesses des Véhicules (Conditions Difficiles)', color='white')
            self.ax3.legend()
            self.ax3.grid(True, color='gray', alpha=0.3)
            self.ax3.tick_params(colors='white')
            
            plt.draw()
            plt.pause(0.001)  # Pause très courte
        except Exception as e:
            print(f"[WARNING] Erreur mise à jour graphique: {e}")
            self.plotting_enabled = False

def main():
    conn = None
    sock = None
    world = None
    
    try:
        pygame.init()
        screen = pygame.display.set_mode((600, 400))
        pygame.display.set_caption("HUD - Simulation AEB Nocturne + Pluie")
        font = pygame.font.Font(None, 24)
        font_small = pygame.font.Font(None, 18)

        client, world = initialize_carla()
        
        # Configuration météo : Nuit + Pluie
        setup_weather_night_rain(world)
        
        # Activation de l'éclairage public
        setup_street_lighting(world)
        
        ego, cyclist = spawn_actors(world)
        
        # Tentative de connexion TCP (optionnelle)
        conn, sock = setup_tcp_server(9001)
        tcp_active = conn is not None
        
        if not tcp_active:
            print("[WARNING] Simulation sans connexion Simulink")
        
        # Initialisation du traceur temps réel
        plotter = RealTimePlotter()

        clock = pygame.time.Clock()
        sim_time = 0.0
        collision = False
        
        # Données par défaut si pas de connexion Simulink
        # Seuils ajustés pour conditions nocturnes
        default_sim_data = {
            'egoCarStop': False,
            'FCW_Activate': False,
            'Deceleration': 0.0,
            'AEB_Status': False,
            'Emergency_Brake': False
        }

        print("[INFO] Simulation démarrée avec conditions NOCTURNES + PLUIE")

        while True:
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    return

            world.tick()
            sim_time += 0.05

            control_cyclist(cyclist, sim_time)

            distance = ego.get_location().distance(cyclist.get_location())
            ego_speed = get_speed(ego)
            cyclist_speed = get_speed(cyclist)
            
            # Calcul vitesse relative et TTC
            relative_velocity = ego_speed - cyclist_speed
            ttc = calculate_ttc(distance, relative_velocity)

            # Communication TCP (si active)
            if tcp_active and conn:
                try:
                    send_data(conn, {
                        'MIO_Distance': distance,
                        'MIO_Velocity': cyclist_speed,
                        'Ego_Velocity': ego_speed
                    })
                    sim_data = receive_data(conn)
                except Exception as e:
                    print(f"[TCP ERROR]: {e}")
                    tcp_active = False
                    sim_data = default_sim_data
            else:
                # Simulation AEB adaptée aux conditions nocturnes
                sim_data = default_sim_data.copy()
                # Seuils plus conservateurs la nuit
                if distance < 15.0 and relative_velocity > 0:  # Détection plus précoce
                    sim_data['AEB_Status'] = True
                    if distance < 8.0:  # Freinage d'urgence plus tôt
                        sim_data['Emergency_Brake'] = True
                        sim_data['Deceleration'] = min(0.8, 15.0/distance)  # Freinage plus doux sur route mouillée

            ctrl = control_ego(ego, sim_time, sim_data, collision)
            ego.apply_control(ctrl)

            if distance < 2.5 and not collision:
                print("[COLLISION] DETECTED!")
                collision = True

            # Mise à jour du graphique temps réel
            plotter.update(sim_time, distance, ttc, ego_speed, cyclist_speed)

            # HUD adapté au thème nocturne
            screen.fill((10, 10, 30))  # Bleu très sombre
            
            # Titre avec statut connexion
            title_text = "SIMULATION AEB - NUIT + PLUIE"
            if not tcp_active:
                title_text += " (SANS SIMULINK)"
            title = font.render(title_text, True, (255, 255, 100))  # Jaune pour la nuit
            screen.blit(title, (10, 10))
            
            # Indicateur conditions météo
            weather_text = "CONDITIONS: NUIT + PLUIE FORTE"
            weather = font_small.render(weather_text, True, (100, 200, 255))  # Bleu clair
            screen.blit(weather, (10, 35))
            
            hud_lines = [
                f"Vitesse Ego: {ego_speed*3.6:.1f} km/h",
                f"Vitesse Cycliste: {cyclist_speed*3.6:.1f} km/h",
                f"Distance: {distance:.2f} m",
                f"TTC: {ttc:.2f} s" if ttc != float('inf') else "TTC: ∞",
                f"Vitesse Relative: {relative_velocity:.2f} m/s",
                f"Visibilité: RÉDUITE",
                "",
                f"Accélérateur: {ctrl.throttle:.2f}",
                f"Frein: {ctrl.brake:.2f}",
                f"Freinage Urgence: {sim_data['Emergency_Brake']}",
                f"AEB Actif: {sim_data['AEB_Status']}",
                f"Éclairage: ACTIVÉ",
                f"Collision: {collision}",
                f"TCP: {'Actif' if tcp_active else 'Inactif'}"
            ]
            
            for i, text in enumerate(hud_lines):
                if text == "":
                    continue
                color = (200, 200, 200)  # Gris clair par défaut
                if "Collision: True" in text:
                    color = (255, 50, 50)  # Rouge vif
                elif "Freinage Urgence: True" in text or "AEB Actif: True" in text:
                    color = (255, 150, 0)  # Orange
                elif "TTC:" in text and ttc < 3.0 and ttc != float('inf'):  # Seuil augmenté pour la nuit
                    color = (255, 100, 100)  # Rouge clair
                elif "TCP: Inactif" in text:
                    color = (255, 200, 0)  # Jaune
                elif "Visibilité: RÉDUITE" in text:
                    color = (255, 255, 0)  # Jaune vif
                elif "Éclairage: ACTIVÉ" in text:
                    color = (0, 255, 100)  # Vert clair
                
                txt = font_small.render(text, True, color)
                screen.blit(txt, (10, 60 + i * 20))
            
            pygame.display.flip()

            # Caméra spectateur optimisée pour conditions nocturnes
            spectator = world.get_spectator()
            ego_location = ego.get_location()
            ego_rotation = ego.get_transform().rotation
            
            # Vue suiveur rapprochée pour compenser la visibilité réduite
            import math
            yaw_rad = math.radians(ego_rotation.yaw)
            camera_x = ego_location.x - 15 * math.cos(yaw_rad)  # Plus proche (15m au lieu de 20m)
            camera_y = ego_location.y - 15 * math.sin(yaw_rad)
            camera_z = ego_location.z + 8  # Un peu plus bas pour meilleure visibilité
            camera_location = carla.Location(x=camera_x, y=camera_y, z=camera_z)
            camera_rotation = carla.Rotation(pitch=-20, yaw=ego_rotation.yaw, roll=0)
            
            spectator.set_transform(carla.Transform(camera_location, camera_rotation))

            clock.tick(20)

    except KeyboardInterrupt:
        print("\n[INFO] Arrêt demandé par l'utilisateur")
    except Exception as e:
        print(f"Erreur principale : {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Nettoyage...")
        try:
            if conn:
                conn.close()
            if sock:
                sock.close()
            if world:
                for actor in world.get_actors().filter('vehicle.*'):
                    actor.destroy()
        except Exception as e:
            print(f"Erreur nettoyage: {e}")
        
        try:
            pygame.quit()
        except:
            pass
        
        try:
            plt.close('all')
        except:
            pass

if __name__ == "__main__":
    print(">>> Lancement du test AEB vs Cycliste - CONDITIONS NOCTURNES + PLUIE")
    print(">>> Fonctionnalités: Nuit, Pluie forte, Éclairage, Visibilité réduite")
    print(">>> Seuils AEB adaptés aux conditions difficiles")
    main()