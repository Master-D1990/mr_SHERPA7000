#!/usr/bin/env python3
"""
MT6701 Hall-Sensor Kalibrierungstool
------------------------------------
Tool zum Kalibrieren und Verwalten mehrerer MT6701 Hall-Sensoren.
Speichert Offset-Werte und Namen für jeden Sensor in einer Konfigurationsdatei.
"""

import os
import json
import math
import time
import argparse
from mt6701_driver import MT6701, scan_i2c_devices

# Standardpfad für die Konfigurationsdatei
DEFAULT_CONFIG_PATH = os.path.expanduser("~/.mt6701_config.json")

class MT6701Calibrated(MT6701):
    """Erweiterte MT6701-Klasse mit Kalibrierung."""
    
    def __init__(self, bus=1, address=0x06, name="sensor1", debug_mode=False, config_file=DEFAULT_CONFIG_PATH):
        """
        Initialisiert einen kalibrierten MT6701 Hall-Sensor.
        
        Args:
            bus (int): I2C-Bus Nummer (Standardwert: 1)
            address (int): I2C-Adresse des Sensors (Standardwert: 0x06)
            name (str): Name für den Sensor
            debug_mode (bool): Debug-Modus aktivieren
            config_file (str): Pfad zur Konfigurationsdatei
        """
        super().__init__(bus=bus, address=address, name=name, debug_mode=debug_mode)
        self.config_file = config_file
        self.offset_rad = 0.0
        self.load_calibration()
    
    def load_calibration(self):
        """Lädt die Kalibrierungsdaten aus der Konfigurationsdatei."""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                
                # Suche nach diesem Sensor (anhand von Bus und Adresse)
                sensor_id = f"{self.bus_num}_{self.address:02x}"
                if sensor_id in config:
                    sensor_config = config[sensor_id]
                    self.name = sensor_config.get('name', self.name)
                    self.offset_rad = float(sensor_config.get('offset_rad', 0.0))
                    
                    if self.debug_mode:
                        print(f"Loaded calibration for '{self.name}': offset = {self.offset_rad:.4f} rad")
        except Exception as e:
            print(f"Error loading calibration: {e}")
    
    def get_angle(self):
        """
        Berechnet den kalibrierten Winkel in Radiant (0-2π).
        
        Returns:
            float: Kalibrierter Winkel in Radiant oder None bei Fehler
        """
        raw_angle = super().get_angle()
        if raw_angle is None:
            return None
        
        # Wende Offset an und normalisiere auf 0-2π
        calibrated_angle = (raw_angle - self.offset_rad) % (2.0 * math.pi)
        return calibrated_angle
    
    def calibrate_to_zero(self):
        """
        Kalibriert den aktuellen Winkel als Nullpunkt.
        
        Returns:
            bool: True bei erfolgreicher Kalibrierung, False bei Fehler
        """
        raw_angle = super().get_angle()
        if raw_angle is None:
            print("Fehler: Konnte keinen Winkel vom Sensor lesen.")
            return False
        
        # Speichere den aktuellen Winkel als Offset
        self.offset_rad = raw_angle
        
        # Speichere die Kalibrierung
        return self.save_calibration()
    
    def save_calibration(self):
        """
        Speichert die Kalibrierungsdaten in der Konfigurationsdatei.
        
        Returns:
            bool: True bei erfolgreicher Speicherung, False bei Fehler
        """
        try:
            # Lade bestehende Konfiguration oder erstelle neue
            config = {}
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
            
            # Aktualisiere die Konfiguration für diesen Sensor
            sensor_id = f"{self.bus_num}_{self.address:02x}"
            config[sensor_id] = {
                'name': self.name,
                'offset_rad': self.offset_rad,
                'bus': self.bus_num,
                'address': self.address,
                'last_calibration': time.strftime("%Y-%m-%d %H:%M:%S")
            }
            
            # Speichere die Konfiguration
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=2)
            
            print(f"Kalibrierung für '{self.name}' gespeichert. Offset: {self.offset_rad:.4f} rad")
            return True
            
        except Exception as e:
            print(f"Fehler beim Speichern der Kalibrierung: {e}")
            return False
    
    def set_name(self, new_name):
        """
        Setzt einen neuen Namen für den Sensor und speichert ihn.
        
        Args:
            new_name (str): Neuer Name für den Sensor
            
        Returns:
            bool: True bei erfolgreicher Umbenennung, False bei Fehler
        """
        if not new_name:
            print("Fehler: Name darf nicht leer sein.")
            return False
        
        self.name = new_name
        return self.save_calibration()

def list_calibrated_sensors(config_file=DEFAULT_CONFIG_PATH):
    """
    Listet alle kalibrierten Sensoren auf.
    
    Args:
        config_file (str): Pfad zur Konfigurationsdatei
        
    Returns:
        dict: Dictionary mit Sensor-Konfigurationen
    """
    if not os.path.exists(config_file):
        print("Keine kalibrierten Sensoren gefunden.")
        return {}
    
    try:
        with open(config_file, 'r') as f:
            config = json.load(f)
        
        print("\nKalibrierte Sensoren:")
        print("---------------------")
        for sensor_id, sensor_config in config.items():
            bus = sensor_config.get('bus', 'unbekannt')
            address = sensor_config.get('address', 'unbekannt')
            name = sensor_config.get('name', 'unbenannt')
            offset = sensor_config.get('offset_rad', 0.0)
            last_cal = sensor_config.get('last_calibration', 'unbekannt')
            
            print(f"ID: {sensor_id}, Name: {name}, Bus: {bus}, Adresse: 0x{address:02x}, "
                  f"Offset: {offset:.4f} rad, Letzte Kalibrierung: {last_cal}")
        
        return config
        
    except Exception as e:
        print(f"Fehler beim Lesen der Konfigurationsdatei: {e}")
        return {}

def continuous_monitoring(sensor, display_mode='rad'):
    """
    Kontinuierliche Überwachung eines kalibrierten Sensors.
    
    Args:
        sensor (MT6701Calibrated): Kalibrierter Sensor
        display_mode (str): Anzeigemodus ('rad', 'deg', 'both')
    """
    try:
        print(f"\nKontinuierliche Überwachung von '{sensor.name}' (Strg+C zum Beenden)")
        print("-----------------------------------------------------")
        
        while True:
            angle_rad = sensor.get_angle()
            if angle_rad is not None:
                angle_deg = math.degrees(angle_rad)
                
                if display_mode == 'rad':
                    print(f"\rWinkel: {angle_rad:.4f} rad      ", end='')
                elif display_mode == 'deg':
                    print(f"\rWinkel: {angle_deg:.2f}°      ", end='')
                else:  # 'both'
                    print(f"\rWinkel: {angle_rad:.4f} rad ({angle_deg:.2f}°)      ", end='')
            else:
                print("\rFehler beim Lesen des Sensors!      ", end='')
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nÜberwachung beendet.")

def main():
    parser = argparse.ArgumentParser(description='MT6701 Hall-Sensor Kalibrierungstool')
    parser.add_argument('--list', action='store_true', help='Listet alle kalibrierten Sensoren auf')
    parser.add_argument('--scan', action='store_true', help='Scannt den I2C-Bus nach Sensoren')
    parser.add_argument('--calibrate', action='store_true', help='Kalibriert einen Sensor auf Null')
    parser.add_argument('--monitor', action='store_true', help='Überwacht kontinuierlich einen Sensor')
    parser.add_argument('--name', type=str, help='Setzt den Namen für einen Sensor')
    parser.add_argument('--bus', type=int, default=1, help='I2C-Bus Nummer (Standard: 1)')
    parser.add_argument('--address', type=str, default='0x06', help='I2C-Adresse des Sensors (Standard: 0x06)')
    parser.add_argument('--display', choices=['rad', 'deg', 'both'], default='both', 
                        help='Anzeigemodus für Monitoring (Standard: both)')
    parser.add_argument('--debug', action='store_true', help='Aktiviert den Debug-Modus')
    
    args = parser.parse_args()
    
    # Konvertiere Hexadezimaladresse zu int
    try:
        if isinstance(args.address, str) and args.address.startswith('0x'):
            address = int(args.address, 16)
        else:
            address = int(args.address)
    except ValueError:
        print(f"Ungültige Adresse: {args.address}")
        return
    
    # Liste alle kalibrierten Sensoren auf
    if args.list:
        list_calibrated_sensors()
        return
    
    # Scanne den I2C-Bus nach Sensoren
    if args.scan:
        scan_i2c_devices(args.bus)
        return
    
    # Erstelle einen kalibrierten Sensor
    sensor = MT6701Calibrated(bus=args.bus, address=address, debug_mode=args.debug)
    
    # Setze einen neuen Namen
    if args.name:
        sensor.set_name(args.name)
    
    # Kalibriere den Sensor auf Null
    if args.calibrate:
        print(f"Kalibriere Sensor '{sensor.name}' auf aktuelle Position als Nullpunkt...")
        sensor.calibrate_to_zero()
    
    # Überwache den Sensor
    if args.monitor:
        continuous_monitoring(sensor, args.display)
    
    # Wenn keine Aktion angegeben wurde
    if not any([args.list, args.scan, args.calibrate, args.monitor, args.name]):
        parser.print_help()

if __name__ == "__main__":
    main()