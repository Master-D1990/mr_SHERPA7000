#!/usr/bin/env python3
"""
Testprogramm für MT6701 Hall-Sensoren mit Ausgabe in Radiant.
Scannt den I2C-Bus und testet gefundene Sensoren.
"""

import time
import math
import os
from mt6701_driver import MT6701, scan_i2c_devices

def clear_screen():
    """Löscht den Terminal-Bildschirm."""
    os.system('clear' if os.name == 'posix' else 'cls')

def format_radians(rad):
    """Formatiert Radiant-Werte als Vielfaches von π."""
    pi_multiple = rad / math.pi
    if abs(pi_multiple - 2) < 0.01:
        return "2π"
    elif abs(pi_multiple - 1.5) < 0.01:
        return "3π/2"
    elif abs(pi_multiple - 1) < 0.01:
        return "π"
    elif abs(pi_multiple - 0.5) < 0.01:
        return "π/2"
    elif abs(pi_multiple) < 0.01 or abs(pi_multiple - 2) < 0.01:
        return "0"
    else:
        return f"{pi_multiple:.2f}π"

def radians_to_direction(rad):
    """Konvertiert Radiant in eine Himmelsrichtung."""
    directions = ["N", "NNO", "NO", "ONO", "O", "OSO", "SO", "SSO", 
                  "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    idx = int((rad / (2 * math.pi) * 16 + 0.5) % 16)
    return directions[idx]

def test_single_sensor(bus=1, address=0x06, name="sensor1"):
    """
    Testet einen einzelnen Sensor mit angegebener Adresse.
    
    Args:
        bus (int): I2C-Bus Nummer
        address (int): I2C-Adresse des Sensors
        name (str): Name für den Sensor
    """
    print(f"\nTesting sensor '{name}' at address 0x{address:02x}...")
    
    sensor = MT6701(bus=bus, address=address, name=name)
    
    # Einzelmessung
    angle = sensor.get_angle()
    if angle is not None:
        print(f"Current angle: {angle:.4f} rad ({format_radians(angle)})")
        print(f"Angle in degrees: {sensor.get_angle_degrees():.2f}°")
        print(f"Direction: {radians_to_direction(angle)}")
    else:
        print("Failed to read angle!")
    
    sensor.close()

def continuous_test(bus=1, address=0x06, name="sensor1", debug_mode=False):
    """
    Führt einen kontinuierlichen Test durch bis der Benutzer ihn abbricht.
    
    Args:
        bus (int): I2C-Bus Nummer
        address (int): I2C-Adresse des Sensors
        name (str): Name für den Sensor
        debug_mode (bool): Debug-Modus aktivieren
    """
    print(f"\nContinuous test of sensor '{name}'. Press Ctrl+C to stop...")
    
    sensor = MT6701(bus=bus, address=address, name=name, debug_mode=debug_mode)
    
    # Speichere alte und neue Werte für die Berechnung der Änderungsrate
    last_angle = None
    last_time = time.time()
    
    try:
        while True:
            # Lese den aktuellen Winkel
            angle = sensor.get_angle()
            current_time = time.time()
            
            if angle is not None:
                # Berechne die Winkelgeschwindigkeit
                angular_velocity = None
                if last_angle is not None:
                    # Berechne Winkeldifferenz unter Berücksichtigung des 0/2π Übergangs
                    angle_diff = angle - last_angle
                    if angle_diff > math.pi:
                        angle_diff -= 2 * math.pi
                    elif angle_diff < -math.pi:
                        angle_diff += 2 * math.pi
                    
                    # Berechne Winkelgeschwindigkeit in rad/s
                    time_diff = current_time - last_time
                    if time_diff > 0:
                        angular_velocity = angle_diff / time_diff
                
                # Aktualisiere die Werte für die nächste Iteration
                last_angle = angle
                last_time = current_time
                
                # Lösche den Bildschirm für eine saubere Ausgabe
                clear_screen()
                
                # Ausgabe der Winkelinformationen
                print(f"MT6701 Hall-Sensor Test - {name}")
                print(f"================================")
                print(f"Current angle: {angle:.4f} rad ({format_radians(angle)})")
                print(f"Angle in degrees: {sensor.get_angle_degrees():.2f}°")
                print(f"Direction: {radians_to_direction(angle)}")
                
                if angular_velocity is not None:
                    print(f"Angular velocity: {angular_velocity:.4f} rad/s ({angular_velocity * 180 / math.pi:.2f}°/s)")
                
                # Balkenanzeige (0 bis 2π)
                width = 50
                position = int((angle / (2 * math.pi)) * width)
                bar = ['─'] * width
                if 0 <= position < width:
                    bar[position] = '│'
                
                print("\nPosition:")
                print(f"0 {''.join(bar)} 2π")
                
                # Zusätzliche Debug-Informationen
                if debug_mode:
                    raw_angle = sensor.read_raw_angle()
                    print(f"\nRaw angle value: {raw_angle}")
                
                print("\nPress Ctrl+C to stop...")
            else:
                print(f"[{name}] Failed to read angle!")
            
            # Warte etwas, um die CPU zu entlasten
            time.sleep(0.1)  # 10 Hz
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    sensor.close()

def debug_raw_readings(bus=1, address=0x06, name="sensor1"):
    """
    Zeigt die Rohwerte des Sensors an, um Probleme zu diagnostizieren.
    
    Args:
        bus (int): I2C-Bus Nummer
        address (int): I2C-Adresse des Sensors
        name (str): Name für den Sensor
    """
    print(f"\nDebug raw readings from sensor '{name}'...")
    
    sensor = MT6701(bus=bus, address=address, name=name, debug_mode=True)
    bus_obj = sensor.bus
    
    try:
        while True:
            print("\n--- Raw Register Values ---")
            # Lese mehrere Register nacheinander
            for reg in range(0x00, 0x10):
                try:
                    value = bus_obj.read_byte_data(address, reg)
                    print(f"Register 0x{reg:02x}: 0x{value:02x} ({value}) - Binary: {bin(value)[2:].zfill(8)}")
                except Exception as e:
                    print(f"Cannot read register 0x{reg:02x}: {e}")
            
            # Lese Winkel mit aktueller Methode
            raw = sensor.read_raw_angle()
            angle = sensor.get_angle()
            print(f"\nRaw angle value: {raw} -> {angle:.4f} rad ({math.degrees(angle):.2f}°)")
            
            time.sleep(2)
    except KeyboardInterrupt:
        print("\nDebug stopped by user")
    
    sensor.close()

def main():
    print("MT6701 Hall Sensor Test Utility (Radiant Output)")
    print("===============================================")
    
    # Scanne den I2C-Bus nach Geräten
    devices = scan_i2c_devices()
    
    if not devices:
        print("\nNo devices found. Testing with default address (0x06)...")
        test_single_sensor()
    else:
        # Teste jeden gefundenen Sensor
        for i, addr in enumerate(devices):
            test_single_sensor(address=addr, name=f"sensor{i+1}")
    
    # Frage nach dem Test-Modus
    print("\nSelect test mode:")
    print("1. Continuous monitoring")
    print("2. Debug raw register values")
    choice = input("Enter choice (1/2): ").strip()
    
    debug_mode = input("Enable debug output? (y/n): ").lower() == 'y'
    
    # Führe gewählten Test mit dem ersten gefundenen Sensor durch
    addr = devices[0] if devices else 0x06
    if choice == '2':
        debug_raw_readings(address=addr)
    else:
        continuous_test(address=addr, debug_mode=debug_mode)
    
    print("\nTest completed!")

if __name__ == "__main__":
    main()