#!/usr/bin/env python3
"""
MT6701 Hall-Sensor Treiber
--------------------------
Ein objektorientierter Treiber für den MT6701 magnetischen Hall-Sensor
mit korrektem Register-Mapping laut Datenblatt.
"""

import math
import time
import smbus2

class MT6701:
    """
    Treiber für den MT6701 magnetischen Hall-Sensor.
    Ermöglicht die Winkelmessung über I2C.
    """
    
    def __init__(self, bus=1, address=0x06, name="sensor1", debug_mode=False):
        """
        Initialisiert einen MT6701 Hall-Sensor.
        
        Args:
            bus (int): I2C-Bus Nummer (Standardwert: 1)
            address (int): I2C-Adresse des Sensors (Standardwert: 0x06)
            name (str): Name für den Sensor
            debug_mode (bool): Debug-Modus aktivieren (gibt Rohdaten aus)
        """
        self.bus_num = bus
        self.address = address
        self.name = name
        self.bus = None
        self.debug_mode = debug_mode
        
        try:
            self.bus = smbus2.SMBus(self.bus_num)
            print(f"Initialized MT6701 '{self.name}' on bus {self.bus_num}, address 0x{self.address:02x}")
        except Exception as e:
            print(f"Error initializing I2C bus {self.bus_num}: {e}")
    
    def read_raw_angle(self):
        """
        Liest den Rohwert des Winkels vom Sensor.
        
        Laut Datenblatt: Der MT6701 verwendet 12-bit Auflösung für den Winkel.
        Die I2C-Register 0x03 und 0x04 liefern den Winkel mit:
        Reg 0x03: Bits 11:4 (8 bits, MSB)
        Reg 0x04: Bits 3:0 (4 bits, LSB)
        
        Returns:
            int: Rohwert des Winkels (0-4095) oder None bei Fehler
        """
        if self.bus is None:
            print("I2C bus not initialized")
            return None
        
        try:
            # Lese die zwei Winkel-Register
            msb = self.bus.read_byte_data(self.address, 0x03)
            lsb = self.bus.read_byte_data(self.address, 0x04)
            
            # Kombiniere die Werte zum 12-bit Winkel
            # MSB enthält die oberen 8 Bits, LSB enthält die untersten 4 Bits (die oberen 4 Bits im Register)
            raw_angle = (msb << 4) | ((lsb & 0xF0) >> 4)
            
            if self.debug_mode:
                print(f"MSB (0x03): 0x{msb:02x}, LSB (0x04): 0x{lsb:02x}")
                print(f"Raw angle: {raw_angle} (0x{raw_angle:03x}), 12-bit value")
            
            return raw_angle
            
        except Exception as e:
            print(f"Error reading from I2C: {e}")
            return None
    
    def get_angle(self):
        """
        Berechnet den Winkel in Radiant (0-2π).
        
        Returns:
            float: Winkel in Radiant oder None bei Fehler
        """
        raw = self.read_raw_angle()
        if raw is None:
            return None
        
        # Umrechnung von 12-bit Wert (0-4095) zu Radiant (0-2π)
        angle_rad = (raw * 2.0 * math.pi) / 4096.0
        return angle_rad
    
    def get_angle_degrees(self):
        """
        Berechnet den Winkel in Grad (0-360°).
        
        Returns:
            float: Winkel in Grad oder None bei Fehler
        """
        angle_rad = self.get_angle()
        if angle_rad is None:
            return None
        
        # Umrechnung von Radiant zu Grad
        return math.degrees(angle_rad)
    
    def close(self):
        """
        Schließt den I2C-Bus.
        """
        if self.bus is not None:
            self.bus.close()
            self.bus = None
    
    def __str__(self):
        """
        String-Repräsentation des Sensors.
        """
        return f"MT6701('{self.name}', bus={self.bus_num}, address=0x{self.address:02x})"

    def __del__(self):
        """
        Aufräumen beim Löschen des Objekts.
        """
        self.close()

def scan_i2c_devices(bus=1):
    """
    Scannt den I2C-Bus nach angeschlossenen Geräten.
    
    Args:
        bus (int): I2C-Bus Nummer (Standardwert: 1)
        
    Returns:
        list: Liste mit den gefundenen Geräteadressen
    """
    print(f"Scanning I2C bus {bus} for devices...")
    
    try:
        bus_obj = smbus2.SMBus(bus)
        devices = []
        
        # Scanne Adressen von 0x03 bis 0x77 (standard I2C Adressbereich)
        for addr in range(0x03, 0x78):
            try:
                bus_obj.read_byte(addr)
                devices.append(addr)
                print(f"Found device at address: 0x{addr:02x}")
            except Exception:
                pass
        
        bus_obj.close()
        
        if not devices:
            print("No I2C devices found")
        else:
            print(f"Found {len(devices)} I2C device(s)")
            
        return devices
    except Exception as e:
        print(f"Error scanning I2C bus: {e}")
        return []