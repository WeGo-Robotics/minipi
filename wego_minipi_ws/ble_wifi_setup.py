#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import subprocess
import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
from gi.repository import GLib

BLUEZ_SERVICE_NAME = 'org.bluez'
GATT_MANAGER_IFACE = 'org.bluez.GattManager1'
ADVERTISING_MANAGER_IFACE = 'org.bluez.LEAdvertisingManager1'

WIFI_SERVICE_UUID = '12345678-1234-5678-1234-56789abc0001'
SSID_CHAR_UUID    = '12345678-1234-5678-1234-56789abc0002'
PWD_CHAR_UUID     = '12345678-1234-5678-1234-56789abc0003'

DEV_NAME = 'Linux_Setup_GATT'

wifi_ssid = ""
wifi_pwd = ""


# =========================
# 공통 Base
# =========================

class Application(dbus.service.Object):
    """ObjectManager 구현. BlueZ가 이걸 읽어서 GATT 트리를 인식함."""
    def __init__(self, bus):
        self.path = '/'
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)

    def add_service(self, service):
        self.services.append(service)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method('org.freedesktop.DBus.ObjectManager',
                         out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        managed = {}
        for service in self.services:
            managed[service.get_path()] = service.get_properties()
            for ch in service.get_characteristics():
                managed[ch.get_path()] = ch.get_properties()
        return managed


class Service(dbus.service.Object):
    def __init__(self, bus, index, uuid, primary):
        self.path = f'/org/bluez/example/service{index}'
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        dbus.service.Object.__init__(self, bus, self.path)

    def add_characteristic(self, ch):
        self.characteristics.append(ch)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def get_characteristics(self):
        return self.characteristics

    def get_properties(self):
        return {
            'org.bluez.GattService1': {
                'UUID': self.uuid,
                'Primary': self.primary,
                'Characteristics': dbus.Array(
                    [ch.get_path() for ch in self.characteristics],
                    signature='o'
                )
            }
        }


class Characteristic(dbus.service.Object):
    def __init__(self, bus, index, uuid, flags, service):
        self.path = service.path + f'/char{index}'
        self.bus = bus
        self.uuid = uuid
        self.flags = flags
        self.service = service
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def get_properties(self):
        return {
            'org.bluez.GattCharacteristic1': {
                'UUID': self.uuid,
                'Service': self.service.get_path(),
                'Flags': dbus.Array(self.flags, signature='s'),
            }
        }

    # read는 안 쓸 거라 구현 안 함
    @dbus.service.method('org.bluez.GattCharacteristic1',
                         in_signature='aya{sv}', out_signature='')
    def WriteValue(self, value, options):
        pass  # 각 subclass에서 override


# =========================
# Wi-Fi SSID / Password 특성
# =========================

class SSIDCharacteristic(Characteristic):
    def __init__(self, bus, index, service):
        # write-without-response 만 사용 → 절대 페어링 안 뜨게
        super().__init__(bus, index, SSID_CHAR_UUID,
                         ['write-without-response'], service)

    @dbus.service.method('org.bluez.GattCharacteristic1',
                         in_signature='aya{sv}', out_signature='')
    def WriteValue(self, value, options):
        global wifi_ssid
        wifi_ssid = bytes(value).decode('utf-8')
        print(f"[BLE] SSID received: {wifi_ssid}")


class PasswordCharacteristic(Characteristic):
    def __init__(self, bus, index, service):
        super().__init__(bus, index, PWD_CHAR_UUID,
                         ['write-without-response'], service)

    @dbus.service.method('org.bluez.GattCharacteristic1',
                         in_signature='aya{sv}', out_signature='')
    def WriteValue(self, value, options):
        global wifi_pwd, wifi_ssid
        wifi_pwd = bytes(value).decode('utf-8')
        print(f"[BLE] Password received: {wifi_pwd}")

        if wifi_ssid:
            connect_wifi(wifi_ssid, wifi_pwd)
        else:
            print("[WARN] Password received but SSID is empty.")


# =========================
# Advertisement
# =========================

class Advertisement(dbus.service.Object):
    PATH_BASE = '/org/bluez/example/advertisement'

    def __init__(self, bus, index):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def get_properties(self):
        return {
            'org.bluez.LEAdvertisement1': {
                'Type': 'peripheral',
                'ServiceUUIDs': dbus.Array([WIFI_SERVICE_UUID], signature='s'),
                'LocalName': DEV_NAME,
                'Discoverable': dbus.Boolean(True),
            }
        }

    @dbus.service.method('org.freedesktop.DBus.Properties',
                         in_signature='s', out_signature='a{sv}')
    def GetAll(self, interface):
        # advertising manager가 여기서 속성 읽어감
        if interface != 'org.bluez.LEAdvertisement1':
            raise dbus.exceptions.DBusException(
                'org.freedesktop.DBus.Error.InvalidArgs',
                'Invalid interface'
            )
        return self.get_properties()['org.bluez.LEAdvertisement1']

    @dbus.service.method('org.bluez.LEAdvertisement1',
                         in_signature='', out_signature='')
    def Release(self):
        print("[BLE] Advertisement released")


# =========================
# Wi-Fi 연결 함수
# =========================

def connect_wifi(ssid, password):
    print(f"[WiFi] Trying to connect to {ssid} ...")

    # 먼저 주변 AP 스캔 (SSID 못 찾는 문제 방지)
    subprocess.run(['nmcli', 'dev', 'wifi', 'rescan'])

    time.sleep(2)

    cmd = ['sudo', 'nmcli', 'dev', 'wifi', 'connect', ssid, 'password', password]
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode == 0:
        print(f"[WiFi] Connected to {ssid}!")
    else:
        print("[WiFi] Failed:")
        print(result.stderr.strip())


# =========================
# 메인
# =========================

def main():
    # BLE 스택 세팅(필요하면 한번만)
    os.system("sudo btmgmt power off")
    os.system("sudo btmgmt le on")
    os.system("sudo btmgmt bredr off")
    os.system("sudo btmgmt power on")
    os.system("sudo btmgmt connectable on")
    os.system("sudo btmgmt bondable off")  # 페어링(저장) 안 함

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    adapter_path = '/org/bluez/hci0'

    # GATT / Advertising Manager 가져오기
    service_manager = dbus.Interface(
        bus.get_object(BLUEZ_SERVICE_NAME, adapter_path),
        GATT_MANAGER_IFACE)

    ad_manager = dbus.Interface(
        bus.get_object(BLUEZ_SERVICE_NAME, adapter_path),
        ADVERTISING_MANAGER_IFACE)

    app = Application(bus)

    # Wi-Fi Service 추가
    wifi_service = Service(bus, 0, WIFI_SERVICE_UUID, True)
    app.add_service(wifi_service)

    ssid_char = SSIDCharacteristic(bus, 0, wifi_service)
    pwd_char = PasswordCharacteristic(bus, 1, wifi_service)

    wifi_service.add_characteristic(ssid_char)
    wifi_service.add_characteristic(pwd_char)

    # GATT 등록
    print("[BLE] Registering GATT application...")
    service_manager.RegisterApplication(
        app.get_path(), {},
        reply_handler=lambda: print("[BLE] GATT registered."),
        error_handler=lambda e: print("[ERR] GATT register failed:", e))

    # 광고 등록
    adv = Advertisement(bus, 0)
    print("[BLE] Registering advertisement...")
    ad_manager.RegisterAdvertisement(
        adv.get_path(), {},
        reply_handler=lambda: print("[BLE] Advertising started."),
        error_handler=lambda e: print("[ERR] Adv register failed:", e))

    print("[BLE] Ready. Waiting for Web Bluetooth client...")
    GLib.MainLoop().run()


if __name__ == '__main__':
    main()
