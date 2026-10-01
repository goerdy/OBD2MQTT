#pragma once

// Copy this file to src/config.h and replace every value with your setup.
#define OBD2MQTT_WIFI_SSID "your-wifi-ssid"
#define OBD2MQTT_WIFI_PASSWORD "your-wifi-password"
#define OBD2MQTT_MQTT_HOST "192.0.2.10"
#define OBD2MQTT_MQTT_USER "mqtt-user"
#define OBD2MQTT_MQTT_PASSWORD "mqtt-password"

// Use a unique prefix and the MAC address of the matching OBD dongle.
#define OBD2MQTT_MQTT_DATA_PREFIX "Car/YourCar/data/"
#define OBD2MQTT_MQTT_STATUS_TOPIC "Car/YourCar/status"
#define OBD2MQTT_TARGET_ADDRESS "AA:BB:CC:DD:EE:FF"
#define OBD2MQTT_TARGET_NAME "IOS-Vlink"

#define OBD2MQTT_SERVICE_UUID "000018f0-0000-1000-8000-00805f9b34fb"
#define OBD2MQTT_NOTIFY_UUID "00002af0-0000-1000-8000-00805f9b34fb"
#define OBD2MQTT_WRITE_UUID "00002af1-0000-1000-8000-00805f9b34fb"
