# OBD2MQTT

OBD2MQTT reads a Seat Mii electric, VW e-Up Gen2 or Škoda Citigo-e iV through a Vgate iCar Pro 2S BLE adapter and publishes vehicle telemetry over Wi-Fi and MQTT.

The firmware targets the Seeed Studio XIAO ESP32-C6 and is designed for permanent power. Each cycle scans for the configured BLE dongle, reads the vehicle, publishes retained MQTT values when Wi-Fi is available, then enters deep sleep for 15 minutes. Only the timer wakes the controller.

## Setup

Copy the example configuration and edit it for the vehicle:

```sh
cp src/config.example.h src/config.h
```

Set the Wi-Fi and MQTT credentials, the MQTT topic prefix, and the BLE MAC address of the matching OBD dongle. `src/config.h` is ignored by Git and must never be committed.

The two dongles have the same advertised name, so the MAC address is the vehicle identity. Keep one configuration per controller and vehicle.

## Build and flash

Install PlatformIO, connect the XIAO ESP32-C6, and run:

```sh
pio run -e xiao_esp32c6
pio run -e xiao_esp32c6 -t upload
pio device monitor --baud 115200
```

The serial log reports BLE/ELM327 activity, telemetry, MQTT results and `SLEEP seconds=900`.

## MQTT

The configured status topic receives `ok` or `noOBDdata`. Individual retained values are published below the configured data prefix, including SOC, odometer, readiness, charging state, range, battery voltage/current/power and charge time where available.

## Safety and limitations

The firmware uses read-only diagnostic requests. It does not clear faults, code control units, start vehicle routines or write vehicle settings. Some optional charger values may report `NO DATA`; those fields remain unavailable rather than being inferred.

## License

Choose and add a license before distributing the project.
