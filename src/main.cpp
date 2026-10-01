#include <Arduino.h>
#include <NimBLEDevice.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include "config.h"

namespace {
constexpr uint32_t kBaudRate = 115200;
constexpr uint32_t kScanSeconds = 8;
constexpr uint32_t kBleConnectTimeoutMs = 10000;
constexpr uint32_t kCommandTimeoutMs = 4000;
constexpr uint32_t kUdsTimeoutMs = 6000;
constexpr uint32_t kResetTimeoutMs = 8000;
constexpr uint32_t kSleepSeconds = 15 * 60;  // 15 minutes between polling cycles.
constexpr uint32_t kTelemetryMagic = 0x4D494932;
constexpr uint32_t kWifiConnectTimeoutMs = 10000;
constexpr uint16_t kMqttPort = 1885;
constexpr bool kActiveScan = true;
constexpr bool kEnableGpsProbe = false;

const char *kWifiSsid = OBD2MQTT_WIFI_SSID;
const char *kWifiPassword = OBD2MQTT_WIFI_PASSWORD;
const char *kMqttHost = OBD2MQTT_MQTT_HOST;
const char *kMqttDataPrefix = OBD2MQTT_MQTT_DATA_PREFIX;
const char *kMqttStatusTopic = OBD2MQTT_MQTT_STATUS_TOPIC;
const char *kMqttUser = OBD2MQTT_MQTT_USER;
const char *kMqttPassword = OBD2MQTT_MQTT_PASSWORD;
const char *kTargetAddress = OBD2MQTT_TARGET_ADDRESS;
const char *kTargetName = OBD2MQTT_TARGET_NAME;
const char *kServiceUuid = OBD2MQTT_SERVICE_UUID;
const char *kNotifyCharacteristicUuid = OBD2MQTT_NOTIFY_UUID;
const char *kWriteCharacteristicUuid = OBD2MQTT_WRITE_UUID;

NimBLEScan *scanner = nullptr;
NimBLEClient *client = nullptr;
NimBLERemoteCharacteristic *notifyCharacteristic = nullptr;
NimBLERemoteCharacteristic *writeCharacteristic = nullptr;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

String responseBuffer;
String foundAddress;
uint8_t foundAddressType = 0;
String foundName;
bool foundTarget = false;
bool elmReady = false;
bool vehicleReady = false;
bool gpsProbeDone = false;
uint32_t lastGpsProbeMs = 0;

struct TelemetryData {
  uint32_t magic;
  bool valid;
  uint32_t ageMinutes;
  float soc;
  uint32_t odometerKm;
  float socNorm;
  bool ready;
  bool charging;
  uint8_t hvChargeMode;
  uint8_t lvPowerState;
  uint8_t driveState;
  int8_t gear;
  uint8_t speedKph;
  uint16_t rangeKm;
  float rangeEnergyKwh;
  float bmsVoltage;
  float bmsCurrent;
  float bmsPowerKw;
  float acPowerKw;
  float dcPowerKw;
  int chargeMinutes;
  char pluggedGuess[8];
  char plugRaw[40];
};

RTC_DATA_ATTR TelemetryData rtcTelemetry = {};
RTC_DATA_ATTR uint32_t rtcPendingAgeSeconds = 0;

String advertisedName(const NimBLEAdvertisedDevice &device) {
  if (device.haveName()) {
    return device.getName().c_str();
  }
  return "-";
}

bool isTargetDongle(const NimBLEAdvertisedDevice &device) {
  const String address = device.getAddress().toString().c_str();
  // Both cars use the same dongle name and service; only the MAC identifies the car.
  return address.equalsIgnoreCase(kTargetAddress);
}

void disconnectClient() {
  elmReady = false;
  vehicleReady = false;
  gpsProbeDone = false;
  lastGpsProbeMs = 0;
  notifyCharacteristic = nullptr;
  writeCharacteristic = nullptr;

  if (client != nullptr && client->isConnected()) {
    client->disconnect();
  }
}

void notifyCallback(NimBLERemoteCharacteristic *, uint8_t *data, size_t length, bool) {
  for (size_t i = 0; i < length; ++i) {
    const char c = static_cast<char>(data[i]);
    responseBuffer += c;
    Serial.write(c);
  }
}

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *advertised) override {
    const NimBLEAdvertisedDevice &device = *advertised;
    if (foundTarget || !isTargetDongle(device)) {
      return;
    }

    foundTarget = true;
    foundAddress = device.getAddress().toString().c_str();
    foundAddressType = device.getAddressType();
    foundName = advertisedName(device);

    Serial.print("OBD dongle found addr=");
    Serial.print(foundAddress);
    Serial.print("; name=");
    Serial.print(foundName);
    Serial.print("; rssi=");
    Serial.println(device.getRSSI());
    scanner->stop();
  }
};

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient *) override {
    Serial.println("BLE connected");
  }

  void onDisconnect(NimBLEClient *, int reason) override {
    Serial.print("BLE disconnected; reason=");
    Serial.println(reason);
    elmReady = false;
  }
};

ScanCallbacks scanCallbacks;
ClientCallbacks clientCallbacks;

bool scanForDongle() {
  foundTarget = false;
  foundAddress = "";
  foundName = "";

  Serial.print("BLE scan start seconds=");
  Serial.println(kScanSeconds);
  scanner->clearResults();
  scanner->getResults(kScanSeconds * 1000, false);
  scanner->clearResults();

  if (!foundTarget) {
    Serial.println("OBD dongle not found");
  }

  return foundTarget;
}

bool connectDongle() {
  if (foundAddress.length() == 0) {
    return false;
  }

  if (client == nullptr) {
    client = NimBLEDevice::createClient();
    client->setClientCallbacks(&clientCallbacks, false);
    client->setConnectTimeout(kBleConnectTimeoutMs);
    client->setConnectRetries(0);
  }

  Serial.print("BLE connect addr=");
  Serial.print(foundAddress);
  Serial.print("; name=");
  Serial.println(foundName);

  Serial.print("BLE connect address_type=");
  Serial.print(foundAddressType);
  Serial.print("; timeout_ms=");
  Serial.println(kBleConnectTimeoutMs);
  // Short ELM commands fit the default MTU; no explicit MTU exchange is needed.
  const uint32_t connectStarted = millis();
  if (!client->connect(NimBLEAddress(foundAddress.c_str(), foundAddressType), true, false, false)) {
    Serial.print("BLE connect failed; elapsed_ms=");
    Serial.print(millis() - connectStarted);
    Serial.print("; error=");
    Serial.println(client->getLastError());
    disconnectClient();
    return false;
  }

  NimBLERemoteService *service = client->getService(NimBLEUUID(kServiceUuid));
  if (service == nullptr) {
    Serial.println("BLE service 18F0 not found");
    disconnectClient();
    return false;
  }

  notifyCharacteristic = service->getCharacteristic(NimBLEUUID(kNotifyCharacteristicUuid));
  writeCharacteristic = service->getCharacteristic(NimBLEUUID(kWriteCharacteristicUuid));

  if (notifyCharacteristic == nullptr || writeCharacteristic == nullptr) {
    Serial.println("BLE ELM327 characteristics not found");
    disconnectClient();
    return false;
  }

  if (notifyCharacteristic->canNotify()) {
    if (!notifyCharacteristic->subscribe(true, notifyCallback)) {
      Serial.println("BLE notify subscription failed");
      disconnectClient();
      return false;
    }
    Serial.println("BLE notify subscribed on 2AF0");
  } else {
    Serial.println("BLE characteristic 2AF0 has no notify property");
    disconnectClient();
    return false;
  }

  if (!writeCharacteristic->canWrite() && !writeCharacteristic->canWriteNoResponse()) {
    Serial.println("BLE characteristic 2AF1 is not writable");
    disconnectClient();
    return false;
  }

  return true;
}

bool waitForPrompt(uint32_t timeoutMs) {
  const uint32_t started = millis();
  while (millis() - started < timeoutMs) {
    if (responseBuffer.indexOf('>') >= 0) {
      return true;
    }
    delay(10);
  }

  return false;
}

String normalizedResponse() {
  String normalized = responseBuffer;
  normalized.replace(">", "");
  normalized.trim();
  return normalized;
}

String hexOnly(const String &input) {
  String hex;
  for (size_t i = 0; i < input.length(); ++i) {
    const char c = input[i];
    if (isxdigit(c)) {
      hex += static_cast<char>(toupper(c));
    }
  }
  return hex;
}

String udsPayloadHex(const String &input) {
  String payload;
  int lineStart = 0;

  while (lineStart < static_cast<int>(input.length())) {
    int lineEnd = input.indexOf('\r', lineStart);
    const int newlineEnd = input.indexOf('\n', lineStart);
    if (lineEnd < 0 || (newlineEnd >= 0 && newlineEnd < lineEnd)) {
      lineEnd = newlineEnd;
    }
    if (lineEnd < 0) {
      lineEnd = input.length();
    }

    String line = input.substring(lineStart, lineEnd);
    line.replace(">", "");
    line.trim();

    const int colon = line.indexOf(':');
    if (colon == 1 && isxdigit(line[0])) {
      line = line.substring(2);
    } else if (line.length() <= 4 && line.indexOf("62") < 0) {
      line = "";
    }

    payload += hexOnly(line);
    lineStart = lineEnd + 1;
  }

  if (payload.length() == 0) {
    payload = hexOnly(input);
  }
  return payload;
}

int hexNibble(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  c = toupper(c);
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  return -1;
}

bool hexByteAt(const String &hex, int offset, uint8_t &value) {
  if (offset < 0 || offset + 1 >= static_cast<int>(hex.length())) {
    return false;
  }

  const int high = hexNibble(hex[offset]);
  const int low = hexNibble(hex[offset + 1]);
  if (high < 0 || low < 0) {
    return false;
  }

  value = static_cast<uint8_t>((high << 4) | low);
  return true;
}

bool hexUint16At(const String &hex, int offset, uint16_t &value) {
  uint8_t high = 0;
  uint8_t low = 0;
  if (!hexByteAt(hex, offset, high) || !hexByteAt(hex, offset + 2, low)) {
    return false;
  }
  value = (static_cast<uint16_t>(high) << 8) | low;
  return true;
}

bool hexInt16At(const String &hex, int offset, int16_t &value) {
  uint16_t raw = 0;
  if (!hexUint16At(hex, offset, raw)) {
    return false;
  }
  value = static_cast<int16_t>(raw);
  return true;
}

int dataOffset(const String &response, const char *did) {
  String prefix = "62";
  prefix += did;
  const int offset = response.indexOf(prefix);
  if (offset < 0) {
    return -1;
  }
  return offset + prefix.length();
}

bool responseContainsError(const String &response) {
  return response.indexOf("CAN ERROR") >= 0 || response.indexOf("NO DATA") >= 0 ||
         response.indexOf("STOPPED") >= 0 || response.indexOf("UNABLE TO CONNECT") >= 0 ||
         response.indexOf("?") >= 0;
}

bool sendElmCommand(const char *command, uint32_t timeoutMs = kCommandTimeoutMs, String *response = nullptr) {
  if (writeCharacteristic == nullptr) {
    return false;
  }

  responseBuffer = "";

  Serial.print("ELM TX ");
  Serial.println(command);

  String payload(command);
  payload += "\r";
  if (!writeCharacteristic->writeValue(
          reinterpret_cast<const uint8_t *>(payload.c_str()), payload.length(),
          !writeCharacteristic->canWriteNoResponse())) {
    Serial.println("ELM BLE write failed");
    return false;
  }

  if (!waitForPrompt(timeoutMs)) {
    Serial.print("ELM timeout command=");
    Serial.println(command);
    return false;
  }

  if (response != nullptr) {
    *response = normalizedResponse();
  }

  Serial.println();
  return true;
}

bool sendVehicleQuery(const char *command) {
  String response;
  if (!sendElmCommand(command, kCommandTimeoutMs, &response)) {
    return false;
  }

  if (responseContainsError(response)) {
    Serial.print("OBD query failed command=");
    Serial.print(command);
    Serial.print("; response=");
    Serial.println(response);
    return false;
  }

  Serial.print("OBD response command=");
  Serial.print(command);
  Serial.print("; response=");
  Serial.println(response);
  return true;
}

bool sendUdsQuery(const char *command, String *response) {
  String rawResponse;
  if (!sendElmCommand(command, kUdsTimeoutMs, &rawResponse)) {
    return false;
  }

  *response = udsPayloadHex(rawResponse);

  if (responseContainsError(rawResponse) ||
      (response->indexOf("7F") >= 0 && response->indexOf("62") < 0)) {
    Serial.print("UDS query failed command=");
    Serial.print(command);
    Serial.print("; response=");
    Serial.println(rawResponse);
    return false;
  }

  Serial.print("UDS response command=");
  Serial.print(command);
  Serial.print("; response=");
  Serial.println(*response);
  return true;
}

bool configureBatteryManagementEcu() {
  Serial.println("Vehicle ECU setup: BMS header=7E5 response=7ED");

  const bool ok =
      sendElmCommand("ATCAF1") &&
      sendElmCommand("ATCFC1") &&
      sendElmCommand("ATAL") &&
      sendElmCommand("ATST64") &&
      sendElmCommand("ATSH7E5") &&
      sendElmCommand("ATCRA7ED");

  vehicleReady = ok;
  Serial.println(ok ? "Vehicle ECU setup done" : "Vehicle ECU setup failed");
  return ok;
}

bool configureEcu(const char *name, const char *txHeader, const char *rxHeader) {
  Serial.print("Vehicle ECU setup: ");
  Serial.print(name);
  Serial.print(" header=");
  Serial.print(txHeader);
  Serial.print(" response=");
  Serial.println(rxHeader);

  String command = "ATSH";
  command += txHeader;
  if (!sendElmCommand(command.c_str())) {
    return false;
  }

  command = "ATCRA";
  command += rxHeader;
  return sendElmCommand(command.c_str());
}

bool readDidPayload(const char *ecuName, const char *txHeader, const char *rxHeader, const char *did,
                    String &response) {
  if (!configureEcu(ecuName, txHeader, rxHeader)) {
    return false;
  }

  String command = "22";
  command += did;
  return sendUdsQuery(command.c_str(), &response);
}

void probeGpsDid(const char *ecuName, const char *did, const char *label) {
  String rawResponse;
  String command = "22";
  command += did;

  const bool ok = sendElmCommand(command.c_str(), kUdsTimeoutMs, &rawResponse);
  const String payload = udsPayloadHex(rawResponse);
  const bool positiveResponse = ok && payload.indexOf("62") >= 0 && payload.indexOf("7F") < 0;

  if (positiveResponse) {
    Serial.print("GPS probe hit ecu=");
    Serial.print(ecuName);
    Serial.print("; did=");
    Serial.print(did);
    Serial.print("; label=");
    Serial.print(label);
    Serial.print("; payload=");
    Serial.println(payload);
  } else {
    Serial.print("GPS probe miss ecu=");
    Serial.print(ecuName);
    Serial.print("; did=");
    Serial.print(did);
    Serial.print("; label=");
    Serial.print(label);
    Serial.print("; payload=");
    Serial.println(payload);
  }
}

void startExtendedSession(const char *ecuName) {
  String rawResponse;
  if (sendElmCommand("1003", kUdsTimeoutMs, &rawResponse)) {
    Serial.print("GPS probe session ecu=");
    Serial.print(ecuName);
    Serial.print("; payload=");
    Serial.println(udsPayloadHex(rawResponse));
  }
}

void probeGpsData() {
  Serial.println("GPS probe start");

  struct Candidate {
    const char *did;
    const char *label;
  };
  static constexpr Candidate candidates[] = {
      {"F190", "VIN / control check"},
      {"06B0", "GPS Position / IDE01712"},
      {"06B1", "Status GPS reception / IDE01713"},
      {"0D95", "GPS time / IDE03477"},
      {"06EA", "Status GPS antenna / IDE01770"},
      {"1712", "GPS Position / IDE01712 direct"},
      {"1713", "Status GPS reception / IDE01713 direct"},
      {"3477", "GPS time / IDE03477 direct"},
      {"1770", "Status GPS antenna / IDE01770 direct"},
  };

  if (configureEcu("SG75", "767", "7D1")) {
    startExtendedSession("SG75");
    for (const Candidate &candidate : candidates) {
      probeGpsDid("SG75", candidate.did, candidate.label);
    }
  }

  if (configureEcu("SG5F", "773", "7DD")) {
    startExtendedSession("SG5F");
    for (const Candidate &candidate : candidates) {
      probeGpsDid("SG5F", candidate.did, candidate.label);
    }
  }

  configureBatteryManagementEcu();
  Serial.println("GPS probe done");
}

bool readBatterySoc(float &soc) {
  String response;
  if (!sendUdsQuery("22028C", &response)) {
    return false;
  }

  const int prefix = response.indexOf("62028C");
  uint8_t rawSoc = 0;
  if (prefix < 0 || !hexByteAt(response, prefix + 6, rawSoc)) {
    Serial.print("SOC parse failed response=");
    Serial.println(response);
    return false;
  }

  soc = rawSoc / 2.55f;
  Serial.print("SOC=");
  Serial.print(soc, 1);
  Serial.print("; raw=0x");
  if (rawSoc < 0x10) {
    Serial.print('0');
  }
  Serial.println(rawSoc, HEX);
  return true;
}

bool readOdometer(uint32_t &odometerKm) {
  String response;
  if (!sendUdsQuery("2202BD", &response)) {
    return false;
  }

  const int prefix = response.indexOf("6202BD");
  uint8_t kmHigh = 0;
  uint8_t kmMid = 0;
  uint8_t kmLow = 0;
  if (prefix < 0 || !hexByteAt(response, prefix + 8, kmHigh) ||
      !hexByteAt(response, prefix + 10, kmMid) || !hexByteAt(response, prefix + 12, kmLow)) {
    Serial.print("ODO parse failed response=");
    Serial.println(response);
    return false;
  }

  odometerKm = (static_cast<uint32_t>(kmHigh) << 16) | (static_cast<uint32_t>(kmMid) << 8) |
               static_cast<uint32_t>(kmLow);
  Serial.print("ODO=");
  Serial.print(odometerKm);
  Serial.println("km");
  return true;
}

bool readBmsVoltageCurrent(float &voltage, float &current, float &powerKw) {
  String response;
  uint16_t rawVoltage = 0;
  uint16_t rawCurrent = 0;

  bool gotVoltage = false;
  bool gotCurrent = false;

  if (sendUdsQuery("221E3B", &response)) {
    const int offset = dataOffset(response, "1E3B");
    gotVoltage = offset >= 0 && hexUint16At(response, offset, rawVoltage);
  }

  if (sendUdsQuery("221E3D", &response)) {
    const int offset = dataOffset(response, "1E3D");
    gotCurrent = offset >= 0 && hexUint16At(response, offset, rawCurrent);
  }

  if (!gotVoltage || !gotCurrent) {
    return false;
  }

  voltage = rawVoltage / 4.0f;
  current = ((rawCurrent - 2044.0f) / 4.0f) * -1.0f;
  powerKw = voltage * current / 1000.0f;
  return true;
}

bool readChgMgmt(uint8_t &hvChargeMode, uint8_t &lvPowerState, float &socNorm, int &chargeMinutes,
                 String &plugRaw) {
  bool gotAny = false;
  String response;

  if (readDidPayload("CHG_MGMT", "765", "7CF", "1DD6", response)) {
    const int offset = dataOffset(response, "1DD6");
    gotAny |= offset >= 0 && hexByteAt(response, offset, hvChargeMode);
  }

  if (sendUdsQuery("221DEC", &response)) {
    const int offset = dataOffset(response, "1DEC");
    gotAny |= offset >= 0 && hexByteAt(response, offset, lvPowerState);
  }

  if (sendUdsQuery("221DD0", &response)) {
    const int offset = dataOffset(response, "1DD0");
    uint8_t raw = 0;
    if (offset >= 0 && hexByteAt(response, offset, raw)) {
      socNorm = raw / 2.0f;
      gotAny = true;
    }
  }

  if (sendUdsQuery("221DE4", &response)) {
    const int offset = dataOffset(response, "1DE4");
    uint8_t raw = 0;
    if (offset >= 0 && hexByteAt(response, offset, raw) && raw != 127) {
      chargeMinutes = raw * 5;
      gotAny = true;
    }
  }

  if (sendUdsQuery("221DDA", &response)) {
    plugRaw = response;
    gotAny = true;
  }

  return gotAny;
}

bool readDriveState(uint8_t &driveState, int8_t &gear, uint8_t &speedKph) {
  bool gotAny = false;
  String response;

  if (readDidPayload("MOT_ELEC", "7E0", "7E8", "14D9", response)) {
    const int offset = dataOffset(response, "14D9");
    gotAny |= offset >= 0 && hexByteAt(response, offset, driveState);
  }

  if (sendUdsQuery("2214CB", &response)) {
    const int offset = dataOffset(response, "14CB");
    uint8_t rawGear = 0;
    if (offset >= 0 && hexByteAt(response, offset, rawGear)) {
      gear = static_cast<int8_t>(rawGear);
      gotAny = true;
    }
  }

  if (sendUdsQuery("22F40D", &response)) {
    const int offset = dataOffset(response, "F40D");
    gotAny |= offset >= 0 && hexByteAt(response, offset, speedKph);
  }

  return gotAny;
}

bool readRange(uint16_t &rangeKm, float &rangeEnergyKwh) {
  bool gotAny = false;
  String response;

  if (readDidPayload("MFD", "714", "77E", "22E0", response)) {
    const int offset = dataOffset(response, "22E0");
    gotAny |= offset >= 0 && hexUint16At(response, offset, rangeKm);
  }

  if (sendUdsQuery("2222E4", &response)) {
    const int offset = dataOffset(response, "22E4");
    uint16_t raw = 0;
    if (offset >= 0 && hexUint16At(response, offset, raw) && raw != 511) {
      rangeEnergyKwh = raw / 10.0f;
      gotAny = true;
    }
  }

  return gotAny;
}

bool readChargePower(float &acPowerKw, float &dcPowerKw) {
  bool gotAny = false;
  bool gotAcU = false;
  bool gotAcI = false;
  bool gotDcU = false;
  bool gotDcI = false;
  String response;
  uint16_t ac1u = 0;
  uint16_t ac2u = 0;
  uint8_t ac1iRaw = 0;
  uint8_t ac2iRaw = 0;
  uint16_t dc1u = 0;
  uint16_t dc2u = 0;
  uint16_t dc1iRaw = 0;
  uint16_t dc2iRaw = 0;

  if (readDidPayload("CHARGER", "744", "7AE", "41FC", response)) {
    const int offset = dataOffset(response, "41FC");
    gotAcU = offset >= 0 && hexUint16At(response, offset, ac1u);
    if (offset >= 0) {
      hexUint16At(response, offset + 4, ac2u);
    }
    gotAny |= gotAcU;
  }

  if (sendUdsQuery("2241FB", &response)) {
    const int offset = dataOffset(response, "41FB");
    if (offset >= 0) {
      gotAcI = hexByteAt(response, offset, ac1iRaw);
      hexByteAt(response, offset + 2, ac2iRaw);
      gotAny |= gotAcI;
    }
  }

  if (sendUdsQuery("2241F8", &response)) {
    const int offset = dataOffset(response, "41F8");
    if (offset >= 0) {
      gotDcU = hexUint16At(response, offset, dc1u);
      hexUint16At(response, offset + 4, dc2u);
      gotAny |= gotDcU;
    }
  }

  if (sendUdsQuery("2241F9", &response)) {
    const int offset = dataOffset(response, "41F9");
    if (offset >= 0) {
      gotDcI = hexUint16At(response, offset, dc1iRaw);
      hexUint16At(response, offset + 4, dc2iRaw);
      gotAny |= gotDcI;
    }
  }

  if (gotAcU && gotAcI) {
    const float ac1i = ac1iRaw == 255 ? 0.0f : ac1iRaw / 10.0f;
    const float ac2i = ac2iRaw == 255 ? 0.0f : ac2iRaw / 10.0f;
    acPowerKw = (ac1u * ac1i + ac2u * ac2i) / 1000.0f;
  }

  if (gotDcU && gotDcI) {
    const float dc1i = dc1iRaw == 1023 ? 0.0f : (dc1iRaw - 510.0f) / 5.0f;
    const float dc2i = dc2iRaw == 1023 ? 0.0f : (dc2iRaw - 510.0f) / 5.0f;
    dcPowerKw = (dc1u * dc1i + dc2u * dc2i) / 1000.0f;
  }

  return gotAny;
}

void copyStringToBuffer(const String &source, char *target, size_t targetSize) {
  if (targetSize == 0) {
    return;
  }

  const size_t count = min(source.length(), targetSize - 1);
  memcpy(target, source.c_str(), count);
  target[count] = '\0';
}

String telemetryPayload(const TelemetryData &telemetry, bool fresh) {
  String payload = "AGE_MIN=";
  payload += telemetry.ageMinutes;
  payload += ";FRESH=";
  payload += fresh ? "yes" : "no";
  payload += ";SOC=";
  payload += String(telemetry.soc, 1);
  payload += ";ODO=";
  payload += telemetry.odometerKm;
  if (telemetry.socNorm >= 0) {
    payload += ";SOC_NORM=";
    payload += String(telemetry.socNorm, 1);
  }
  payload += ";READY=";
  payload += telemetry.ready ? "yes" : "no";
  payload += ";CHARGING=";
  payload += telemetry.charging ? "yes" : "no";
  payload += ";PLUGGED_GUESS=";
  payload += telemetry.pluggedGuess;
  payload += ";CHG_MODE=";
  payload += telemetry.hvChargeMode;
  payload += ";LV_STATE=";
  payload += telemetry.lvPowerState;
  payload += ";DRIVE_STATE=";
  payload += telemetry.driveState;
  payload += ";GEAR=";
  payload += telemetry.gear;
  payload += ";SPEED=";
  payload += telemetry.speedKph;
  payload += ";RANGE=";
  payload += telemetry.rangeKm;
  payload += ";RANGE_KWH=";
  payload += String(telemetry.rangeEnergyKwh, 1);
  payload += ";BAT_U=";
  payload += String(telemetry.bmsVoltage, 1);
  payload += ";BAT_I=";
  payload += String(telemetry.bmsCurrent, 1);
  payload += ";BAT_P=";
  payload += String(telemetry.bmsPowerKw, 2);
  payload += ";AC_P=";
  payload += String(telemetry.acPowerKw, 2);
  payload += ";DC_P=";
  payload += String(telemetry.dcPowerKw, 2);
  if (telemetry.chargeMinutes >= 0) {
    payload += ";CHG_REM_MIN=";
    payload += telemetry.chargeMinutes;
  }
  payload += ";PLUG_RAW=";
  payload += telemetry.plugRaw;
  return payload;
}

void printTelemetry(const TelemetryData &telemetry, bool fresh) {
  Serial.print("TELEMETRY ");
  Serial.println(telemetryPayload(telemetry, fresh));
}

String dataTopic(const char *name) {
  String topic = kMqttDataPrefix;
  topic += name;
  return topic;
}

bool connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(kWifiSsid, kWifiPassword);

  Serial.print("WIFI connect ssid=");
  Serial.println(kWifiSsid);

  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < kWifiConnectTimeoutMs) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WIFI connect failed");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return false;
  }

  Serial.print("WIFI connected ip=");
  Serial.println(WiFi.localIP());
  return true;
}

bool connectMqtt() {
  if (!connectWifi()) {
    return false;
  }

  wifiClient.setConnectionTimeout(5000);
  mqttClient.setSocketTimeout(5);
  mqttClient.setServer(kMqttHost, kMqttPort);
  mqttClient.setBufferSize(768);
  String clientId = "MiiMesh-";
  clientId += WiFi.macAddress();
  clientId.replace(":", "");

  Serial.print("MQTT connect host=");
  Serial.print(kMqttHost);
  Serial.print("; port=");
  Serial.println(kMqttPort);

  if (!mqttClient.connect(clientId.c_str(), kMqttUser, kMqttPassword)) {
    Serial.print("MQTT connect failed state=");
    Serial.println(mqttClient.state());
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return false;
  }

  return true;
}

void disconnectMqtt() {
  mqttClient.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

bool publishMqttPayload(const char *topic, const String &payload) {
  if (!connectMqtt()) {
    return false;
  }

  const bool ok = mqttClient.publish(topic, payload.c_str(), true);
  Serial.print("MQTT publish topic=");
  Serial.print(topic);
  Serial.print("; result=");
  Serial.println(ok ? "ok" : "failed");
  disconnectMqtt();
  return ok;
}

bool publishMqttValue(const char *name, const String &value) {
  const String topic = dataTopic(name);
  const bool ok = mqttClient.publish(topic.c_str(), value.c_str(), true);
  Serial.print("MQTT publish topic=");
  Serial.print(topic);
  Serial.print("; value=");
  Serial.print(value);
  Serial.print("; result=");
  Serial.println(ok ? "ok" : "failed");
  return ok;
}

bool publishMqttTelemetry(const TelemetryData &telemetry) {
  if (telemetry.magic != kTelemetryMagic || !telemetry.valid) {
    return false;
  }

  if (!connectMqtt()) {
    return false;
  }

  bool ok = true;
  ok &= mqttClient.publish(kMqttStatusTopic, "ok", true);
  ok &= publishMqttValue("age_min", String(telemetry.ageMinutes));
  ok &= publishMqttValue("soc", String(telemetry.soc, 1));
  ok &= publishMqttValue("odo", String(telemetry.odometerKm));
  if (telemetry.socNorm >= 0) {
    ok &= publishMqttValue("soc_norm", String(telemetry.socNorm, 1));
  }
  ok &= publishMqttValue("ready", telemetry.ready ? "yes" : "no");
  ok &= publishMqttValue("charging", telemetry.charging ? "yes" : "no");
  ok &= publishMqttValue("plugged_guess", telemetry.pluggedGuess);
  ok &= publishMqttValue("chg_mode", String(telemetry.hvChargeMode));
  ok &= publishMqttValue("lv_state", String(telemetry.lvPowerState));
  ok &= publishMqttValue("drive_state", String(telemetry.driveState));
  ok &= publishMqttValue("gear", String(telemetry.gear));
  ok &= publishMqttValue("speed", String(telemetry.speedKph));
  ok &= publishMqttValue("range", String(telemetry.rangeKm));
  ok &= publishMqttValue("range_kwh", String(telemetry.rangeEnergyKwh, 1));
  ok &= publishMqttValue("bat_u", String(telemetry.bmsVoltage, 1));
  ok &= publishMqttValue("bat_i", String(telemetry.bmsCurrent, 1));
  ok &= publishMqttValue("bat_p", String(telemetry.bmsPowerKw, 2));
  ok &= publishMqttValue("ac_p", String(telemetry.acPowerKw, 2));
  ok &= publishMqttValue("dc_p", String(telemetry.dcPowerKw, 2));
  if (telemetry.chargeMinutes >= 0) {
    ok &= publishMqttValue("chg_rem_min", String(telemetry.chargeMinutes));
  }
  ok &= publishMqttValue("plug_raw", telemetry.plugRaw);

  Serial.print("MQTT telemetry publish ");
  Serial.println(ok ? "ok" : "partial-failed");
  disconnectMqtt();
  return ok;
}

void deepSleep() {
  disconnectClient();
  // Stop the radio without deleting clients after their host timers are gone.
  // Deep sleep resets normal RAM, so these objects need no explicit deletion.
  NimBLEDevice::deinit(false);
  rtcPendingAgeSeconds += kSleepSeconds;
  // Only the timer may wake the permanently powered controller.
  ESP_ERROR_CHECK(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL));
  ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(
      static_cast<uint64_t>(kSleepSeconds) * 1000000ULL));
  Serial.print("SLEEP seconds=");
  Serial.println(kSleepSeconds);
  Serial.flush();
  esp_deep_sleep_start();
}

bool pollVehicleMetrics(TelemetryData &telemetry) {
  float soc = 0;
  uint32_t odometerKm = 0;
  float bmsVoltage = 0;
  float bmsCurrent = 0;
  float bmsPowerKw = 0;
  uint8_t hvChargeMode = 0;
  uint8_t lvPowerState = 0;
  float socNorm = -1;
  int chargeMinutes = -1;
  String plugRaw = "-";
  uint8_t driveState = 0;
  int8_t gear = 0;
  uint8_t speedKph = 0;
  uint16_t rangeKm = 0;
  float rangeEnergyKwh = -1;
  float acPowerKw = 0;
  float dcPowerKw = 0;

  configureBatteryManagementEcu();
  const bool gotSoc = readBatterySoc(soc);
  const bool gotOdo = readOdometer(odometerKm);
  const bool gotBmsPower = readBmsVoltageCurrent(bmsVoltage, bmsCurrent, bmsPowerKw);
  const bool gotChgMgmt = readChgMgmt(hvChargeMode, lvPowerState, socNorm, chargeMinutes, plugRaw);
  const bool gotDriveState = readDriveState(driveState, gear, speedKph);
  const bool gotRange = readRange(rangeKm, rangeEnergyKwh);
  const bool gotChargePower = readChargePower(acPowerKw, dcPowerKw);

  if (gotSoc && gotOdo) {
    const bool ready = (lvPowerState > 12) || driveState == 4;
    const bool charging = hvChargeMode != 0 || acPowerKw > 0.1f || dcPowerKw > 0.1f || bmsPowerKw < -0.1f;
    const char *pluggedGuess = hvChargeMode != 0 ? "yes" : "unknown";

    telemetry.magic = kTelemetryMagic;
    telemetry.valid = true;
    telemetry.ageMinutes = 0;
    telemetry.soc = soc;
    telemetry.odometerKm = odometerKm;
    telemetry.socNorm = gotChgMgmt ? socNorm : -1;
    telemetry.ready = ready;
    telemetry.charging = charging;
    telemetry.hvChargeMode = hvChargeMode;
    telemetry.lvPowerState = lvPowerState;
    telemetry.driveState = driveState;
    telemetry.gear = gear;
    telemetry.speedKph = speedKph;
    telemetry.rangeKm = gotRange ? rangeKm : 0;
    telemetry.rangeEnergyKwh = gotRange ? rangeEnergyKwh : -1;
    telemetry.bmsVoltage = gotBmsPower ? bmsVoltage : 0;
    telemetry.bmsCurrent = gotBmsPower ? bmsCurrent : 0;
    telemetry.bmsPowerKw = gotBmsPower ? bmsPowerKw : 0;
    telemetry.acPowerKw = gotChargePower ? acPowerKw : 0;
    telemetry.dcPowerKw = gotChargePower ? dcPowerKw : 0;
    telemetry.chargeMinutes = chargeMinutes;
    copyStringToBuffer(pluggedGuess, telemetry.pluggedGuess, sizeof(telemetry.pluggedGuess));
    copyStringToBuffer(plugRaw, telemetry.plugRaw, sizeof(telemetry.plugRaw));
  }

  if (kEnableGpsProbe && (!gpsProbeDone || millis() - lastGpsProbeMs >= 30000)) {
    gpsProbeDone = true;
    lastGpsProbeMs = millis();
    probeGpsData();
  }

  return gotSoc && gotOdo;
}

bool initializeElm327() {
  Serial.println("ELM init start");

  const bool ok =
      sendElmCommand("ATZ", kResetTimeoutMs) &&
      sendElmCommand("ATE0") &&
      sendElmCommand("ATL0") &&
      sendElmCommand("ATS0") &&
      sendElmCommand("ATH0") &&
      sendElmCommand("ATSP6") &&
      sendElmCommand("ATAT1") &&
      sendElmCommand("ATI") &&
      sendElmCommand("ATRV");

  elmReady = ok;
  Serial.println(ok ? "ELM init done" : "ELM init failed");
  return ok;
}
}  // namespace

void setup() {
  Serial.begin(kBaudRate);
  delay(1500);

  if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER) {
    rtcTelemetry = {};
    rtcPendingAgeSeconds = 0;
  }

  if (rtcTelemetry.magic != kTelemetryMagic) {
    rtcTelemetry = {};
    rtcPendingAgeSeconds = 0;
  }

  if (rtcTelemetry.valid && rtcPendingAgeSeconds > 0) {
    rtcTelemetry.ageMinutes += rtcPendingAgeSeconds / 60;
    rtcPendingAgeSeconds %= 60;
  }

  Serial.println();
  Serial.println("MiiMesh BLE OBD adapter");
  Serial.println("Board: Seeed Studio XIAO ESP32-C6");
  Serial.print("Wake cause=");
  Serial.println(static_cast<int>(esp_sleep_get_wakeup_cause()));
  Serial.print("Target addr=");
  Serial.print(kTargetAddress);
  Serial.print("; name=");
  Serial.println(kTargetName);
  Serial.println();

  NimBLEDevice::init("MiiMeshOBD");
  scanner = NimBLEDevice::getScan();
  scanner->setScanCallbacks(&scanCallbacks, true);
  scanner->setActiveScan(kActiveScan);
  scanner->setInterval(100);
  scanner->setWindow(90);
}

void loop() {
  bool freshTelemetry = false;
  bool haveTelemetryToSend = false;

  disconnectClient();

  if (scanForDongle() && connectDongle()) {
    if (initializeElm327() && configureBatteryManagementEcu()) {
      TelemetryData currentTelemetry = {};
      freshTelemetry = pollVehicleMetrics(currentTelemetry);
      if (freshTelemetry) {
        rtcTelemetry = currentTelemetry;
        rtcPendingAgeSeconds = 0;
        printTelemetry(rtcTelemetry, true);
        haveTelemetryToSend = true;
      }
    }
  }

  disconnectClient();

  if (!freshTelemetry) {
    if (rtcTelemetry.magic == kTelemetryMagic && rtcTelemetry.valid) {
      Serial.println("TELEMETRY using cached data");
      printTelemetry(rtcTelemetry, false);
      haveTelemetryToSend = true;
    } else {
      Serial.println("TELEMETRY no fresh or cached data");
      Serial.println("STATUS noOBDdata");
      if (publishMqttPayload(kMqttStatusTopic, "noOBDdata")) {
        Serial.println("SEND wifi=mqtt status=noOBDdata");
      } else {
        Serial.println("SEND mqtt unavailable; retry next wake cycle");
      }
    }
  }

  if (haveTelemetryToSend) {
    if (publishMqttTelemetry(rtcTelemetry)) {
      Serial.println("SEND wifi=mqtt status=ok");
    } else {
      Serial.println("SEND mqtt failed; retry next wake cycle");
    }
  }

  deepSleep();
}
