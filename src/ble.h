#ifndef BLE_H
#define BLE_H

#include <BLE2902.h>
#include <BLEDescriptor.h>
#include <BLEDevice.h>
#include <BLESecurity.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <vector>

BLEServer* pServer      = NULL;
bool deviceConnected    = false;
bool oldDeviceConnected = false;

#define SERVICE_UUID          "ec91d7ab-e87c-48d5-adfa-cc4b2951298a"
#define CHA_SETTINGS          "9d37a346-63d3-4df6-8eee-f0242949f59f"
#define CHA_NAV               "0b11deef-1563-447f-aece-d3dfeb1c1f20"
#define CHA_NAV_TBT_ICON      "d4d8fcca-16b2-4b8e-8ed5-90137c44a8ad"
#define CHA_NAV_TBT_ICON_DESC "d63a466e-5271-4a5d-a942-a34ccdb013d9"
#define CHA_GPS_SPEED         "98b6073a-5cf3-4e73-b6d3-f8e05fa018a9"
#define CHA_MODE              "00000004-0000-0000-0000-000000000000" // пример UUID
#define CHA_GPS_POSITIONS     "7a4b8c2d-9e1f-4a3d-b5c6-7d8e9f0a1b2c"

// typedef void (*OnCharacteristicWriteCallback)(const String& uuid, uint8_t* data, size_t length);
// typedef void (*OnConnectionChangeCallback)(bool connected);

// extern OnCharacteristicWriteCallback onCharacteristicWrite;
// extern OnConnectionChangeCallback onConnectionChange;
void onCharacteristicWrite(const String& uuid, uint8_t* data, size_t length);
void onConnectionChange(bool connected);

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

struct CharacteristicConfig {
	String name;
	String uuid;
	// const uint32_t types;
	BLECharacteristic* bleCharacteristic = nullptr;
};

struct ServiceConfig {
	String name;
	String uuid;
	BLEService* bleService = nullptr;
	std::vector<CharacteristicConfig> characteristics{};

	CharacteristicConfig* findCharacteristicByUuid(const String& uuid) {
		for (auto& item : characteristics) {
			if (item.uuid == uuid)
				return &item;
		}
		return nullptr;
	}
};

struct MyBleServer {
	BLEServer* bleServer = nullptr;
	std::vector<ServiceConfig> services{};

	ServiceConfig* findServiceByUuid(const String& uuid) {
		for (auto& item : services) {
			if (item.uuid == uuid)
				return &item;
		}

		return nullptr;
	}

	CharacteristicConfig* findCharacteristicByUuid(const String& uuid) {
		for (auto& item : services) {
			const auto result = item.findCharacteristicByUuid(uuid);
			if (result)
				return result;
		}

		return nullptr;
	}
};

MyBleServer server;

class ServerCallbacks : public BLEServerCallbacks {
	void onConnect(BLEServer* pServer) {
		Serial.println("Device connected");
		deviceConnected = true;
		onConnectionChange(deviceConnected);
	};

	void onDisconnect(BLEServer* pServer) {
		Serial.println("Device disconnected, start advertising...");
		deviceConnected = false;
		server.bleServer->startAdvertising();
		onConnectionChange(deviceConnected);
	}
};

class CharacteristicWriteCallbacks : public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic* pCharacteristic) {
	    std::string uuidStr = pCharacteristic->getUUID().toString();
		const String uuid = String(uuidStr.c_str());
		const auto characteristicInfo = server.findCharacteristicByUuid(uuid);


		if (!characteristicInfo) {
			Serial.print("Error: No characteristic found with UUID: ");
			Serial.println(uuid);
		}

		Serial.print(characteristicInfo->name);
		Serial.print('=');

		if (pCharacteristic->getLength() > 180) {
			Serial.print("<large ");
			Serial.print(pCharacteristic->getLength());
			Serial.println("B>");
		} else {
			Serial.println(pCharacteristic->getValue().c_str());
		}

		onCharacteristicWrite(uuid, pCharacteristic->getData(), pCharacteristic->getLength());
	}
};

void initBle() {
	ServiceConfig catDriveService;
  catDriveService.name = "CATDRIVE";
  catDriveService.uuid = SERVICE_UUID;
  
  CharacteristicConfig config;
  config.name = "SETTINGS";
  config.uuid = CHA_SETTINGS;
  catDriveService.characteristics.push_back(config);

  CharacteristicConfig config2;
  config2.name = "NAV";
  config2.uuid = CHA_NAV;
  catDriveService.characteristics.push_back(config2);

  CharacteristicConfig config3;
  config3.name = "NAV_ICON";
  config3.uuid = CHA_NAV_TBT_ICON;
  catDriveService.characteristics.push_back(config3);

  CharacteristicConfig config4;
  config4.name = "GPS_SPEED";
  config4.uuid = CHA_GPS_SPEED;
  catDriveService.characteristics.push_back(config4);

    // Второй сервис для GPS данных
    ServiceConfig gpsService;
    gpsService.name = "GPS_SERVICE";
    gpsService.uuid = "7a4b8c2d-9e1f-4a3d-b5c6-7d8e9f0a1b2d"; // Новый UUID сервиса

    // Характеристика для получения GPS координат с телефона
    CharacteristicConfig config5;
    config5.name = "GPS_POSITIONS";
    config5.uuid = CHA_GPS_POSITIONS;
    gpsService.characteristics.push_back(config5);

	CharacteristicConfig config6;
	config6.name = "MODE";
	config6.uuid = CHA_MODE;
	catDriveService.characteristics.push_back(config6);


	// Create the BLE Device
	BLEDevice::init("CatDrive");
	Serial.println(BLEDevice::getMTU());
	BLEDevice::setMTU(517);

	// Add all services
	server.services.push_back(catDriveService);
	server.services.push_back(gpsService);

	server.bleServer = BLEDevice::createServer();
	server.bleServer->setCallbacks(new ServerCallbacks());
	const auto writeCallbacks = new CharacteristicWriteCallbacks();

	// Init services and their characteristic
	for (auto& serviceConfig : server.services) {
//		serviceConfig.bleService = server.bleServer->createService(serviceConfig.uuid, 4 * serviceConfig.characteristics.size());

    serviceConfig.bleService = server.bleServer->createService(
        BLEUUID(serviceConfig.uuid.c_str())    // Правильно: конвертируем String в BLEUUID
    );

		for (auto& characteristicConfig : serviceConfig.characteristics) {
			const auto property = BLECharacteristic::PROPERTY_WRITE;

			characteristicConfig.bleCharacteristic =
		//	serviceConfig.bleService->createCharacteristic(characteristicConfig.uuid, property);
		characteristicConfig.bleCharacteristic =
    serviceConfig.bleService->createCharacteristic(
        BLEUUID(characteristicConfig.uuid.c_str()),  // Конвертируем String в BLEUUID
        property
    );

    characteristicConfig.bleCharacteristic->setCallbacks(writeCallbacks);

		//	const auto desc = new BLE2901();
		//	desc->setDescription(characteristicConfig.name);
		//	characteristicConfig.bleCharacteristic->addDescriptor(desc);
    BLEDescriptor* desc = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    desc->setValue(std::string(characteristicConfig.name.c_str()));
    characteristicConfig.bleCharacteristic->addDescriptor(desc);

		}

		serviceConfig.bleService->start();
	}


 // Выводим все характеристики после старта
    for (auto& service : server.services) {
        Serial.println("=== Final characteristic list ===");
        for (auto& characteristic : service.characteristics) {
            if (characteristic.bleCharacteristic) {
                Serial.print("✓ ");
            } else {
                Serial.print("✗ ");
            }
            Serial.print(characteristic.name.c_str());
            Serial.print(" - ");
            Serial.println(characteristic.uuid.c_str());
        }
    }



	server.bleServer->getAdvertising()->start();
}


void notifyCharacteristic(const String& uuid, uint8_t* data, size_t length) {
	const auto pCharacteristic = server.findCharacteristicByUuid(uuid)->bleCharacteristic;
	if (!pCharacteristic) {
		Serial.print("Error: No characteristic found with UUID: ");
		Serial.println(uuid);
		return;
	}

	pCharacteristic->setValue(data, length);
	pCharacteristic->notify();
}

#endif // BLE_H