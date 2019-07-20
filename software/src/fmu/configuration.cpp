/*
configuration.cpp
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2018 Bolder Flight Systems
Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "configuration.h"
#include "fmu_messages.h"

/* Load configuration from EEPROM */
void AircraftConfiguration::Load() {
  uint16_t EepromLocation = 0;
  Serial.println("Loading configuration...");
  config_.BfsAddr = EEPROM.read(EepromLocation);
  Serial.print("\tBFS Address: ");
  Serial.println(config_.BfsAddr);
  Serial.println("done!");
}

/* Update configuration structure from JSON payloads */
void AircraftConfiguration::Update(const char* JsonString,AircraftMission *AircraftMissionPtr,AircraftSensors *AircraftSensorsPtr,ControlLaws *ControlLawsPtr,AircraftEffectors *AircraftEffectorsPtr,DefinitionTree *DefinitionTreePtr) {
  DynamicJsonBuffer ConfigBuffer;
  std::vector<char> buffer;

  Serial.print("AircraftConfiguration Parsing: ");
  Serial.print(JsonString);

  JsonObject &Config = ConfigBuffer.parseObject(JsonString);
  buffer.resize(ConfigBuffer.size());

  Serial.print("\tSuccess: ");
  Serial.println(Config.success());

  if (Config.success()) {
    if (Config.containsKey("Sensors")) {
      Serial.println("AircraftConfiguration::Update() [Sensors] we should never be here!");
    }
    if (Config.containsKey("Control")) {
      JsonObject &Control = Config["Control"];
      Control.printTo(buffer.data(),buffer.size());
      ControlLawsPtr->Configure(buffer.data(),DefinitionTreePtr);
    }
    if (Config.containsKey("Effectors")) {
      Serial.println("AircraftConfiguration::Update() [Effectors] we should never be here!");
    }
    if (Config.containsKey("Mission-Manager")) {
      Serial.println("AircraftConfiguration::Update() [Mission-Manager] we should never be here!");
    }
  }
  else {
    Serial.println("ERROR: FMU Config Message Failed to Parse: ");
    Serial.println(JsonString);
  }
}

/* Update configuration structure from JSON payloads */
bool AircraftConfiguration::Update(uint8_t id, uint8_t address, std::vector<uint8_t> *Payload, AircraftMission *AircraftMissionPtr,AircraftSensors *AircraftSensorsPtr,ControlLaws *ControlLawsPtr,AircraftEffectors *AircraftEffectorsPtr,DefinitionTree *DefinitionTreePtr) {
  if ( AircraftSensorsPtr->UpdateConfig(id, address, Payload, DefinitionTreePtr) ) {
    return true;
  } else if ( AircraftEffectorsPtr->UpdateConfig(id, address, Payload, DefinitionTreePtr) ) {
    return true;
  } else if ( AircraftMissionPtr->UpdateConfig(id, Payload, DefinitionTreePtr) ) {
    return true;
  // fixme: move control and mission processing messages here as well
  // } else if (control message) {
  } else {
    Serial.print("Unknown config msg in fmu/configuration.cpp:"); Serial.println(id);
  }
  return false;
}
