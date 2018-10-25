/*
fmu.cxx
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

#include "fmu.h"

#include <string>
using std::to_string;

/* Opens port to communicate with FMU. */
void FlightManagementUnit::Begin() {
  if ((FmuFileDesc_=open(Port_.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK))<0) {
    throw std::runtime_error(std::string("ERROR")+RootPath_+std::string(": UART failed to open."));
  }
  struct termios Options;
  tcgetattr(FmuFileDesc_,&Options);
  Options.c_cflag = Baud_ | CS8 | CREAD | CLOCAL;
  Options.c_iflag = IGNPAR;
  Options.c_oflag = 0;
  Options.c_lflag = 0;
  Options.c_cc[VTIME] = 0;
  Options.c_cc[VMIN] = 0;
  tcflush(FmuFileDesc_,TCIFLUSH);
  tcsetattr(FmuFileDesc_,TCSANOW,&Options);
  fcntl(FmuFileDesc_,F_SETFL,O_NONBLOCK);
}

/* Updates FMU configuration given a JSON value and registers data with global defs */
void FlightManagementUnit::Configure(const rapidjson::Value& Config) {
  std::vector<uint8_t> Payload;

  // switch FMU to configuration mode
  SendModeCommand(kConfigMode);

  // clear the serial buffer
  while ((read(FmuFileDesc_,&RxByte_,sizeof(RxByte_)))>0) {}

  // configure FMU sensors
  if (Config.HasMember("Sensors")) {
    std::cout << "\t\tSending Sensors config to FMU..." << std::flush;
    ConfigureSensors(Config["Sensors"]);
    std::cout << "done!" << std::endl;
  }

  // configuring FMU mission manager
  if (Config.HasMember("Mission-Manager")) {
    std::cout << "\t\tSending Mission-Manager config to FMU..." << std::flush;
    ConfigureMissionManager(Config["Mission-Manager"]);
    std::cout << "done!" << std::endl;
  }

  // configuring FMU control laws
  if (Config.HasMember("Control")) {
    std::cout << "\t\tSending Control config to FMU..." << std::flush;
    ConfigureControlLaws(Config["Control"]);
    std::cout << "done!" << std::endl;
  }

  // configuring FMU effectors
  if (Config.HasMember("Effectors")) {
    std::cout << "\t\tSending Effector config to FMU..." << std::flush;
    ConfigureEffectors(Config["Effectors"]);
    std::cout << "done!" << std::endl;
  }

  // switch FMU to run mode
  SendModeCommand(kRunMode);
  sleep(2);

  // get the updated configuration from the sensor meta data
  std::cout << "\t\tReading Sensors config back from FMU..." << std::flush;
  size_t i=0;
  while(i < 100) {
    if (ReceiveSensorData()) {
      i++;
    }
  }
  std::cout << "done!" << std::endl;

  // register sensor data with global definition tree
  if (Config.HasMember("Sensors")) {
    std::cout << "\t\tRegistering Sensors with DefinitionTree..." << std::flush;
    RegisterSensors(Config["Sensors"]);
    std::cout << "done!" << std::endl;
  }
}

/* Sends a mode command to the FMU */
void FlightManagementUnit::SendModeCommand(Mode mode) {
  std::vector<uint8_t> Payload;
  Payload.push_back((uint8_t)mode);
  SendMessage(Message::kModeCommand,Payload);
}

/* Receive sensor data from FMU */
bool FlightManagementUnit::ReceiveSensorData() {
  Message message;
  std::vector<uint8_t> Payload;
  size_t PayloadLocation = 0;
  if (ReceiveMessage(&message,&Payload)) {
    if (message == kSensorData) {
      // meta data
      uint8_t AcquireInternalData,NumberPwmVoltageSensor,NumberSbusVoltageSensor,NumberMpu9250Sensor,NumberBme280Sensor,NumberuBloxSensor,NumberSwiftSensor,NumberAms5915Sensor,NumberSbusSensor,NumberAnalogSensor;
      memcpy(&AcquireInternalData,Payload.data()+PayloadLocation,sizeof(AcquireInternalData));
      PayloadLocation += sizeof(AcquireInternalData);
      memcpy(&NumberPwmVoltageSensor,Payload.data()+PayloadLocation,sizeof(NumberPwmVoltageSensor));
      PayloadLocation += sizeof(NumberPwmVoltageSensor);
      memcpy(&NumberSbusVoltageSensor,Payload.data()+PayloadLocation,sizeof(NumberSbusVoltageSensor));
      PayloadLocation += sizeof(NumberSbusVoltageSensor);
      memcpy(&NumberMpu9250Sensor,Payload.data()+PayloadLocation,sizeof(NumberMpu9250Sensor));
      PayloadLocation += sizeof(NumberMpu9250Sensor);
      memcpy(&NumberBme280Sensor,Payload.data()+PayloadLocation,sizeof(NumberBme280Sensor));
      PayloadLocation += sizeof(NumberBme280Sensor);
      memcpy(&NumberuBloxSensor,Payload.data()+PayloadLocation,sizeof(NumberuBloxSensor));
      PayloadLocation += sizeof(NumberuBloxSensor);
      memcpy(&NumberSwiftSensor,Payload.data()+PayloadLocation,sizeof(NumberSwiftSensor));
      PayloadLocation += sizeof(NumberSwiftSensor);
      memcpy(&NumberAms5915Sensor,Payload.data()+PayloadLocation,sizeof(NumberAms5915Sensor));
      PayloadLocation += sizeof(NumberAms5915Sensor);
      memcpy(&NumberSbusSensor,Payload.data()+PayloadLocation,sizeof(NumberSbusSensor));
      PayloadLocation += sizeof(NumberSbusSensor);
      memcpy(&NumberAnalogSensor,Payload.data()+PayloadLocation,sizeof(NumberAnalogSensor));
      PayloadLocation += sizeof(NumberAnalogSensor);
      // resize data buffers
      if (AcquireInternalData & 0x01) {
        SensorData_.Time_us.resize(1);
        SensorNodes_.Time_us.resize(1);
      }
      if (AcquireInternalData & 0x02) {
        SensorData_.InternalMpu9250.resize(1);
        SensorNodes_.InternalMpu9250.resize(1);
      }
      if (AcquireInternalData & 0x04) {
        SensorData_.InternalBme280.resize(1);
        SensorNodes_.InternalBme280.resize(1);
      }
      if (AcquireInternalData & 0x08) {
        SensorData_.InputVoltage_V.resize(1);
        SensorNodes_.input_volts.resize(1);
      }
      if (AcquireInternalData & 0x10) {
        SensorData_.RegulatedVoltage_V.resize(1);
        SensorNodes_.reg_volts.resize(1);
      }
      SensorData_.PwmVoltage_V.resize(NumberPwmVoltageSensor);
      SensorData_.SbusVoltage_V.resize(NumberSbusVoltageSensor);
      SensorData_.Mpu9250.resize(NumberMpu9250Sensor);
      SensorData_.Bme280.resize(NumberBme280Sensor);
      SensorData_.uBlox.resize(NumberuBloxSensor);
      SensorData_.Swift.resize(NumberSwiftSensor);
      SensorData_.Ams5915.resize(NumberAms5915Sensor);
      SensorData_.Sbus.resize(NumberSbusSensor);
      SensorData_.Analog.resize(NumberAnalogSensor);
      SensorNodes_.pwm_volts.resize(NumberPwmVoltageSensor);
      SensorNodes_.sbus_volts.resize(NumberSbusVoltageSensor);
      SensorNodes_.Mpu9250.resize(NumberMpu9250Sensor);
      SensorNodes_.Bme280.resize(NumberBme280Sensor);
      SensorNodes_.uBlox.resize(NumberuBloxSensor);
      SensorNodes_.Swift.resize(NumberSwiftSensor);
      SensorNodes_.Ams5915.resize(NumberAms5915Sensor);
      SensorNodes_.Sbus.resize(NumberSbusSensor);
      SensorNodes_.Analog.resize(NumberAnalogSensor);

      // sensor data
      memcpy(SensorData_.Time_us.data(),Payload.data()+PayloadLocation,SensorData_.Time_us.size()*sizeof(SensorData_.Time_us[0]));
      PayloadLocation += SensorData_.Time_us.size()*sizeof(SensorData_.Time_us[0]);
      memcpy(SensorData_.InternalMpu9250.data(),Payload.data()+PayloadLocation,SensorData_.InternalMpu9250.size()*sizeof(InternalMpu9250SensorData));
      PayloadLocation += SensorData_.InternalMpu9250.size()*sizeof(InternalMpu9250SensorData);
      memcpy(SensorData_.InternalBme280.data(),Payload.data()+PayloadLocation,SensorData_.InternalBme280.size()*sizeof(InternalBme280SensorData));
      PayloadLocation += SensorData_.InternalBme280.size()*sizeof(InternalBme280SensorData);
      memcpy(SensorData_.InputVoltage_V.data(),Payload.data()+PayloadLocation,SensorData_.InputVoltage_V.size()*sizeof(SensorData_.InputVoltage_V[0]));
      PayloadLocation += SensorData_.InputVoltage_V.size()*sizeof(SensorData_.InputVoltage_V[0]);
      memcpy(SensorData_.RegulatedVoltage_V.data(),Payload.data()+PayloadLocation,SensorData_.RegulatedVoltage_V.size()*sizeof(SensorData_.RegulatedVoltage_V[0]));
      PayloadLocation += SensorData_.RegulatedVoltage_V.size()*sizeof(SensorData_.RegulatedVoltage_V[0]);
      memcpy(SensorData_.PwmVoltage_V.data(),Payload.data()+PayloadLocation,SensorData_.PwmVoltage_V.size()*sizeof(SensorData_.PwmVoltage_V[0]));
      PayloadLocation += SensorData_.PwmVoltage_V.size()*sizeof(SensorData_.PwmVoltage_V[0]);
      memcpy(SensorData_.SbusVoltage_V.data(),Payload.data()+PayloadLocation,SensorData_.SbusVoltage_V.size()*sizeof(SensorData_.SbusVoltage_V[0]));
      PayloadLocation += SensorData_.SbusVoltage_V.size()*sizeof(SensorData_.SbusVoltage_V[0]);
      memcpy(SensorData_.Mpu9250.data(),Payload.data()+PayloadLocation,SensorData_.Mpu9250.size()*sizeof(Mpu9250SensorData));
      PayloadLocation += SensorData_.Mpu9250.size()*sizeof(Mpu9250SensorData);
      memcpy(SensorData_.Bme280.data(),Payload.data()+PayloadLocation,SensorData_.Bme280.size()*sizeof(Bme280SensorData));
      PayloadLocation += SensorData_.Bme280.size()*sizeof(Bme280SensorData);
      memcpy(SensorData_.uBlox.data(),Payload.data()+PayloadLocation,SensorData_.uBlox.size()*sizeof(uBloxSensorData));
      PayloadLocation += SensorData_.uBlox.size()*sizeof(uBloxSensorData);
      memcpy(SensorData_.Swift.data(),Payload.data()+PayloadLocation,SensorData_.Swift.size()*sizeof(SwiftSensorData));
      PayloadLocation += SensorData_.Swift.size()*sizeof(SwiftSensorData);
      memcpy(SensorData_.Ams5915.data(),Payload.data()+PayloadLocation,SensorData_.Ams5915.size()*sizeof(Ams5915SensorData));
      PayloadLocation += SensorData_.Ams5915.size()*sizeof(Ams5915SensorData);
      memcpy(SensorData_.Sbus.data(),Payload.data()+PayloadLocation,SensorData_.Sbus.size()*sizeof(SbusSensorData));
      PayloadLocation += SensorData_.Sbus.size()*sizeof(SbusSensorData);
      memcpy(SensorData_.Analog.data(),Payload.data()+PayloadLocation,SensorData_.Analog.size()*sizeof(AnalogSensorData));
      PayloadLocation += SensorData_.Analog.size()*sizeof(AnalogSensorData);

      // copy the incoming sensor data into the definition tree
      PublishSensors();
      
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

/* Sends effector commands to FMU */
void FlightManagementUnit::SendEffectorCommands(std::vector<float> Commands) {
  std::vector<uint8_t> Payload;
  Payload.resize(Commands.size()*sizeof(float));
  memcpy(Payload.data(),Commands.data(),Payload.size());
  SendMessage(kEffectorCommand,Payload);
}

/* Configures the FMU sensors */
void FlightManagementUnit::ConfigureSensors(const rapidjson::Value& Config) {
  std::vector<uint8_t> Payload;
  assert(Config.IsArray());
  for (size_t i=0; i < Config.Size(); i++) {
    const rapidjson::Value& Sensor = Config[i];
    if (Sensor.HasMember("Type")) {
      Payload.clear();
      rapidjson::StringBuffer StringBuff;
      rapidjson::Writer<rapidjson::StringBuffer> Writer(StringBuff);
      Sensor.Accept(Writer);
      std::string OutputString = StringBuff.GetString();
      std::string ConfigString = std::string("{\"Sensors\":[") + OutputString + std::string("]}");
      for (size_t j=0; j < ConfigString.size(); j++) {
        Payload.push_back((uint8_t)ConfigString[j]);
      }
      SendMessage(Message::kConfigMesg,Payload);
      sleep(6);
    }
  }
}

/* Configures the FMU mission manager */
void FlightManagementUnit::ConfigureMissionManager(const rapidjson::Value& Config) {
  std::vector<uint8_t> Payload;
  rapidjson::StringBuffer StringBuff;
  rapidjson::Writer<rapidjson::StringBuffer> Writer(StringBuff);
  Config.Accept(Writer);
  std::string OutputString = StringBuff.GetString();
  std::string ConfigString = std::string("{\"Mission-Manager\":") + OutputString + std::string("}");
  for (size_t j=0; j < ConfigString.size(); j++) {
    Payload.push_back((uint8_t)ConfigString[j]);
  }
  SendMessage(Message::kConfigMesg,Payload);
  sleep(6);
}

/* Configures the FMU control laws */
void FlightManagementUnit::ConfigureControlLaws(const rapidjson::Value& Config) {
  std::vector<uint8_t> Payload;
  if (Config.HasMember("Fmu")) {
    if (Config.HasMember(Config["Fmu"].GetString())) {
      rapidjson::StringBuffer FmuStringBuff;
      rapidjson::Writer<rapidjson::StringBuffer> FmuWriter(FmuStringBuff);
      const rapidjson::Value& Fmu = Config["Fmu"];
      Fmu.Accept(FmuWriter);
      std::string FmuString = FmuStringBuff.GetString();
      rapidjson::StringBuffer CntrlStringBuff;
      rapidjson::Writer<rapidjson::StringBuffer> CntrlWriter(CntrlStringBuff);
      const rapidjson::Value& Cntrl = Config[Config["Fmu"].GetString()];
      Cntrl.Accept(CntrlWriter);
      std::string CntrlString = CntrlStringBuff.GetString();
      std::string ConfigString = std::string("{\"Control\":{") + std::string("\"Fmu\":") + FmuString + std::string(",")
        + std::string("\"") + Config["Fmu"].GetString() + std::string("\":") + CntrlString + std::string("}}");
      for (size_t j=0; j < ConfigString.size(); j++) {
        Payload.push_back((uint8_t)ConfigString[j]);
      }
      SendMessage(Message::kConfigMesg,Payload);
      sleep(6);
    }
  }
}

/* Configures the FMU effectors */
void FlightManagementUnit::ConfigureEffectors(const rapidjson::Value& Config) {
  std::vector<uint8_t> Payload;
  assert(Config.IsArray());
  for (size_t i=0; i < Config.Size(); i++) {
    const rapidjson::Value& Effector = Config[i];
    Payload.clear();
    rapidjson::StringBuffer StringBuff;
    rapidjson::Writer<rapidjson::StringBuffer> Writer(StringBuff);
    Effector.Accept(Writer);
    std::string OutputString = StringBuff.GetString();
    std::string ConfigString = std::string("{\"Effectors\":[") + OutputString + std::string("]}");
    for (size_t j=0; j < ConfigString.size(); j++) {
      Payload.push_back((uint8_t)ConfigString[j]);
    }
    SendMessage(Message::kConfigMesg,Payload);
    sleep(4);
  }
}

/* Registers sensor data with global definition tree */
void FlightManagementUnit::RegisterSensors(const rapidjson::Value& Config) {
  for (size_t i=0; i < SensorData_.Time_us.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"Time",i);
    SensorNodes_.Time_us[i] = deftree.initElement(Path, "Flight management unit time, us", true, false);
  }
  for (size_t i=0; i < SensorData_.InternalMpu9250.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"InternalMpu9250",i);
    SensorNodes_.InternalMpu9250[i].ax = deftree.initElement(Path+"/AccelX_mss", "Flight management unit MPU-9250 X accelerometer, corrected for installation rotation, m/s/s", true, false);
    SensorNodes_.InternalMpu9250[i].ay = deftree.initElement(Path+"/AccelY_mss", "Flight management unit MPU-9250 Y accelerometer, corrected for installation rotation, m/s/s", true, false);
    SensorNodes_.InternalMpu9250[i].az = deftree.initElement(Path+"/AccelZ_mss", "Flight management unit MPU-9250 Z accelerometer, corrected for installation rotation, m/s/s", true, false);
    SensorNodes_.InternalMpu9250[i].p = deftree.initElement(Path+"/GyroX_rads", "Flight management unit MPU-9250 X gyro, corrected for installation rotation, rad/s", true, false);
    SensorNodes_.InternalMpu9250[i].q = deftree.initElement(Path+"/GyroY_rads", "Flight management unit MPU-9250 Y gyro, corrected for installation rotation, rad/s", true, false);
    SensorNodes_.InternalMpu9250[i].r = deftree.initElement(Path+"/GyroZ_rads", "Flight management unit MPU-9250 Z gyro, corrected for installation rotation, rad/s", true, false);
    SensorNodes_.InternalMpu9250[i].hx = deftree.initElement(Path+"/MagX_uT", "Flight management unit MPU-9250 X magnetometer, corrected for installation rotation, uT", true, false);
    SensorNodes_.InternalMpu9250[i].hy = deftree.initElement(Path+"/MagY_uT", "Flight management unit MPU-9250 Y magnetometer, corrected for installation rotation, uT", true, false);
    SensorNodes_.InternalMpu9250[i].hz = deftree.initElement(Path+"/MagZ_uT", "Flight management unit MPU-9250 Z magnetometer, corrected for installation rotation, uT", true, false);
    SensorNodes_.InternalMpu9250[i].temp = deftree.initElement(Path+"/Temperature_C", "Flight management unit MPU-9250 temperature, C", true, false);
  }
  for (size_t i=0; i < SensorData_.InternalBme280.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"InternalBme280",i);
    SensorNodes_.InternalBme280[i].press = deftree.initElement(Path+"/Pressure_Pa", "Flight management unit BME-280 static pressure, Pa", true, false);
    SensorNodes_.InternalBme280[i].temp = deftree.initElement(Path+"/Temperature_C", "Flight management unit BME-280 temperature, C", true, false);
    SensorNodes_.InternalBme280[i].hum = deftree.initElement(Path+"/Humidity_RH", "Flight management unit BME-280 percent relative humidity", true, false);
  }
  for (size_t i=0; i < SensorData_.InputVoltage_V.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"InputVoltage",i);
    SensorNodes_.input_volts[i] = deftree.initElement(Path, "Flight management unit input voltage, V", true, false);
  }
  for (size_t i=0; i < SensorData_.RegulatedVoltage_V.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"RegulatedVoltage",i);
    SensorNodes_.reg_volts[i] = deftree.initElement(Path, "Flight management unit regulated voltage, V", true, false);
  }
  for (size_t i=0; i < SensorData_.PwmVoltage_V.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"PwmVoltage",i);
    SensorNodes_.pwm_volts[i] = deftree.initElement(Path, "Flight management unit PWM servo voltage, V", true, false);
  }
  for (size_t i=0; i < SensorData_.SbusVoltage_V.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"SbusVoltage",i);
    SensorNodes_.sbus_volts[i] = deftree.initElement(Path, "Flight management unit SBUS servo voltage, V", true, false);
  }
  for (size_t i=0; i < SensorData_.Mpu9250.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"Mpu9250",i);
    SensorNodes_.Mpu9250[i].status = deftree.initElement(Path+"/Status", "MPU-9250_" + to_string(i) + " read status, positive if a good sensor read", true, false);
    SensorNodes_.Mpu9250[i].ax = deftree.initElement(Path+"/AccelX_mss", "MPU-9250_" + to_string(i) + " X accelerometer, corrected for installation rotation, m/s/s", true, false);
    SensorNodes_.Mpu9250[i].ay = deftree.initElement(Path+"/AccelY_mss", "MPU-9250_" + to_string(i) + " Y accelerometer, corrected for installation rotation, m/s/s", true, false);
    SensorNodes_.Mpu9250[i].az = deftree.initElement(Path+"/AccelZ_mss", "MPU-9250_" + to_string(i) + " Z accelerometer, corrected for installation rotation, m/s/s", true, false);
    SensorNodes_.Mpu9250[i].p = deftree.initElement(Path+"/GyroX_rads", "MPU-9250_" + to_string(i) + " X gyro, corrected for installation rotation, rad/s", true, false);
    SensorNodes_.Mpu9250[i].q = deftree.initElement(Path+"/GyroY_rads", "MPU-9250_" + to_string(i) + " Y gyro, corrected for installation rotation, rad/s", true, false);
    SensorNodes_.Mpu9250[i].r = deftree.initElement(Path+"/GyroZ_rads", "MPU-9250_" + to_string(i) + " Z gyro, corrected for installation rotation, rad/s", true, false);
    SensorNodes_.Mpu9250[i].hx = deftree.initElement(Path+"/MagX_uT", "MPU-9250_" + to_string(i) + " X magnetometer, corrected for installation rotation, uT", true, false);
    SensorNodes_.Mpu9250[i].hy = deftree.initElement(Path+"/MagY_uT", "MPU-9250_" + to_string(i) + " Y magnetometer, corrected for installation rotation, uT", true, false);
    SensorNodes_.Mpu9250[i].hz = deftree.initElement(Path+"/MagZ_uT", "MPU-9250_" + to_string(i) + " Z magnetometer, corrected for installation rotation, uT", true, false);
    SensorNodes_.Mpu9250[i].temp = deftree.initElement(Path+"/Temperature_C", "MPU-9250_" + to_string(i) + " temperature, C", true, false);
  }
  for (size_t i=0; i < SensorData_.Bme280.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"Bme280",i);
    SensorNodes_.Bme280[i].status = deftree.initElement(Path+"/Status", "BME-280_" + to_string(i) + " read status, positive if a good sensor read", true, false);
    SensorNodes_.Bme280[i].press = deftree.initElement(Path+"/Pressure_Pa", "BME-280_" + to_string(i) + " static pressure, Pa", true, false);
    SensorNodes_.Bme280[i].temp = deftree.initElement(Path+"/Temperature_C", "BME-280_" + to_string(i) + " temperature, C", true, false);
    SensorNodes_.Bme280[i].hum = deftree.initElement(Path+"/Humidity_RH", "BME-280_" + to_string(i) + " percent relative humidity", true, false);
  }
  for (size_t i=0; i < SensorData_.uBlox.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"uBlox",i);
    SensorNodes_.uBlox[i].fix = deftree.initElement(Path+"/Fix", "uBlox_" + to_string(i) + " fix status, true for 3D fix only", true, false);
    SensorNodes_.uBlox[i].sats = deftree.initElement(Path+"/NumberSatellites", "uBlox_" + to_string(i) + " number of satellites used in solution", true, false);
    SensorNodes_.uBlox[i].tow = deftree.initElement(Path+"/TOW", "uBlox_" + to_string(i) + " GPS time of the navigation epoch", true, false);
    SensorNodes_.uBlox[i].year = deftree.initElement(Path+"/Year", "uBlox_" + to_string(i) + " UTC year", true, false);
    SensorNodes_.uBlox[i].month = deftree.initElement(Path+"/Month", "uBlox_" + to_string(i) + " UTC month", true, false);
    SensorNodes_.uBlox[i].day = deftree.initElement(Path+"/Day", "uBlox_" + to_string(i) + " UTC day", true, false);
    SensorNodes_.uBlox[i].hour = deftree.initElement(Path+"/Hour", "uBlox_" + to_string(i) + " UTC hour", true, false);
    SensorNodes_.uBlox[i].min = deftree.initElement(Path+"/Minute", "uBlox_" + to_string(i) + " UTC minute", true, false);
    SensorNodes_.uBlox[i].sec = deftree.initElement(Path+"/Second", "uBlox_" + to_string(i) + " UTC second", true, false);
    SensorNodes_.uBlox[i].lat = deftree.initElement(Path+"/Latitude_rad", "uBlox_" + to_string(i) + " latitude, rad", true, false);
    SensorNodes_.uBlox[i].lon = deftree.initElement(Path+"/Longitude_rad", "uBlox_" + to_string(i) + " longitude, rad", true, false);
    SensorNodes_.uBlox[i].alt = deftree.initElement(Path+"/Altitude_m", "uBlox_" + to_string(i) + " altitude above mean sea level, m", true, false);
    SensorNodes_.uBlox[i].vn = deftree.initElement(Path+"/NorthVelocity_ms", "uBlox_" + to_string(i) + " north velocity, m/s", true, false);
    SensorNodes_.uBlox[i].ve = deftree.initElement(Path+"/EastVelocity_ms", "uBlox_" + to_string(i) + " east velocity, m/s", true, false);
    SensorNodes_.uBlox[i].vd = deftree.initElement(Path+"/DownVelocity_ms", "uBlox_" + to_string(i) + " down velocity, m/s", true, false);
    SensorNodes_.uBlox[i].horiz_acc = deftree.initElement(Path+"/HorizontalAccuracy_m", "uBlox_" + to_string(i) + " horizontal accuracy estimate, m", true, false);
    SensorNodes_.uBlox[i].vert_acc = deftree.initElement(Path+"/VerticalAccuracy_m", "uBlox_" + to_string(i) + " vertical accuracy estimate, m", true, false);
    SensorNodes_.uBlox[i].vel_acc = deftree.initElement(Path+"/VelocityAccuracy_ms", "uBlox_" + to_string(i) + " velocity accuracy estimate, m/s", true, false);
    SensorNodes_.uBlox[i].pdop = deftree.initElement(Path+"/pDOP", "uBlox_" + to_string(i) + " position dilution of precision", true, false);
  }
  for (size_t i=0; i < SensorData_.Swift.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"Swift",i);
    SensorNodes_.Swift[i].Static.status = deftree.initElement(Path+"/Static/Status", "Swift_" + to_string(i) + " static pressure read status, positive if a good sensor read", true, false);
    SensorNodes_.Swift[i].Static.press = deftree.initElement(Path+"/Static/Pressure_Pa", "Swift_" + to_string(i) + " static pressure, Pa", true, false);
    SensorNodes_.Swift[i].Static.temp = deftree.initElement(Path+"/Static/Temperature_C", "Swift_" + to_string(i) + " static pressure transducer temperature, C", true, false);
    SensorNodes_.Swift[i].Differential.status = deftree.initElement(Path+"/Differential/Status", "Swift_" + to_string(i) + " differential pressure read status, positive if a good sensor read", true, false);
    SensorNodes_.Swift[i].Differential.press = deftree.initElement(Path+"/Differential/Pressure_Pa", "Swift_" + to_string(i) + " differential pressure, Pa", true, false);
    SensorNodes_.Swift[i].Differential.temp = deftree.initElement(Path+"/Differential/Temperature_C", "Swift_" + to_string(i) + " differential pressure transducer temperature, C", true, false);
  }
  for (size_t i=0; i < SensorData_.Ams5915.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"Ams5915",i);
    SensorNodes_.Ams5915[i].status = deftree.initElement(Path+"/Status", "AMS-5915_" + to_string(i) + " read status, positive if a good sensor read", true, false);
    SensorNodes_.Ams5915[i].press = deftree.initElement(Path+"/Pressure_Pa", "AMS-5915_" + to_string(i) + " pressure, Pa", true, false);
    SensorNodes_.Ams5915[i].temp = deftree.initElement(Path+"/Temperature_C", "AMS-5915_" + to_string(i) + " pressure transducer temperature, C", true, false);
  }
  for (size_t i=0; i < SensorData_.Sbus.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"Sbus",i);
    SensorNodes_.Sbus[i].failsafe = deftree.initElement(Path+"/FailSafe", "SBUS_" + to_string(i) + " fail safe status", true, false);
    SensorNodes_.Sbus[i].lost_frames = deftree.initElement(Path+"/LostFrames", "SBUS_" + to_string(i) + " number of lost frames", true, false);
    for (size_t j=0; j < 16; j++) {
      SensorNodes_.Sbus[i].ch[j] = deftree.initElement(Path+"/Channels/"+to_string(j), "SBUS_" + to_string(i) + " channel" + to_string(j) + " normalized value", true, false);
    }
  }
  for (size_t i=0; i < SensorData_.Analog.size(); i++) {
    string Path = RootPath_+"/"+GetSensorOutputName(Config,"Analog",i);
    SensorNodes_.Analog[i].volt = deftree.initElement(Path+"/Voltage_V", "Analog_" + to_string(i) + " measured voltage, V", true, false);
    SensorNodes_.Analog[i].val = deftree.initElement(Path+"/CalibratedValue", "Analog_" + to_string(i) + " calibrated value", true, false);
  }
}

/* Gets the sensor output name from JSON config given the "Type" and index */
std::string FlightManagementUnit::GetSensorOutputName(const rapidjson::Value& Config,std::string Key,size_t index) {
  size_t k=0;
  assert(Config.IsArray());
  for (size_t i=0; i < Config.Size(); i++) {
    const rapidjson::Value& Sensor = Config[i];
    if (Sensor.HasMember("Type")) {
      if (Sensor["Type"].GetString() == Key) {
        if (k == index) {
          return Sensor["Output"].GetString();
        }
        k++;
      }
      if (Sensor["Type"] == "Node") {
        const rapidjson::Value& Node = Sensor;
        if (Node.HasMember("Sensors")) {
          const rapidjson::Value& NodeSensors = Node["Sensors"];
          assert(NodeSensors.IsArray());
          for (size_t j=0; j < NodeSensors.Size(); j++) {
            const rapidjson::Value& NodeSensor = NodeSensors[j];
            if (NodeSensor.HasMember("Type")) {
              if (NodeSensor["Type"].GetString() == Key) {
                if (k == index) {
                  return NodeSensor["Output"].GetString();
                }
                k++;
              }
            }
          }
        }
      }
    }
  }
  return "";
}

/* Send a BFS Bus message. */
void FlightManagementUnit::SendMessage(Message message,std::vector<uint8_t> &Payload) {
  // check that the payload length is within the maximum buffer size
  if (Payload.size() < (kUartBufferMaxSize-headerLength_-checksumLength_)) {
    // header
    Buffer_[0] = header_[0];
    Buffer_[1] = header_[1];
    // message ID
    Buffer_[2] = (uint8_t)message;
    // payload length
    Buffer_[3] = Payload.size() & 0xff;
    Buffer_[4] = Payload.size() >> 8;
    // payload
    std::memcpy(Buffer_+headerLength_,Payload.data(),Payload.size());
    // checksum
    CalcChecksum((size_t)(Payload.size()+headerLength_),Buffer_,Checksum_);
    Buffer_[Payload.size()+headerLength_] = Checksum_[0];
    Buffer_[Payload.size()+headerLength_+1] = Checksum_[1];
    // transmit
    WritePort(Buffer_,Payload.size()+headerLength_+checksumLength_);
  }
}

/* Receive a BFS Bus message. */
bool FlightManagementUnit::ReceiveMessage(Message *message,std::vector<uint8_t> *Payload) {
  int count;
  while ((count=read(FmuFileDesc_,&RxByte_,sizeof(RxByte_)))>0) {
    // header
    if (ParserState_ < 2) {
      if (RxByte_ == header_[ParserState_]) {
        Buffer_[ParserState_] = RxByte_;
        ParserState_++;
      }
    // length
    } else if (ParserState_ == 3) {
      LengthBuffer_[0] = RxByte_;
      Buffer_[ParserState_] = RxByte_;
      ParserState_++;
    } else if (ParserState_ == 4) {
      LengthBuffer_[1] = RxByte_;
      Length_ = ((uint16_t)LengthBuffer_[1] << 8) | LengthBuffer_[0];
      if (Length_ > (kUartBufferMaxSize-headerLength_-checksumLength_)) {
        ParserState_ = 0;
        LengthBuffer_[0] = 0;
        LengthBuffer_[1] = 0;
        Length_ = 0;
        Checksum_[0] = 0;
        Checksum_[1] = 0;
        return false;
      }
      Buffer_[ParserState_] = RxByte_;
      ParserState_++;
    // message ID and payload
    } else if (ParserState_ < (Length_ + headerLength_)) {
      Buffer_[ParserState_] = RxByte_;
      ParserState_++;
    // checksum 0
    } else if (ParserState_ == (Length_ + headerLength_)) {
      CalcChecksum(Length_ + headerLength_,Buffer_,Checksum_);
      if (RxByte_ == Checksum_[0]) {
        ParserState_++;
      } else {
        ParserState_ = 0;
        LengthBuffer_[0] = 0;
        LengthBuffer_[1] = 0;
        Length_ = 0;
        Checksum_[0] = 0;
        Checksum_[1] = 0;
        return false;
      }
    // checksum 1
    } else if (ParserState_ == (Length_ + headerLength_ + 1)) {
      if (RxByte_ == Checksum_[1]) {
        // message ID
        *message = (Message) Buffer_[2];
        // payload size
        Payload->resize(Length_);
        // payload
        std::memcpy(Payload->data(),Buffer_+headerLength_,Length_);
        ParserState_ = 0;
        LengthBuffer_[0] = 0;
        LengthBuffer_[1] = 0;
        Length_ = 0;
        Checksum_[0] = 0;
        Checksum_[1] = 0;
        return true;
      } else {
        ParserState_ = 0;
        LengthBuffer_[0] = 0;
        LengthBuffer_[1] = 0;
        Length_ = 0;
        Checksum_[0] = 0;
        Checksum_[1] = 0;
        return false;
      }
    }
  }
  return false;
}

/* Writes data to serial port. */
void FlightManagementUnit::WritePort(uint8_t* Buffer,size_t BufferSize) {
  int count;
  if ((count=write(FmuFileDesc_,Buffer,BufferSize))<0) {
    do {
      count=write(FmuFileDesc_,Buffer,BufferSize);
      usleep(10);
    } while (count < 0);
  }
}

/* Computes a two byte checksum. */
void FlightManagementUnit::CalcChecksum(size_t ArraySize, uint8_t *ByteArray, uint8_t *Checksum) {
  Checksum[0] = 0;
  Checksum[1] = 0;
  for (size_t i = 0; i < ArraySize; i++) {
    Checksum[0] += ByteArray[i];
    Checksum[1] += Checksum[0];
  }
}

void FlightManagementUnit::PublishSensors() {
  for (size_t i=0; i < SensorData_.Time_us.size(); i++) {
      // fixme: uint64_t
      SensorNodes_.Time_us[i]->setLong(SensorData_.Time_us[i]);
  }
  for (size_t i=0; i < SensorData_.InternalMpu9250.size(); i++) {
      SensorNodes_.InternalMpu9250[i].ax->setFloat(SensorData_.InternalMpu9250[i].Accel_mss(0,0));
      SensorNodes_.InternalMpu9250[i].ay->setFloat(SensorData_.InternalMpu9250[i].Accel_mss(1,0));
      SensorNodes_.InternalMpu9250[i].az->setFloat(SensorData_.InternalMpu9250[i].Accel_mss(2,0));
      SensorNodes_.InternalMpu9250[i].p->setFloat(SensorData_.InternalMpu9250[i].Gyro_rads(0,0));
      SensorNodes_.InternalMpu9250[i].q->setFloat(SensorData_.InternalMpu9250[i].Gyro_rads(1,0));
      SensorNodes_.InternalMpu9250[i].r->setFloat(SensorData_.InternalMpu9250[i].Gyro_rads(2,0));
      SensorNodes_.InternalMpu9250[i].hx->setFloat(SensorData_.InternalMpu9250[i].Mag_uT(0,0));
      SensorNodes_.InternalMpu9250[i].hy->setFloat(SensorData_.InternalMpu9250[i].Mag_uT(1,0));
      SensorNodes_.InternalMpu9250[i].hz->setFloat(SensorData_.InternalMpu9250[i].Mag_uT(2,0));
      SensorNodes_.InternalMpu9250[i].temp->setFloat(SensorData_.InternalMpu9250[i].Temperature_C);
  }
  for (size_t i=0; i < SensorData_.InternalBme280.size(); i++) {
      SensorNodes_.InternalBme280[i].press->setFloat(SensorData_.InternalBme280[i].Pressure_Pa);
      SensorNodes_.InternalBme280[i].temp->setFloat(SensorData_.InternalBme280[i].Temperature_C);
      SensorNodes_.InternalBme280[i].hum->setFloat(SensorData_.InternalBme280[i].Humidity_RH);
  }
  for (size_t i=0; i < SensorData_.InputVoltage_V.size(); i++) {
      SensorNodes_.input_volts[i]->setFloat(SensorData_.InputVoltage_V[i]);
  }
  for (size_t i=0; i < SensorData_.RegulatedVoltage_V.size(); i++) {
      SensorNodes_.reg_volts[i]->setFloat(SensorData_.RegulatedVoltage_V[i]);
  }
  for (size_t i=0; i < SensorData_.PwmVoltage_V.size(); i++) {
      SensorNodes_.pwm_volts[i]->setFloat(SensorData_.PwmVoltage_V[i]);
  }
  for (size_t i=0; i < SensorData_.SbusVoltage_V.size(); i++) {
      SensorNodes_.sbus_volts[i]->setFloat(SensorData_.SbusVoltage_V[i]);
  }
  for (size_t i=0; i < SensorData_.Mpu9250.size(); i++) {
      SensorNodes_.Mpu9250[i].status->setInt(SensorData_.Mpu9250[i].status);
      SensorNodes_.Mpu9250[i].ax->setFloat(SensorData_.Mpu9250[i].Accel_mss(0,0));
      SensorNodes_.Mpu9250[i].ay->setFloat(SensorData_.Mpu9250[i].Accel_mss(1,0));
      SensorNodes_.Mpu9250[i].az->setFloat(SensorData_.Mpu9250[i].Accel_mss(2,0));
      SensorNodes_.Mpu9250[i].p->setFloat(SensorData_.Mpu9250[i].Gyro_rads(0,0));
      SensorNodes_.Mpu9250[i].q->setFloat(SensorData_.Mpu9250[i].Gyro_rads(1,0));
      SensorNodes_.Mpu9250[i].r->setFloat(SensorData_.Mpu9250[i].Gyro_rads(2,0));
      SensorNodes_.Mpu9250[i].hx->setFloat(SensorData_.Mpu9250[i].Mag_uT(0,0));
      SensorNodes_.Mpu9250[i].hy->setFloat(SensorData_.Mpu9250[i].Mag_uT(1,0));
      SensorNodes_.Mpu9250[i].hz->setFloat(SensorData_.Mpu9250[i].Mag_uT(2,0));
      SensorNodes_.Mpu9250[i].temp->setFloat(SensorData_.Mpu9250[i].Temperature_C);
  }
  for (size_t i=0; i < SensorData_.Bme280.size(); i++) {
      SensorNodes_.Bme280[i].status->setInt(SensorData_.Bme280[i].status);
      SensorNodes_.Bme280[i].press->setFloat(SensorData_.Bme280[i].Pressure_Pa);
      SensorNodes_.Bme280[i].temp->setFloat(SensorData_.Bme280[i].Temperature_C);
      SensorNodes_.Bme280[i].hum->setFloat(SensorData_.Bme280[i].Humidity_RH);
  }
  for (size_t i=0; i < SensorData_.uBlox.size(); i++) {
      SensorNodes_.uBlox[i].fix->setInt(SensorData_.uBlox[i].Fix);
      SensorNodes_.uBlox[i].sats->setInt(SensorData_.uBlox[i].NumberSatellites);
      SensorNodes_.uBlox[i].tow->setInt(SensorData_.uBlox[i].TOW);
      SensorNodes_.uBlox[i].year->setInt(SensorData_.uBlox[i].Year);
      SensorNodes_.uBlox[i].month->setInt(SensorData_.uBlox[i].Month);
      SensorNodes_.uBlox[i].day->setInt(SensorData_.uBlox[i].Day);
      SensorNodes_.uBlox[i].hour->setInt(SensorData_.uBlox[i].Hour);
      SensorNodes_.uBlox[i].min->setInt(SensorData_.uBlox[i].Min);
      SensorNodes_.uBlox[i].sec->setInt(SensorData_.uBlox[i].Sec);
      SensorNodes_.uBlox[i].lat->setDouble(SensorData_.uBlox[i].LLA(0,0));
      SensorNodes_.uBlox[i].lon->setDouble(SensorData_.uBlox[i].LLA(1,0));
      SensorNodes_.uBlox[i].alt->setFloat(SensorData_.uBlox[i].LLA(2,0));
      SensorNodes_.uBlox[i].vn->setFloat(SensorData_.uBlox[i].NEDVelocity_ms(0,0));
      SensorNodes_.uBlox[i].ve->setFloat(SensorData_.uBlox[i].NEDVelocity_ms(1,0));
      SensorNodes_.uBlox[i].vd->setFloat(SensorData_.uBlox[i].NEDVelocity_ms(2,0));
      SensorNodes_.uBlox[i].horiz_acc->setFloat(SensorData_.uBlox[i].Accuracy(0,0));
      SensorNodes_.uBlox[i].vert_acc->setFloat(SensorData_.uBlox[i].Accuracy(1,0));
      SensorNodes_.uBlox[i].vel_acc->setFloat(SensorData_.uBlox[i].Accuracy(2,0));
      SensorNodes_.uBlox[i].pdop->setFloat(SensorData_.uBlox[i].pDOP);
  }
  for (size_t i=0; i < SensorData_.Swift.size(); i++) {
      SensorNodes_.Swift[i].Static.status->setInt(SensorData_.Swift[i].Static.status);
      SensorNodes_.Swift[i].Static.press->setFloat(SensorData_.Swift[i].Static.Pressure_Pa);
      SensorNodes_.Swift[i].Static.temp->setFloat(SensorData_.Swift[i].Static.Temperature_C);
      SensorNodes_.Swift[i].Differential.status->setInt(SensorData_.Swift[i].Differential.status);
      SensorNodes_.Swift[i].Differential.press->setFloat(SensorData_.Swift[i].Differential.Pressure_Pa);
      SensorNodes_.Swift[i].Differential.temp->setFloat(SensorData_.Swift[i].Differential.Temperature_C);
  }
  for (size_t i=0; i < SensorData_.Ams5915.size(); i++) {
      SensorNodes_.Ams5915[i].status->setInt(SensorData_.Ams5915[i].status);
      SensorNodes_.Ams5915[i].press->setFloat(SensorData_.Ams5915[i].Pressure_Pa);
      SensorNodes_.Ams5915[i].temp->setFloat(SensorData_.Ams5915[i].Temperature_C);
  }
  for (size_t i=0; i < SensorData_.Sbus.size(); i++) {
      SensorNodes_.Sbus[i].failsafe->setInt(SensorData_.Sbus[i].FailSafe);
      SensorNodes_.Sbus[i].lost_frames->setInt(SensorData_.Sbus[i].LostFrames);
    for (size_t j=0; j < 16; j++) {
        SensorNodes_.Sbus[i].ch[j]->setFloat(SensorData_.Sbus[i].Channels[j]);
    }
  }
  for (size_t i=0; i < SensorData_.Analog.size(); i++) {
      SensorNodes_.Analog[i].volt->setFloat(SensorData_.Analog[i].Voltage_V);
      SensorNodes_.Analog[i].val->setFloat(SensorData_.Analog[i].CalibratedValue);
  }
}
