#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "DFRobot_Aliyun.h"
#include <OneWire.h>
#define BEDROOD_LIGHT LED_BUILTIN
/*DS18S20引脚*/
const int DS18S20_Pin = 3;//扩展版0，对应板子D3
OneWire ds(DS18S20_Pin);  // on digital pin 2
//温度获取函数
float getTemp(){
      //returns the temperature from one DS18S20 in DEG Celsius

      byte data[12];
      byte addr[8];

      if ( !ds.search(addr)) {
          //no more sensors on chain, reset search
          ds.reset_search();
          return -1000;
      }

      if ( OneWire::crc8( addr, 7) != addr[7]) {
          Serial.println("CRC is not valid!");
          return -1000;
      }

      if ( addr[0] != 0x10 && addr[0] != 0x28) {
          Serial.print("Device is not recognized");
          return -1000;
      }

      ds.reset();
      ds.select(addr);
      ds.write(0x44,1); // start conversion, with parasite power on at the end

      byte present = ds.reset();
      ds.select(addr);
      ds.write(0xBE); // Read Scratchpad


      for (int i = 0; i < 9; i++) { // we need 9 bytes
        data[i] = ds.read();
      }

      ds.reset_search();

      byte MSB = data[1];
      byte LSB = data[0];

      float tempRead = ((MSB << 8) | LSB); //using two's compliment
      float TemperatureSum = tempRead / 16;

      return TemperatureSum;

    }




/*配置WIFI名和密码*/
const char* WIFI_SSID = "TP-LINK_900F";
const char* WIFI_PASSWORD = "cxd12345";

/*配置设备证书信息*/
String ProductKey = "hrg067MoRPX";//DS18S20设备ProductKey
String ClientId = "12345"; /*自定义ID*/
String DS18S20_DeviceName = "temperature_1";//温度传感器
String DS18S20_DeviceSecret = "d32bb16685134c9f9f486007654f7e07";

/*配置域名和端口号*/
String ALIYUN_SERVER = "iot-as-mqtt.cn-shanghai.aliyuncs.com";
uint16_t PORT = 1883;

/*需要操作的产品标识符*/
String TempIdentifier = "dushu";
String NTUIdentifier="water_hunzhuo_du";

/*需要上报TOPIC*/

const char* pubTopic = "/sys/hrg067MoRPX/temperature_1/thing/event/property/post";  //******post

DFRobot_Aliyun myAliyun;
WiFiClient espClient;
PubSubClient client(espClient);


void connectWiFi() {
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP Adderss: ");
  Serial.println(WiFi.localIP());
}

/*灯的callback
void callback(char* topic, byte* payload, unsigned int len) {
  Serial.print("Recevice [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < len; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  StaticJsonBuffer<300> jsonBuffer;
  //arduino json5
  JsonObject& root = jsonBuffer.parseObject((const char*)payload);
 // DeserializationError root = deserializeJson(jsonBuffer, payload);
//arduino json5
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return;
  }  
  const uint16_t LightStatus = root["params"][Identifier];
  //if (root)
 // {
  // / //  return;
  //} 
//const uint16_t LightStatus = root["params"][Identifier];
  if (LightStatus == 1) {
    openLight();
  } else {
    closeLight();
  }
  String tempMseg = "{\"id\":" + ClientId + ",\"params\":{\"" + Identifier + "\":" + (String)LightStatus + "},\"method\":\"thing.event.property.post\"}";
  char sendMseg[tempMseg.length()];
  strcpy(sendMseg, tempMseg.c_str());
  client.publish(pubTopic, sendMseg);
}
*/
//温度的callback
void callback(char * topic, byte * payload, unsigned int len){
  Serial.print("Recevice [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < len; i++){
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
void ConnectAliyun() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    /*根据自动计算的用户名和密码连接到Alinyun的设备，不需要更改*/
    if (client.connect(myAliyun.client_id, myAliyun.username, myAliyun.password)) {
      Serial.println("connected");
     // client.subscribe(subTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
void setup() {
  Serial.begin(115200);
  //pinMode(BEDROOD_LIGHT, OUTPUT);

  /*连接WIFI*/
  connectWiFi();

  /*初始化Alinyun的配置，可自动计算用户名和密码*/
  myAliyun.init(ALIYUN_SERVER, ProductKey, ClientId, DS18S20_DeviceName, DS18S20_DeviceSecret);

  client.setServer(myAliyun.mqtt_server, PORT);

  /*设置回调函数，当收到订阅信息时会执行回调函数*/
  client.setCallback(callback);

  /*连接到Aliyun*/
  ConnectAliyun();


}

void loop() {
  if (!client.connected()) {
    ConnectAliyun();
  }
  int sensorValue = analogRead(A0);// read the input on analog pin 0:
  float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float temp_val=getTemp();
  Serial.println(voltage); // print out the value you read:
  client.publish(pubTopic,("{\"id\":"+ClientId+",\"params\":{\""+TempIdentifier+"\":"+voltage+",\""+NTUIdentifier+"\":"+temp_val+"},\"method\":\"thing.event.property.post\"}").c_str());
  delay(500);

  client.loop();
}