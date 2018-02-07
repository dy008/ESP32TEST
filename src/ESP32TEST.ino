/*

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>                         // I2C support library
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#include "thermistor.h"
#define PANSTAMP_NRG 1
// Analog pin used to read the NTC
#define NTC_PIN               A0
// Thermistor object
THERMISTOR thermistor(NTC_PIN,        // Analog pin
                      10000,          // Nominal resistance at 25 ºC
                      3950,           // thermistor's beta coefficient
                      10000,         // Value of the series resistor
                      93);          // fix ADC value
static float Oil_Temp =0;

#include "BLEDevice.h"
//#include "BLEScan.h"

// The remote service we wish to connect to.
static BLEUUID serviceUUID("0000FFF0-0000-1000-8000-00805F9B34FB");
//static BLEUUID serviceUUID("FFF0");
// The characteristic of the remote service we are interested in.
static BLEUUID    WcharUUID("0000FFF2-0000-1000-8000-00805F9B34FB");
//static BLEUUID    WcharUUID("FFF2");
static BLEUUID    RcharUUID("0000FFF1-0000-1000-8000-00805F9B34FB");
//static BLEUUID    RcharUUID("FFF1");

static BLEAddress *pServerAddress;
static BLEClient*  pClient;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* WpRemoteCharacteristic ;
static BLERemoteCharacteristic* RpRemoteCharacteristic;

#define READTEMP  1
#define READRPM 3
#define READNONE 0
static uint8_t ReadDATA = READNONE;    // "0":不做什么；“1”读取电压；“2”读取温度；“3”读取转速
static uint16_t EngineRPM = 0;
static uint8_t EngineTEMP = 0;
static String IncomingBuffer = "";
static  uint8_t ReadRPMCONT = 0;
static  char run[] = {'|','/','-','\\','-'};
static  float BatV = 0;
static  uint8_t Data_Link = 0;

const int wdtTimeout = 300000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    if (Data_Link++ > 3) {
      Data_Link = 0;
    }
    timerWrite(timer, 0); //reset timer (feed watchdog)
    IncomingBuffer = "";
    String stemp = "";
    uint16_t itemp = 0;
    digitalWrite(2, !digitalRead(2));
    if (length > 4) {
        for (int i = 0; i < length; i++)
          IncomingBuffer = IncomingBuffer + char(pData[i]);
        Serial.println(IncomingBuffer);
        if ((IncomingBuffer.indexOf("NO DATA")) >= 0) { //如果OBD离线则持续读取发动机温度并等待上线
          ReadDATA = READTEMP;
          return;
        }
        if ((IncomingBuffer.indexOf("TO CONNECT")) >= 0) {  //如果没有搜索到正确的OBD协议则持续读取发动机温度并等待上线
          ReadDATA = READTEMP;
          return;
        }
        if ((IncomingBuffer.indexOf("41 0C")) >= 0) { //读取发动机转速
          stemp = (IncomingBuffer.substring((IncomingBuffer.indexOf("41 0C")+6),
                  IncomingBuffer.indexOf("41 0C")+8))+IncomingBuffer.substring((IncomingBuffer.indexOf("41 0C")+9),
                  IncomingBuffer.indexOf("41 0C")+11);
          EngineRPM = strtol(stemp.c_str(),NULL,16) / 4;  // OBD转速数据需要除4才得到正确值
          if (ReadRPMCONT++ > 5) { //连续读取6次之后再读其他数据
            ReadDATA = READTEMP;
            ReadRPMCONT = 0;
          }else{
            ReadDATA = READRPM;
          }
          return;
        }
        if ((IncomingBuffer.indexOf("41 05")) >= 0) {
          stemp = IncomingBuffer.substring((IncomingBuffer.indexOf("41 05")+6),
                  IncomingBuffer.indexOf("41 05")+8);
          EngineTEMP = strtol(stemp.c_str(),NULL,16) - 40;  // OBD发动机温度数据需要减去40度才是正确值
          ReadDATA = READRPM;
          return;
        }
      }   // End if (length > 4)
}

bool connectToServer(BLEAddress pAddress) {
    Serial.print("Forming a connection to ");
    Serial.println(pAddress.toString().c_str());

    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    // Connect to the remove BLE Server.
    pClient->connect(pAddress);
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    WpRemoteCharacteristic = pRemoteService->getCharacteristic(WcharUUID);
    if (WpRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic WUUID: ");
      Serial.println(WcharUUID.toString().c_str());
      return false;
    }
    Serial.print(" - Found our Wcharacteristic:");
    // Read the value of the characteristic.
    BLEUUID value = WpRemoteCharacteristic->getUUID();
    Serial.println(value.toString().c_str());

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    RpRemoteCharacteristic = pRemoteService->getCharacteristic(RcharUUID);
    if (RpRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic RUUID: ");
      Serial.println(RcharUUID.toString().c_str());
      return false;
    }
    Serial.print(" - Found our Rcharacteristic");
    value = RpRemoteCharacteristic->getUUID();
    Serial.println(value.toString().c_str());

    RpRemoteCharacteristic->registerForNotify(notifyCallback);
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) {

      //
      Serial.print("Found our device!  address: ");
      advertisedDevice.getScan()->stop();

      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

// Please UNCOMMENT one of the contructor lines below
// U8g2 Contructor List (Frame Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
//U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ U8X8_PIN_NONE, /* dc=*/ 19, /* reset=*/ 5); // node32s and OLED
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);   // pin remapping with ESP8266 HW I2C

void u8g2_prepare(void) {
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  //u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void u8g2_show() {
  char stemp[20];

  u8g2.setFont(u8g2_font_profont12_tf);
  snprintf_P(stemp, sizeof(stemp), PSTR("%4.1fV"), BatV);
  u8g2.drawStr(0,11,stemp);
  u8g2.drawStr(60,11,String(run[Data_Link]).c_str());
  snprintf_P(stemp, sizeof(stemp), PSTR("%5.1f"), Oil_Temp);
  u8g2.drawStr(95,11,stemp);
  u8g2.setFont(u8g2_font_profont22_tr);
  snprintf_P(stemp, sizeof(stemp), PSTR("%3d"), EngineTEMP);
  u8g2.drawStr(0,31,stemp);
  u8g2.drawCircle(39, 19, 2);
  u8g2.drawHLine(0,13,128);
  u8g2.drawVLine(45,13,19);
  snprintf_P(stemp, sizeof(stemp), PSTR("%4d"), EngineRPM);
  u8g2.drawStr(60,31,stemp);
  u8g2.setFont(u8g2_font_profont12_tf);
  u8g2.drawStr(109,31,"rpm");
}

void u8g2_showTempVoltage(){
  char stemp[20];

  u8g2.drawVLine(48,0,31);
  u8g2.setFont(u8g2_font_profont22_tr);
  snprintf_P(stemp, sizeof(stemp), PSTR("%4.1f"), BatV);
  u8g2.drawStr(0,31,stemp);
  u8g2.drawStr(36,14,"V");
  u8g2.setFont(u8g2_font_profont29_tr);
  snprintf_P(stemp, sizeof(stemp), PSTR("%5.1f"), Oil_Temp);
  u8g2.drawStr(50,31,stemp);
  u8g2.drawCircle(125, 5, 2);
}

float read_ADBATV(){  // 读取A03引脚上的电瓶电压，分压电阻为 V12-->10K-->A03-->2K-->GND
  float average = 0;
  uint16_t sample;

  for (size_t i = 0; i < 5; i++) {
    sample = analogRead(A3);
    average += sample;
    delay(10);
  }
  average /= 5;
  average = average * 0.004872 + 0.47 ;
  return (average);
}

void setup(void) {
  char stemp[20];

  pinMode(23,OUTPUT);
  digitalWrite(23,HIGH);  // power on OLED
  Serial.begin(115200);

  for (size_t i = 0; i < 10; i++) {
    Oil_Temp = thermistor.read();   // Read temperature
    BatV = read_ADBATV(); // 读取电瓶电压
  }


  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &esp_restart, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);                          //enable interrupt

  u8g2.begin();
  u8g2_prepare();
  u8g2.clearBuffer();
  u8g2_showTempVoltage();
  u8g2.sendBuffer();

  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5);
}

void loop(void) {
  String newValue = "";

  Oil_Temp = thermistor.read();   // Read temperature
  BatV = read_ADBATV();

    // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      connected = true;

      newValue = "ATZ\r\n";
      // Set the characteristic's value to be the array of bytes that is actually a string.
      WpRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
      delay(2000);
      newValue = "ATSP0\r\n";
      WpRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
      delay(500);
      newValue = "01051\r\n";
      WpRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
      delay(200);
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    switch (ReadDATA) {
      case READTEMP: ReadDATA = READNONE;
        WpRemoteCharacteristic->writeValue("01051\r\n", newValue.length());
        break;
      case READRPM: ReadDATA = READNONE;
        WpRemoteCharacteristic->writeValue("010C1\r\n", newValue.length());
        break;
      default: ReadDATA = READNONE;
        break;
    }
    u8g2.clearBuffer();
    u8g2_show();
    u8g2.sendBuffer();
    delay(300);
  }else{
    u8g2.clearBuffer();
    u8g2_showTempVoltage();
    u8g2.sendBuffer();
    delay(500);
    btStop();   // power off BLE Mode
  }
}
