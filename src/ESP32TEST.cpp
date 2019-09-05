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
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <OneWire.h>
#include <DallasTemperature.h>
// Data wire is plugged into port 25 on the Arduino
#define ONE_WIRE_BUS 25
#define FAN_RUN 26    // 风扇运行指示 高电平运行
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
unsigned long lastTempRequest = 0;
int  delayInMillis = 0;
static float Oil_Temp =0;

// Please UNCOMMENT one of the contructor lines below
// U8g2 Contructor List (Frame Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ U8X8_PIN_NONE, /* dc=*/ 16, /* reset=*/ 17); // node32s and OLED
//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);

static boolean Display_Flash = false;   // 显示闪烁标志

#define u8g_logo_bits_width 128
#define u8g_logo_bits_height 64
static const unsigned char u8g_logo_bits[] U8X8_PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xC0, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0x05, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x01, 0x00, 0x00, 0xFC, 0xFF, 0x3F,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x07, 0x00,
  0x80, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x38, 0x0C, 0x00, 0xE0, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xF0,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x38, 0x00, 0xF8, 0xFF, 0xFF, 0xF7,
  0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x70, 0x00,
  0xFE, 0xFF, 0xFF, 0xE1, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00,
  0xE0, 0x00, 0xC0, 0x81, 0xFF, 0xFF, 0x7F, 0x80, 0x00, 0x00, 0x00, 0xFF,
  0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x80, 0xE3, 0xFF, 0xFF, 0x1F, 0x00,
  0x00, 0x00, 0xF0, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x00, 0xFE,
  0xFF, 0xFF, 0x8F, 0xFF, 0x00, 0x00, 0xF8, 0xFF, 0x00, 0x00, 0x00, 0x00,
  0x07, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 0xE3, 0xFF, 0x01, 0x00, 0xFF, 0xFF,
  0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0xF0, 0xFF,
  0x03, 0x80, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0xC2, 0xFF,
  0xFF, 0xFF, 0xF8, 0xFF, 0x07, 0xE0, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x78,
  0x00, 0x00, 0xC2, 0xFF, 0xFF, 0xFF, 0xFC, 0xFF, 0x0F, 0xE0, 0xFF, 0xFF,
  0x00, 0x00, 0x00, 0x6E, 0x00, 0x00, 0xC2, 0xFF, 0xFF, 0x7F, 0xFE, 0xFF,
  0x0F, 0xE0, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x63, 0x00, 0x00, 0xFE, 0xFF,
  0xFF, 0x7F, 0xFE, 0xFF, 0x1F, 0xFE, 0xFF, 0xFF, 0x00, 0x00, 0xC0, 0xC1,
  0x00, 0x00, 0xFE, 0xFF, 0xFF, 0x7F, 0xFE, 0xE1, 0x1F, 0xFE, 0xFF, 0xFF,
  0x00, 0x00, 0x70, 0x80, 0x01, 0x00, 0x02, 0xFF, 0xFF, 0x7F, 0xFF, 0xC0,
  0xDF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x3C, 0x80, 0x01, 0x00, 0x02, 0xFE,
  0xFF, 0x7F, 0xFF, 0xC0, 0xDF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x06, 0x00,
  0x03, 0x00, 0x02, 0xFC, 0xFF, 0x7F, 0xFF, 0xC0, 0xFF, 0xFF, 0xFF, 0xFF,
  0x00, 0x80, 0x03, 0x00, 0x03, 0x00, 0x00, 0xC0, 0xFF, 0x7F, 0xFF, 0xC0,
  0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xE0, 0x00, 0x00, 0x06, 0x00, 0x00, 0x03,
  0xFF, 0x7F, 0xFE, 0xE1, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x70, 0x00, 0x00,
  0x6C, 0x00, 0x00, 0x07, 0xFF, 0x3F, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0x00, 0x18, 0x00, 0x00, 0xEC, 0x01, 0x00, 0x0C, 0xFF, 0x3F, 0xFE, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x0C, 0x00, 0x00, 0xF8, 0x07, 0x60, 0xF8,
  0xFF, 0x1B, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x08, 0x00, 0x00,
  0xD8, 0x0F, 0xF8, 0xE1, 0xFF, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0x00, 0x08, 0x00, 0x00, 0xF0, 0x3F, 0xFE, 0xE3, 0x7F, 0x00, 0xF8, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x18, 0x00, 0x00, 0xA0, 0xFF, 0xFF, 0xF3,
  0x1F, 0x00, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x18, 0x00, 0x00,
  0xE0, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0x00, 0x10, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0xF8,
  0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x30, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0xFF,
  0x00, 0x00, 0x00, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x30, 0x00, 0x00,
  0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0x00, 0x20, 0x1C, 0xC0, 0xFF, 0xFF, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x67, 0x3C, 0xF0, 0xFF, 0xFF, 0xFF, 0x07,
  0x00, 0x00, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x6F, 0x78, 0xF8,
  0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xC0, 0x6F, 0xF8, 0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0xE0, 0xC3, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x7F, 0xF0, 0xFF, 0xFF, 0xFF, 0x1F, 0x00,
  0x00, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xFF, 0xE0, 0xFF,
  0x07, 0xFF, 0x0F, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xC0, 0xFF, 0xF8, 0xFF, 0x01, 0xFC, 0x03, 0x00, 0x00, 0xFC, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xFF, 0xFE, 0x7F, 0xF0, 0xF8, 0x00, 0x00,
  0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0xFF, 0xFF, 0x1F,
  0xFC, 0xC3, 0x00, 0x1C, 0x00, 0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0x00, 0xFF, 0xFF, 0x0F, 0xFF, 0x0F, 0x00, 0x3E, 0x00, 0x82, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xC7, 0xFF, 0x1F, 0x00, 0x7F,
  0xC0, 0x83, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFE, 0xFF, 0xC3,
  0xFF, 0x3F, 0x00, 0x7F, 0xE0, 0x83, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0x00, 0xFE, 0xFF, 0xE3, 0xFF, 0x3F, 0x00, 0xFF, 0xF3, 0x83, 0x3F, 0xF8,
  0x3F, 0xF8, 0x83, 0xF0, 0x00, 0xFC, 0xFF, 0xF3, 0xFF, 0x7F, 0x00, 0xFF,
  0xFF, 0x83, 0x0F, 0xE0, 0x0F, 0xE0, 0x03, 0xE0, 0x00, 0xF8, 0xFF, 0xF3,
  0xFF, 0xFF, 0x80, 0xFF, 0xFF, 0x83, 0x07, 0xC0, 0x07, 0xC0, 0x03, 0xC0,
  0x00, 0xF8, 0xFF, 0xF3, 0x0F, 0xFF, 0x80, 0xFF, 0xFF, 0x83, 0x07, 0xC1,
  0x07, 0xC1, 0x03, 0x81, 0x00, 0xF0, 0xFF, 0xFB, 0x07, 0xFE, 0x81, 0xFF,
  0xFF, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x00, 0xF0, 0xFF, 0xFB,
  0x07, 0xFE, 0xC1, 0xFF, 0xFF, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83,
  0x00, 0xE0, 0xF7, 0xF9, 0x03, 0xFE, 0xE1, 0xFF, 0xC7, 0x83, 0x03, 0x80,
  0x03, 0x80, 0x83, 0x83, 0x00, 0xC0, 0xB1, 0xF9, 0x07, 0xFE, 0xF1, 0xFF,
  0xC1, 0x83, 0x03, 0x80, 0x03, 0x80, 0x83, 0x83, 0x00, 0x00, 0x00, 0xF0,
  0x0F, 0xFF, 0xF1, 0xFF, 0x81, 0x81, 0x83, 0xFF, 0x83, 0xFF, 0x83, 0x83,
  0x00, 0x00, 0x1E, 0xF0, 0xDF, 0xFF, 0xF8, 0xFF, 0x03, 0xC0, 0x07, 0x83,
  0x07, 0x83, 0x03, 0x81, 0x00, 0x80, 0x3F, 0xF0, 0xFF, 0xFF, 0xFC, 0xFF,
  0x03, 0xC0, 0x07, 0xC0, 0x07, 0xC0, 0x03, 0xC0, 0x00, 0xE0, 0x7F, 0xE0,
  0xFF, 0x7F, 0xFE, 0xFF, 0x07, 0xE0, 0x0F, 0xE0, 0x0F, 0xE0, 0x03, 0xC0,
  0x00, 0xF8, 0xFF, 0xE0, 0xFF, 0x3F, 0xFE, 0xFF, 0x1F, 0xF8, 0x1F, 0xF0,
  0x1F, 0xF0, 0x83, 0xF0, 0x00, 0xF8, 0xFF, 0x83, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x83, 0xFF, 0xE0, 0xFF, 0xFF, 0x03,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x83, 0xFF,
  0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0x83, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x83, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x83, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, };

#include "BLEDevice.h"
// The remote service we wish to connect to.
static BLEUUID serviceUUID("0000FFF0-0000-1000-8000-00805F9B34FB");
//static BLEUUID serviceUUID("FFF0");
// The characteristic of the remote service we are interested in.
static BLEUUID    WcharUUID("0000FFF2-0000-1000-8000-00805F9B34FB");
//static BLEUUID    WcharUUID("FFF2");
static BLEUUID    RcharUUID("0000FFF1-0000-1000-8000-00805F9B34FB");
//static BLEUUID    RcharUUID("FFF1");

static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* WpRemoteCharacteristic ;
static BLERemoteCharacteristic* RpRemoteCharacteristic;
static boolean BLE_OnTime_Scan = false;

#define READTEMP  1
#define READRPM 3
#define READNONE 0
static uint8_t ReadDATA = READNONE;    // "0":不做什么；“1”读取温度；“3”读取转速
static uint16_t EngineRPM = 0;
static uint8_t EngineTEMP = 0;
static String IncomingBuffer = "";
static  uint8_t ReadRPMCONT = 0;
static  char run[] = {'|','/','-','\\','-'};
static  float BatV = 0;
static  uint8_t Data_Link = 0;
static boolean GearOil_Temp_High = false;   // 变速箱油温过高
static boolean EngOil_Temp_High = false;   // 发动机油温过高
static boolean BatV_Low = false;   // 电池电压过低

const int wdtTimeout = 2000000;  //time in ms to trigger the watchdog To Reset!
hw_timer_t *timer = NULL;

void IRAM_ATTR onTimer(){
  if (connected) {    // 是否蓝牙通信监视看门狗溢出
    ESP.restart();
  } else{
    BLE_OnTime_Scan = true;
  }
}

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
    return true;
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


// 递推平均滤波法（又称滑动平均滤波法）
#define FILTER_N 4
int filter_buf[FILTER_N + 1];
float read_ADBATV(){  // 读取A03引脚上的电瓶电压，分压电阻为 V12-->10K-->A03-->2K-->GND
  int i;
  int filter_sum = 0;
  filter_buf[FILTER_N] = analogRead(A3);
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = filter_buf[i + 1]; // 所有数据左移，低位仍掉
    filter_sum += filter_buf[i];
  }
  return ((filter_sum / FILTER_N) * 0.004872 + 0.47 );
}

void u8g2_prepare(void) {
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontDirection(0);
}

void u8g2_show() {
  char stemp[20];

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso20_tr);
  if (BatV_Low ) {    // 如果电压过低则闪烁
    if (Display_Flash) {
      snprintf_P(stemp, sizeof(stemp), PSTR("%s"), " ");
    } else {
      snprintf_P(stemp, sizeof(stemp), PSTR("%4.1f"), BatV);
    }
  } else {
    snprintf_P(stemp, sizeof(stemp), PSTR("%4.1f"), BatV);
  }
  u8g2.drawStr(0,21,stemp);
  u8g2.drawStr(55,21,String(run[Data_Link]).c_str());
  if (GearOil_Temp_High ) {     // 如果变速箱温度过高则闪烁
    if (Display_Flash) {
      snprintf_P(stemp, sizeof(stemp), PSTR("%s"), " ");
    } else {
      snprintf_P(stemp, sizeof(stemp), PSTR("%5.1f"), Oil_Temp);
    }
  } else {
    snprintf_P(stemp, sizeof(stemp), PSTR("%5.1f"), Oil_Temp);
  }
  u8g2.drawStr(65,21,stemp);
  u8g2.setFont(u8g2_font_logisoso28_tn);
  if (EngOil_Temp_High ) {      // 如果发动机温度过高则闪烁
    if (Display_Flash) {
      snprintf_P(stemp, sizeof(stemp), PSTR("%s"), " ");
    } else {
      snprintf_P(stemp, sizeof(stemp), PSTR("%3d"), EngineTEMP);
    }
  } else {
    snprintf_P(stemp, sizeof(stemp), PSTR("%3d"), EngineTEMP);
  }
  u8g2.drawStr(0,63,stemp);
  u8g2.drawCircle(49, 28, 2);
  u8g2.drawHLine(0,23,128);
  u8g2.drawVLine(55,23,40);
  snprintf_P(stemp, sizeof(stemp), PSTR("%4d"), EngineRPM);
  u8g2.drawStr(56,63,stemp);
  u8g2.setFont(u8g2_font_profont12_tf);
  u8g2.drawStr(109,31,"rpm");
  u8g2.sendBuffer();
}

void u8g2_showTempVoltage(){
  char stemp[20];

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_freedoomr25_tn);

  if (BatV_Low ) {      // 如果电压过低则闪烁
    if (Display_Flash) {
      snprintf_P(stemp, sizeof(stemp), PSTR("%s"), " ");
    } else {
      snprintf_P(stemp, sizeof(stemp), PSTR("%4.1f"), BatV);
    }
  } else {
    snprintf_P(stemp, sizeof(stemp), PSTR("%4.1f"), BatV);
  }
  u8g2.drawStr(31,26,stemp);
  u8g2.setFont(u8g2_font_logisoso20_tr);
  u8g2.drawStr(104,26,"V");
  u8g2.setFont(u8g2_font_freedoomr25_tn);
  if (GearOil_Temp_High ) {       // 如果变速箱温度过高则闪烁
    if (Display_Flash) {
      snprintf_P(stemp, sizeof(stemp), PSTR("%s"), " ");
    } else {
      snprintf_P(stemp, sizeof(stemp), PSTR("%5.1f"), Oil_Temp);
    }
  } else {
    snprintf_P(stemp, sizeof(stemp), PSTR("%5.1f"), Oil_Temp);
  }
  u8g2.drawStr(20,63,stemp);
  u8g2.drawCircle(109, 36, 3);
  u8g2.drawHLine(0,28,128);
  if ((digitalRead(FAN_RUN) > 0) && Display_Flash) {
    u8g2.drawDisc(12, 28, 12,U8G2_DRAW_ALL);
  }
  u8g2.sendBuffer();
}

void setup(void) {

  Serial.begin(115200);

  pinMode(FAN_RUN, INPUT);

  BatV = read_ADBATV();   // 预先采集蓄电池电压

  u8g2.begin();
  u8g2_prepare();
  u8g2.clearBuffer();
  u8g2.setDrawColor(0);     // 反显LOGO
  u8g2.drawXBM( 0, 0, 128, 64, u8g_logo_bits);
  u8g2.sendBuffer();
  u8g2.setDrawColor(1);


  // Start up the Dallas Temperature IC Control library
  sensors.begin();
  delayInMillis = 801;
  sensors.requestTemperatures();
  lastTempRequest = millis();

  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5);
  if (!doConnect) {   // 没有找到蓝牙OBD则关闭BLE电源
    btStop();
  }
}

void loop(void) {
  String newValue = "";

    Display_Flash = !Display_Flash;   // 闪烁显示标志

  if (millis() - lastTempRequest >= delayInMillis) // waited long enough??
  {
    Oil_Temp = sensors.getTempCByIndex(0);   // Read temperature
    if (Oil_Temp > 85) {    // 变速箱温度过高报警
      GearOil_Temp_High = true;
    } else {
      GearOil_Temp_High = false;
    }
    sensors.requestTemperatures();
    lastTempRequest = millis();

    BatV = read_ADBATV();
    if (BatV < 12) {    // 蓄电池电压过低报警
      BatV_Low = true;
    } else {
      BatV_Low = false;
    }
    if (EngineTEMP > 108) {   // 发动机温度高于108度报警
      EngOil_Temp_High = true;
    } else {
      EngOil_Temp_High = false;
    }
  }

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
      btStop();   // 没有连接上蓝牙OBD服务则关闭BLE电源
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
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
    u8g2_show();
    delay(300);
  }else{
    u8g2_showTempVoltage();
    delay(300);
  }
}
