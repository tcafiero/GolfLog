//#define TEST
#include <PacketSerial.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>
#include <bluefruit.h>

// BLE Service
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;
bool connected;
extern LSM9DS1 imu;
SLIPPacketSerial myPacketSerial;
struct imu_data {
  int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
  int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
  int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
  int32_t ts;         // time stamp
};
TaskHandle_t  SendDataHandle;
TickType_t xLastWakeTime;
const TickType_t xFrequency = 10;

unsigned int i;
unsigned long j;
void setup()
{
  i = 0;
  j = 0;
  pinMode(LED_RED, OUTPUT);
  Serial.begin(115200);
  delay(1000);
  Serial.println("IoThingsWare GolfLogger");
  Serial.println("-----------------------\n");
  connected = false;
  initLSM9DS1();
  Bluefruit.autoConnLed(true);
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  // Set power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(-30);
  Bluefruit.setName("GolfLogger");
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);
  // Configure and Start Device Information Service
  bledis.setManufacturer("IoThingsWare");
  bledis.setModel("TopView GolfLogger");
  bledis.begin();
  // Configure and Start BLE Uart Service
  bleuart.begin();
  myPacketSerial.setStream(&bleuart);
  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);
  // Set up and start advertising
  startAdv();
  xTaskCreate( SendData_callback, "SendData", configMINIMAL_STACK_SIZE + 200, NULL, 2, &SendDataHandle );
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);
  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void loop()
{
  // Request CPU to enter low-power mode until an event/interrupt occurs
  waitForEvent();
}

void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));
  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  Serial.println();
  Serial.println("Disconnected");
}

void getData()
{
  #ifndef TEST
    imuRead();
    imu.ts = millis();
#else
    if (i > 30000) i = 0;
    imu.ax = i++;
    imu.ay = i++;
    imu.az = i++;
    imu.gx = i++;
    imu.gy = i++;
    imu.gz = i++;
    imu.mx = i++;
    imu.my = i++;
    imu.mz = i++;
    imu.ts = j++;
#endif
}

void SendData_callback(void *pvParameter)
{
  for ( ;; )
  {
    xLastWakeTime = xTaskGetTickCount();
    digitalToggle(LED_RED);
    getData();
    myPacketSerial.send((uint8_t *)&imu.gx, (uint8_t)sizeof(imu_data));
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}
