#include <ArduinoSTL.h>
#include <deque.h>
#include <vector.h>
#include <PacketSerial.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>
#include <bluefruit.h>

//#define TEST
//#define SLIP
#define THRESHOLD 2.4      // 2.4 g
#define WINDOWLEN 10       // 10 samples
#define SAMPLINGPERIOD 10  // 10ms
#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732
#define SENSITIVITY_GYROSCOPE_245    0.00875
#define SENSITIVITY_GYROSCOPE_500    0.0175
#define SENSITIVITY_GYROSCOPE_2000   0.07
#define SENSITIVITY_MAGNETOMETER_4   0.00014
#define SENSITIVITY_MAGNETOMETER_8   0.00029
#define SENSITIVITY_MAGNETOMETER_12  0.00043
#define SENSITIVITY_MAGNETOMETER_16  0.00058
const float ascale = SENSITIVITY_ACCELEROMETER_16;
const float gscale = SENSITIVITY_GYROSCOPE_245;
const float mscale = SENSITIVITY_MAGNETOMETER_12;



uint16_t initLSM9DS1();
void imuRead();
template <typename E>
class SlidingWindowMinMax final {

    /*-- Fields --*/

  private: etl::deque < E, WINDOWLEN + 5 > minDeque;
  private: etl::deque < E, WINDOWLEN + 5 > maxDeque;


    /*-- Methods --*/

  public: E getMinimum() {
      return minDeque.front();
    }


  public: E getMaximum() {
      return maxDeque.front();
    }

  public: E clear() {
      maxDeque.clear();
      minDeque.clear();
    }



  public: void addTail(const E &val) {
      while (!minDeque.empty() && val < minDeque.back())
        minDeque.pop_back();
      minDeque.push_back(val);

      while (!maxDeque.empty() && val > maxDeque.back())
        maxDeque.pop_back();
      maxDeque.push_back(val);
    }


  public: void removeHead(const E &val) {
      if (val == minDeque.front())
        minDeque.pop_front();

      if (val == maxDeque.front())
        maxDeque.pop_front();
    }
};

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
} imu_buffer;
TaskHandle_t  SendDataHandle;
TickType_t xLastWakeTime;
const TickType_t xFrequency = SAMPLINGPERIOD;
etl::deque < imu_data, WINDOWLEN > window;
//etl::vector < imu_data, WINDOWLEN > window;
SlidingWindowMinMax<int16_t> swm;
int window_not_full;

unsigned int i;
unsigned long j;
float spread;

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
    swm.addTail(imu.ax);
    memcpy(&imu_buffer, &imu.gx, sizeof(imu_data));
    window.push_back(imu_buffer);
    if (window_not_full)
    {
      window_not_full--;
    } else
    {
      //etl::vector<imu_data, 1>::const_iterator start = window.begin();
      //swm.removeHead(start->ax);
      //window.erase(window.begin());
      window.pop_front();
      spread = swm.getMaximum() * ascale - swm.getMinimum() * ascale;
      //Serial.printf("max=%f min=%f\n",swm.getMaximum()*ascale,swm.getMaximum()*ascale);
      if (spread > THRESHOLD)
      {
        Serial.printf("spread=%f\n", spread);
        window_not_full = 10;
        swm.clear();
        while (!window.empty())
        {
          imu_data in = window.front();
#ifdef SLIP
          myPacketSerial.send((uint8_t*)&in, (uint8_t)sizeof(imu_data));
#else
          bleuart.printf("{\"a\":[%f,%f,%f],", in.ax * ascale, in.ay * ascale, in.az * ascale);
          bleuart.printf("\"g\":[%f,%f,%f],", in.gx * gscale, in.gy * gscale, in.gz * gscale);
          bleuart.printf("\"m\":[%f,%f,%f],", in.mx * mscale, in.my * mscale, in.mz * mscale);
          bleuart.printf("\"ts\":%lu}\n", in.ts);
#endif
          window.pop_front();
        }
      }
    }
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void setup()
{
  i = 0;
  j = 0;
  window_not_full = 10;
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

void loop()
{
  // Request CPU to enter low-power mode until an event/interrupt occurs
  waitForEvent();
}
