//#define TEST
#include <PacketSerial.h>
#include <bluefruit.h>
//#define DEBUG
BLEClientDis  clientDis;
BLEClientUart clientUart;
unsigned int i, trigger;
unsigned long int j;
struct imu_data {
  int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
  int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
  int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
  int32_t ts;         // time stamp
};

SLIPPacketSerial myPacketSerial;

void setup()
{
  trigger=0;
  Serial.begin(115200);
#ifdef DEBUG
  Serial.println("IoThingsWare nRF52 Central BLEUART Gateway");
  Serial.println("------------------------------------------\n");
#endif

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);

  Bluefruit.setName("IoThingsWare BLEUART Gateway");

  // Configure DIS client
  clientDis.begin();

  // Init BLE Central Uart Serivce
  clientUart.begin();
  myPacketSerial.setStream(&clientUart);
  myPacketSerial.setPacketHandler(&onPacketReceived);

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
     - Enable auto scan if disconnected
     - Interval = 100 ms, window = 80 ms
     - Don't use active scan
     - Start(timeout) with timeout = 0 will scan forever (until connected)
  */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);                   // // 0 = Don't stop scanning after n seconds
}

/**
   Callback invoked when scanner pick up an advertising data
   @param report Structural advertising data
*/
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  // Check if advertising contain BleUart service
  if ( Bluefruit.Scanner.checkReportForService(report, clientUart) )
  {
#ifdef DEBUG
    Serial.print("BLE UART service detected. Connecting ... ");
#endif

    // Connect to device with bleuart service in advertising
    Bluefruit.Central.connect(report);
  }
}

/**
   Callback invoked when an connection is established
   @param conn_handle
*/
void connect_callback(uint16_t conn_handle)
{
#ifdef DEBUG
  Serial.println("Connected");
  Serial.print("Dicovering DIS ... ");
#endif
  if ( clientDis.discover(conn_handle) )
  {
#ifdef DEBUG
    Serial.println("Found it");
#endif
    char buffer[32 + 1];

    // read and print out Manufacturer
    memset(buffer, 0, sizeof(buffer));
    if ( clientDis.getManufacturer(buffer, sizeof(buffer)) )
    {
      #ifdef DEBUG
      Serial.print("Manufacturer: ");
      Serial.println(buffer);
      #endif
    }

    // read and print out Model Number
    memset(buffer, 0, sizeof(buffer));
    if ( clientDis.getModel(buffer, sizeof(buffer)) )
    {
      #ifdef DEBUG
      Serial.print("Model: ");
      Serial.println(buffer);
      #endif
    }

    Serial.println();
  }

  Serial.print("Discovering BLE Uart Service ... ");

  if ( clientUart.discover(conn_handle) )
  {
    Serial.println("Found it");

    Serial.println("Enable TXD's notify");
    clientUart.enableTXD();
    #ifdef DEBUG
    Serial.println("Ready to receive from peripheral");
    #endif
  } else
  {
    #ifdef DEBUG
    Serial.println("Found NONE");
    #endif
    // disconect since we couldn't find bleuart service
    Bluefruit.Central.disconnect(conn_handle);
  }
}

/**
   Callback invoked when a connection is dropped
   @param conn_handle
   @param reason
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
#ifdef DEBUG
  Serial.println("Disconnected");
#endif
}

void onPacketReceived(const uint8_t* buffer, size_t size)
{
  struct imu_data *imu;
  if (size <= 0) return;
  imu = (struct imu_data *)buffer;
  #ifndef TEST
  Serial.printf("{\"a\":[%d,%d,%d],", imu->ax, imu->ay, imu->az);
  Serial.printf("\"g\":[%d,%d,%d],", imu->gx, imu->gy, imu->gz);
  Serial.printf("\"m\":[%d,%d,%d],", imu->mx, imu->my, imu->mz);
  Serial.printf("\"ts\":%lu}\n", imu->ts);
  #else
  if(i > 30000) i=0;
  if(trigger == 0)
  {
    trigger=1;
    i=imu->ax;
    j=imu->ts;
  }
  if(imu->ax != i++ || imu->ay != i++ || imu->az != i++) { Serial.println("ERROR"); trigger=0; return;};
  if(imu->gx != i++ || imu->gy != i++ || imu->gz != i++)  { Serial.println("ERROR"); trigger=0; return;};
  if(imu->mx != i++ || imu->my != i++ || imu->mz != i++)  { Serial.println("ERROR"); trigger=0; return;};
  if(imu->ts != j++)  { Serial.println("ERROR"); trigger=0; return;};
  #endif
}

void loop()
{
  if ( Bluefruit.Central.connected() )
  {
    myPacketSerial.update();
  }
}
