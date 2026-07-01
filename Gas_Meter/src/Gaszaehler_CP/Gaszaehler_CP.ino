/**************************************************************************/
/*
  Gas counter using RTC as counter and storing values in RAM
  (c) Hans Schneider 2026 
  V1.4.2 (final compact DS1307 RAM layout)
  30.06.2026
  forked from /Rainer-G/Gas_Meter
  Board: Lolin(Wemos) D1 R2 & mini oder D1 mini pro
*/
/**************************************************************************/
#define THINGSPEAK                                                        // if you want to send data to the Internet
// #define debugserial                                                    // uncomment if you would like to see debug lines in serial monitor
// #define debugmqtt                                                      // uncomment if you would like to see debug lines over MWTT

#include <Wire.h>
#include <PicoMQTT.h>
#include "Gas_WiFi.h"
#include "Gas_MQTT.h"
#include "Gas_Time.h"
#include <ArduinoOTA.h>
#ifdef THINGSPEAK
  #include "Gas_Thingspeak.h"
#endif

#define RESET_PIN                  D7                                     // LOW will reset the counter and clear old data
#define UPDATE_PIN                 D6                                     // LOW will prevent going to sleep, needed for MQTT Commands & OTA updates

#define COUNTER_MODE               0x20                                   // counter mode 0x20
#define COUNTER_ADDRESS            0x50                                   // 7 I²C bit address for RTC Chip
#define CONTROL_ADDRESS            0x00
#define ADDRESS_COUNTER            0x01                                   // only 3 bytes with 6 BCD digits

#define FLAG_VALID                 0xA5
#define RAM_BASE                   0x08                                   // DS1307 free RAM: 0x08..0x3F

#define ADDRESS_OLD_COUNTER        (RAM_BASE + 0)                         // 4 Byte

#define ADDRESS_START_METER        (RAM_BASE + 4)                         // 4 Byte
#define ADDRESS_START_METER_FLAG   (ADDRESS_START_METER + 4)

#define ADDRESS_START_PERIOD       (RAM_BASE + 9)                         // 4 Byte
#define ADDRESS_START_PERIOD_FLAG  (ADDRESS_START_PERIOD + 4)

#define ADDRESS_DAY_START_COUNT    (RAM_BASE + 14)                        // 4 Byte
#define ADDRESS_DAY_DATE           (RAM_BASE + 18)                        // 3 Byte
#define ADDRESS_DAY_INITIALIZED    (RAM_BASE + 21)                        // 1 Byte
#define ADDRESS_DAY_STATE          (RAM_BASE + 22)                        // 1 Byte

#define ADDRESS_MONTHS_VALID       (RAM_BASE + 23)                        // 1 Byte
#define ADDRESS_MONTH_01           (RAM_BASE + 24)                        // 12 x 2 Byte = 24 Byte

#ifdef debugmqtt
  #define MQTT_DEBUG_NEW_COUNT       "debug/new_count"
  #define MQTT_DEBUG_DAY_START_COUNT "debug/day_start_count"
  #define MQTT_DEBUG_DAILY_COUNT     "debug/daily_count"
  #define MQTT_DEBUG_DAY_INIT        "debug/day_initialized"
  #define MQTT_DEBUG_DATE            "debug/date"
  #define MQTT_DEBUG_TIME            "debug/time"
  #define MQTT_DEBUG_TAG             "debug/tag"
#endif

// Global runtime state
bool      error               = false;            // report error to MQTT
bool      log_msg             = true;             // for Test only, set to false when in operation

uint32_t  old_time;                               // in seconds
uint32_t  new_time;                               // in seconds

uint32_t  old_count           = 0;
uint32_t  new_count           = 0;
uint32_t  delta_count         = 0;

uint32_t  liter_per_count     = 10;               // see meter label
float     mexp3_per_count     = 0.01;             // see meter label
double    brennwert           = 11.688629;        // 11.53800
double    zustandszahl        = 0.93940;          // 0.93940
uint16_t  year_offset         = 2000;

float     energy1             = 0;                // current load in kWh
double    total_energy        = 0;                // total usage in kWh

uint32_t  start_consumption   = 0;                // set after first start or battery change
double    consumption         = 0;                // Gas in liter
uint32_t  start_period        = 0;                // Gas in liter
uint32_t  period              = 0;                // in seconds

String    consumption_string  = "";               // temp string for a monthly consumption

uint32_t  day_start_count     = 0;                // start value for normal daily counting
uint32_t  daily_count         = 0;                // calculated daily counter
double    daily_consumption_l = 0;
double    daily_volume_m3     = 0;
double    daily_energy_kWh    = 0;
uint8_t   day_initialized     = 0;

#define DAY_STATE_NORMAL           0
#define DAY_STATE_MANUAL_RESET     1

uint8_t   day_state = DAY_STATE_NORMAL;

char      valueString1[ 80 ];
char      valueString2[ 80 ];
char      valueString3[ 80 ];
char      valueString4[ 80 ];
char      valueString5[ 80 ];
char      valueString6[ 80 ];
char      valueString7[ 80 ];
char      valueString8[ 80 ];
char      valueString9[ 80 ];
char      valueString10[ 80 ];
char      valueString11[ 80 ];
char      valueString12[ 80 ];
char      valueString13[ 80 ];

uint8_t old_hour;
uint8_t old_min;
uint8_t old_sec;

// Helper unions for byte conversion
union long_t
{
  uint32_t long_data;
  uint8_t  byte_data[ 4 ];
};

union int_t
{
  uint16_t int_data;
  uint8_t  byte_data[ 2 ];
};

// Convert BCD to decimal
uint8_t bcd2byte( uint8_t value )
{
  return (( value >> 4 ) * 10 ) + ( value & 0x0f );
}

// Convert decimal to BCD
uint8_t byte2bcd( uint8_t value )
{
  return (( value / 10) << 4 ) + ( value % 10 );
}

// Set counter mode on the RTC counter chip
bool set_Counter_Mode( )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( CONTROL_ADDRESS );
  Wire.write( COUNTER_MODE );
  return ( Wire.endTransmission( true ) == 0 );
}

// Read control register
uint8_t get_Mode( )
{
  return get_Byte( CONTROL_ADDRESS );
}

// Write one byte to RTC RAM
bool set_Byte( int8_t address, int8_t value )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.write( value );
  return ( Wire.endTransmission( true ) == 0 );
}

// Read one byte from RTC RAM
int8_t get_Byte( int8_t address )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.endTransmission( );
  Wire.requestFrom( COUNTER_ADDRESS, 1 );
  return Wire.read( );
}

// Write 2 bytes to RTC RAM
bool set_Integer( int8_t address, int16_t value )
{
  int_t temp_data;
  temp_data.int_data = value;
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.write( temp_data.byte_data[ 0 ] );
  Wire.write( temp_data.byte_data[ 1 ] );
  return ( Wire.endTransmission( true ) == 0 );
}

// Read 2 bytes from RTC RAM
bool get_Integer( int8_t address, int16_t &value )
{
  int_t temp_data;
  value = 0;
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.endTransmission( );
  byte len = Wire.requestFrom( COUNTER_ADDRESS, 2 );
  if ( len == 0 )
  {
    #ifdef debugserial
      Serial.println("");
      Serial.println("***************** Error occured when reading 2 bytes ******************");
    #endif
    return false;
  }
  temp_data.byte_data[ 0 ] = Wire.read( );
  temp_data.byte_data[ 1 ] = Wire.read( );
  value = temp_data.int_data;
  return true;
}

// Write 4 bytes to RTC RAM
bool set_My_4_bytes( int8_t address, int32_t value )
{
  union long_t temp_data;
  temp_data.long_data = value;
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.write( temp_data.byte_data[ 0 ] );
  Wire.write( temp_data.byte_data[ 1 ] );
  Wire.write( temp_data.byte_data[ 2 ] );
  Wire.write( temp_data.byte_data[ 3 ] );
  return ( Wire.endTransmission( true ) == 0 );
}

// Read 4 bytes from RTC RAM
bool get_My_4_bytes( int8_t address, uint32_t &value )
{
  long_t temp_data;
  value = 0;
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.endTransmission( );
  byte len = Wire.requestFrom( COUNTER_ADDRESS, 4 );
  if ( len == 0 )
  {
    #ifdef debugserial
      Serial.println("");
      Serial.println("********************* Error occured when reading My 4 bytes **************");
    #endif
    return false;
  }
  temp_data.byte_data[ 0 ] = Wire.read( );
  temp_data.byte_data[ 1 ] = Wire.read( );
  temp_data.byte_data[ 2 ] = Wire.read( );
  temp_data.byte_data[ 3 ] = Wire.read( );
  value = temp_data.long_data;
  return true;
}

// Read the external counter value
bool get_Counter( int8_t address, uint32_t &count  )
{
  count = 0;
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.endTransmission( );
  byte len = Wire.requestFrom( COUNTER_ADDRESS, 3 );
  if ( len == 0 )
  {
    #ifdef debugserial
      Serial.println("");
      Serial.println("********************* Error occured when reading Counter *****************");
    #endif
    return false;
  }

  uint8_t b1 = Wire.read();
  uint8_t b2 = Wire.read();
  uint8_t b3 = Wire.read();

  count = bcd2byte( b1 );
  count = count + bcd2byte( b2 ) * 100L;
  count = count + bcd2byte( b3 ) * 10000L;
  return true;
}

// Write counter value in BCD format
bool set_Counter( int8_t address, int32_t value )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.write( byte2bcd(   value % 100L ));
  Wire.write( byte2bcd( ( value / 100L ) % 100L ));
  Wire.write( byte2bcd( ( value / 10000L ) % 100L ));
  return ( Wire.endTransmission( true ) == 0 );
}

// Read time from RTC RAM
void get_Time( int8_t address, uint8_t &o_hour, uint8_t &o_min, uint8_t &o_sec )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.endTransmission( );
  Wire.requestFrom( COUNTER_ADDRESS, 3 );
  o_hour   = Wire.read( );
  o_min    = Wire.read( );
  o_sec    = Wire.read( );
}

// Write time to RTC RAM
bool set_Time( int8_t address, uint8_t o_hour, uint8_t o_min, uint8_t o_sec )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.write( o_hour );
  Wire.write( o_min );
  Wire.write( o_sec );
  return ( Wire.endTransmission( true ) == 0 );
}

// Check of plausibility
bool validDate(uint8_t d, uint8_t m, uint16_t y)
{
  return (d >= 1 && d <= 31 && m >= 1 && m <= 12 && y >= 2000 && y <= 2099);
}

// Write date to RTC RAM
bool set_Date( int8_t address, uint8_t day, uint8_t month, int year )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.write( day );
  Wire.write( month );
  year = year - year_offset;
  Wire.write( (uint8_t)year );
  return ( Wire.endTransmission( true ) == 0 );
}

// Read date from RTC RAM
void get_Date( int8_t address, uint8_t &day, uint8_t &month, uint16_t &year )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.endTransmission();
  Wire.requestFrom( COUNTER_ADDRESS, 3 );
  day   = Wire.read();
  month = Wire.read();
  year  = Wire.read() + year_offset;
}

// Check if months are valid
bool months_valid()
{
  return get_Byte(ADDRESS_MONTHS_VALID) == FLAG_VALID;
}

void set_months_valid(bool ok)
{
  set_Byte(ADDRESS_MONTHS_VALID, ok ? FLAG_VALID : 0x00);
}

bool valid_month_value(uint16_t value)
{
  return value <= 60000;
}

// Write 4-byte value and flag
bool set_My_4_bytes_with_flag(int8_t address, uint32_t value, int8_t flag_address)
{
  bool ok = set_My_4_bytes(address, value);
  if (ok) ok = set_Byte(flag_address, FLAG_VALID);
  return ok;
}

// Read 4-byte value only if flag is valid
bool get_My_4_bytes_with_flag(int8_t address, int8_t flag_address, uint32_t &value)
{
  uint8_t flag = get_Byte(flag_address);
  if (flag != FLAG_VALID)
  {
    value = 0;
    return false;
  }
  return get_My_4_bytes(address, value);
}

// Save the start count for a month
bool set_month(int8_t month, uint16_t current_count)
{
  if (month < 1 || month > 12) return false;
  uint8_t month_address = ADDRESS_MONTH_01 + (month - 1) * 2;
  bool ok = set_Integer(month_address, (int16_t)current_count);
  if (ok) set_months_valid(true);
  return ok;
}

// Read the start count for a month
bool get_month(int8_t month, uint16_t &current_count)
{
  if (month < 1 || month > 12) return false;
  if (!months_valid()) return false;

  uint8_t month_address = ADDRESS_MONTH_01 + (month - 1) * 2;
  int16_t tmp = 0;
  if (!get_Integer(month_address, tmp)) return false;

  current_count = (uint16_t)tmp;
  return valid_month_value(current_count);
}

//
//**************************************************************************************************
//                                          L O G P R I N T                                        *
//**************************************************************************************************
// Log messages if log_msg is true                                                                 *
// Uses vsnprint for format strings, needs /r/n for CR/LF at end                                   *
//**************************************************************************************************
//
// Simple logging helper
char* logprint ( const char* format, ... )
{
  static char sbuf[ 128 ];                             // For debug lines
  va_list varArgs;                                     // For variable number of params
  va_start ( varArgs, format );                        // Prepare parameters
  vsnprintf ( sbuf, sizeof(sbuf), format, varArgs );   // Format the message
  va_end ( varArgs );                                  // End of using parameters
  if ( log_msg ) Serial.print ( sbuf );                // Log Messages on? Info
  return sbuf;                                         // Return stored string
}

// Initialize the hardware counter and time base
void init_counter()
{
  set_Counter_Mode();
  set_Counter( ADDRESS_COUNTER, 0 );
  set_My_4_bytes( ADDRESS_OLD_COUNTER, 0 );
}

// Initialize the day counter
void init_day_counter()
{
  day_state = DAY_STATE_MANUAL_RESET;

  day_start_count = new_count;
  daily_count = 0;
  daily_consumption_l = 0;
  daily_volume_m3 = 0;
  daily_energy_kWh = 0;

  set_My_4_bytes(ADDRESS_DAY_START_COUNT, day_start_count);

  // Check of plausibility
  if (validDate(Gas_day, Gas_month, Gas_year))
  {
    set_Date(ADDRESS_DAY_DATE, Gas_day, Gas_month, Gas_year);
  }
  set_Byte(ADDRESS_DAY_INITIALIZED, 1);
  set_Byte(ADDRESS_DAY_STATE, DAY_STATE_MANUAL_RESET);

  #ifdef debugmqtt
    publish_topic("debug/day_start_count", String(day_start_count));
    publish_topic("debug/daily_count", "0");
    publish_topic("debug/day_initialized", "1");
  #endif  

  MQTT_command = "";
}

// Publish one or all monthly counters
void publish_month( )
{
  uint8_t  cmd_month = 0;
  uint8_t  cmd_value = 0;
  uint8_t  cmd_start = 0;
  uint8_t  cmd_end   = 0;
  uint16_t month_start_count = 0;
  uint32_t month_consumption = 0;
  String   value3;

  consumption_string = MQTT_command.substring( MQTT_command.indexOf(',') + 1);
  cmd_value          = consumption_string.toInt();
  cmd_month          = constrain( cmd_value, 1, 12 );
  value3             = "";

  if ( cmd_value <= 12 )
  {
    cmd_start = cmd_month;
    cmd_end   = cmd_month;
  }
  else
  {
    cmd_start = 1;
    cmd_end   = 12;
  }

  for ( int i = cmd_start; i <= cmd_end; i++ )
  {
    if ( get_month(i, month_start_count) )
    {
      month_consumption = ( new_count >= month_start_count ) ? ( new_count - month_start_count ) * liter_per_count : 0;
    }
    else
    {
      month_consumption = 0;
    }
    value3 = value3 + ';' + String( i ) + ';' + String( month_consumption );
  }

  publish_topic( "month_data", value3 );
  MQTT_command  = "";
}

// Save period start
void set_period( )
{
  uint32_t cmd_data = 0;
  String value3;
  if ( MQTT_command.indexOf(',') > 0 )
  {
    value3 = MQTT_command.substring( MQTT_command.indexOf(',') + 1);
    cmd_data = value3.toInt();
  }
  set_My_4_bytes( ADDRESS_START_PERIOD, cmd_data );
  set_Byte(ADDRESS_START_PERIOD_FLAG, FLAG_VALID);
  start_period = cmd_data;
  MQTT_command = "";
}

// Save meter start
void set_start( )
{
  uint32_t cmd_data = 0;
  String value3;
  if ( MQTT_command.indexOf(',') > 0 )
  {
    value3 = MQTT_command.substring( MQTT_command.indexOf(',') + 1);
    cmd_data = value3.toInt();
  }
  set_My_4_bytes( ADDRESS_START_METER, cmd_data );
  set_Byte(ADDRESS_START_METER_FLAG, FLAG_VALID);
  start_consumption = cmd_data;
  MQTT_command = "";
}

// Initialize counter and optionally save a meter offset
void init_counter_and_data()
{
  uint32_t cmd_data = 0;
  String value3;

  init_counter();

  if (MQTT_command.indexOf(',') > 0)
  {
    value3 = MQTT_command.substring(MQTT_command.indexOf(',') + 1);
    cmd_data = value3.toInt();
  }

  set_My_4_bytes(ADDRESS_START_METER, cmd_data);
  set_Byte(ADDRESS_START_METER_FLAG, FLAG_VALID);
  start_consumption = cmd_data;
  MQTT_command = "";
}

// Save month data from MQTT
void set_month_data( )
{
  uint32_t cmd_data = 0;
  uint8_t  cmd_month = 0;
  String value2;
  String value3;

  value2    = MQTT_command.substring( MQTT_command.indexOf(',') + 1);
  cmd_month = value2.toInt();

  if (( MQTT_command.indexOf(',') > 0 ) &&
      ( cmd_month > 0 ) &&
      ( cmd_month <= 12 ))
  {
    cmd_data = 0;
    if ( MQTT_command.indexOf(';') > 0 )
    {
      value3 = MQTT_command.substring( MQTT_command.indexOf(';') + 1);
      cmd_data = value3.toInt();
    }
    set_month( cmd_month, (uint16_t)cmd_data );
  }

  publish_month( );
  delay( 3000 );
  MQTT_command = "";
}

// Parse incoming MQTT commands
void command_parser()
{
  //********* Publish consumption of a single month or month 1 to 12 ***********************
  // SYNTAX  : "Energy/Gas/command"
  // PAYLOAD : "Month, x"
  // value for x : either 1 to 12 for a single month, x > 12 shows all 12 months
  //****************************************************************************************
  if ( MQTT_command.indexOf( "Month" ) >= 0 ) publish_month();

  //********* Set consumption for the start of a heating period ****************************
  // SYNTAX  : "Energy/Gas/command"
  // PAYLOAD : "InitP, x"
  // value for x : consumption in liter
  //****************************************************************************************
  if ( MQTT_command.indexOf( "InitP" ) >= 0 ) set_period();
  
  //********* Set initial consumption value for count  value of zero ***********************
  // SYNTAX  : "Energy/Gas/command"
  // PAYLOAD : "InitS, x"
  // value for x : consumption in liter
  //****************************************************************************************
  if ( MQTT_command.indexOf( "InitS" ) >= 0 ) set_start();
  
  //********* Set consumption for a single month to init data   ****************************
  // SYNTAX  : "Energy/Gas/command"
  // PAYLOAD : "InitM, x ; y"
  // value for x : selected month
  // value for y : consumption in liter
  //****************************************************************************************
  if ( MQTT_command.indexOf( "InitM" ) >= 0 ) set_month_data();
  
  //********* Set consumption for the strat of a counting period ***************************
  // Initialize RTC chip and set initial consumption value for count = 0of a counting period 
  // SYNTAX  : "Energy/Gas/command"
  // PAYLOAD : "InitC, x"
  // value for x : consumption in liter
  //****************************************************************************************
  if ( MQTT_command.indexOf( "InitC" ) >= 0 ) init_counter_and_data();
  
  //********* Set consumption of the last day to zero **************************************
  // SYNTAX  : "Energy/Gas/command"
  // PAYLOAD : "InitD"
  //****************************************************************************************
  if ( MQTT_command.indexOf( "InitD" ) >= 0 ) init_day_counter();
}

// Initialize hardware, network, MQTT, OTA and RAM-values
void setup()
{
  Serial.begin(115200);
  delay(1000);
  pinMode(RESET_PIN, INPUT_PULLUP);           // pull to ground if you want to clear the RTC Data
  pinMode(UPDATE_PIN, INPUT_PULLUP);          // pull to ground if you want prevent ESP going to sleep


  Wire.setClock(50000);                       // reduce SCL speed to 50kHz
  Wire.begin();

  // Connect to WiFi
  Gas_WiFi_connect();
  begin_time();

  #ifdef THINGSPEAK
    ThingSpeak.begin(client1);                // Initialize Thingspeak
  #endif

  // Optional hardware reset
  if (digitalRead(RESET_PIN) == 0)
  {
    init_counter();
    set_Byte(ADDRESS_DAY_INITIALIZED, 0);     // Beim kompletten Reset auch Initialisierungs-Flag zurücksetzen
    set_Byte(ADDRESS_DAY_STATE, DAY_STATE_NORMAL);
    set_Byte(ADDRESS_MONTHS_VALID, 0x00);
  }

  setup_MQTT();                               // Setup MQTT Client

  // get start consumption
  if (get_My_4_bytes_with_flag(ADDRESS_START_METER, ADDRESS_START_METER_FLAG, start_consumption) != true)
    start_consumption = 0;

  // get period consumption
  if (get_My_4_bytes_with_flag(ADDRESS_START_PERIOD, ADDRESS_START_PERIOD_FLAG, start_period) != true)
    start_period = 0;

  // get day start count from RAM
  if (get_My_4_bytes(ADDRESS_DAY_START_COUNT, day_start_count) != true)
    day_start_count = 0;

  day_initialized = get_Byte(ADDRESS_DAY_INITIALIZED);
  day_state = get_Byte(ADDRESS_DAY_STATE);

  if (day_initialized == 0)
  {
    init_day_counter();
    day_initialized = 1;
  }
  else
  {
    day_state = DAY_STATE_NORMAL;
  }

  // Initialise OTA
  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname("Gascounter");
  ArduinoOTA.setPasswordHash("8839735356e0412c4735811c23214209");
  ArduinoOTA.begin();
}
  //
  //**************************************************************************************************
  //                                          H A S H E D  PWD                                       *
  //**************************************************************************************************
  //           Das hashed pwd kann zum Beispiel gebildet werden über den Hash-Generator              *
  //                                https://hash-generieren.de/                                      *
  //                             https://www.md5hashgenerator.com/                                   *
  //                    https://www.hexhero.com/tools/sha256-hash-generator                          *
  //**************************************************************************************************
  //


#ifdef debugmqtt
  // Publish debug values for MQTT monitoring
  void publish_debug_topics(const char* tag)
  {
    char buf[32];
    publish_topic(MQTT_DEBUG_NEW_COUNT, String(new_count));
    publish_topic(MQTT_DEBUG_DAY_START_COUNT, String(day_start_count));
    publish_topic(MQTT_DEBUG_DAILY_COUNT, String(daily_count));
    publish_topic(MQTT_DEBUG_DAY_INIT, String(day_initialized));
    snprintf(buf, sizeof(buf), "%02d.%02d.%04d", Gas_day, Gas_month, Gas_year);
    publish_topic(MQTT_DEBUG_DATE, buf);
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d", Gas_hour, Gas_min, Gas_sec);
    publish_topic(MQTT_DEBUG_TIME, buf);
    publish_topic(MQTT_DEBUG_TAG, String(tag));
  }
#endif  

// It reads the counter, updates daily/monthly values, publishes data, and then goes to sleep
void loop()
{
  mqtt.loop();
  ArduinoOTA.handle();                  // keep OTA responsive

  error = false;

  if (WiFi.status() != WL_CONNECTED)
    Gas_WiFi_reconnect();

  if (MQTT_command != "")
    command_parser();

  if (get_My_4_bytes(ADDRESS_OLD_COUNTER, old_count) != true)
    old_count = 0;

  if (get_Counter(ADDRESS_COUNTER, new_count) != true)
  {
    error = true;
    new_count = 0;
  }

  if (!get_LocalTime())
    return;

  uint8_t stored_day = 0;
  uint8_t stored_month = 0;
  uint16_t stored_year = 0;

  get_Date(ADDRESS_DAY_DATE, stored_day, stored_month, stored_year);

  // Restore valid values if stored values are corrupt
  if (!validDate(stored_day, stored_month, stored_year))
  {
    #ifdef debugmqtt
      publish_topic("debug/day_change", "before correction: ");
      publish_topic("debug/day_change", "Gas_date: " + String(Gas_day) + "." + String(Gas_month) + "." + String(Gas_year));
      publish_topic("debug/day_change", "stored_date: " + String(stored_day) + "." + String(stored_month) + "." + String(stored_year));
        #endif
    stored_day = Gas_day;
    stored_month = Gas_month;
    stored_year = Gas_year;
    #ifdef debugmqtt
      publish_topic("debug/day_change", "after correction: ");
      publish_topic("debug/day_change", "Gas_date: " + String(Gas_day) + "." + String(Gas_month) + "." + String(Gas_year));
      publish_topic("debug/day_change", "stored_date: " + String(stored_day) + "." + String(stored_month) + "." + String(stored_year));
    #endif
    set_Date(ADDRESS_DAY_DATE, stored_day, stored_month, stored_year);
    set_My_4_bytes(ADDRESS_DAY_START_COUNT, new_count);
    day_start_count = new_count;
    daily_count = 0;
    day_initialized = 1;
    day_state = DAY_STATE_NORMAL;
    set_Byte(ADDRESS_DAY_INITIALIZED, 1);
    set_Byte(ADDRESS_DAY_STATE, DAY_STATE_NORMAL);
    #ifdef debugmqtt
      publish_topic("debug/day_change", "invalid date -> init");
    #endif
  }
  else if ((Gas_day != stored_day) || (Gas_month != stored_month) || (Gas_year != stored_year))
  {
    day_start_count = new_count;
    daily_count = 0;
    set_My_4_bytes(ADDRESS_DAY_START_COUNT, day_start_count);
    set_Date(ADDRESS_DAY_DATE, Gas_day, Gas_month, Gas_year);
    #ifdef debugmqtt
      publish_topic("debug/day_change", "day change");
      publish_topic("debug/day_change", "Gas_date: " + String(Gas_day) + "." + String(Gas_month) + "." + String(Gas_year));
      publish_topic("debug/day_change", "stored_date: " + String(stored_day) + "." + String(stored_month) + "." + String(stored_year));
      publish_topic("debug/day_change", "day_start_count: " + String(day_start_count));
      publish_topic("debug/day_change", "new_count: " + String(new_count));
    #endif
  }
  else
  {
    // Correction of any implausible condition
    if (day_start_count > new_count)
      day_start_count = new_count;

    daily_count = new_count - day_start_count;
    #ifdef debugmqtt
      publish_topic("debug/day_change", "no day change");
      publish_topic("debug/day_change", "Gas_date: " + String(Gas_day) + "." + String(Gas_month) + "." + String(Gas_year));
      publish_topic("debug/day_change", "stored_date: " + String(stored_day) + "." + String(stored_month) + "." + String(stored_year));
      publish_topic("debug/day_change", "day_start_count: " + String(day_start_count));
      publish_topic("debug/day_change", "new_count: " + String(new_count));
    #endif
  }

  daily_consumption_l = daily_count * liter_per_count;
  daily_volume_m3     = daily_count * mexp3_per_count;
  daily_energy_kWh    = daily_count * mexp3_per_count * brennwert * zustandszahl;

  #ifdef debugmqtt
    publish_debug_topics("after_daily_calc");
  #endif

  // Save previous counter/time/date for the next loop iteration
  set_My_4_bytes(ADDRESS_OLD_COUNTER, new_count);

  consumption  = (start_consumption + new_count * liter_per_count) / 1000.0;
  total_energy = consumption * brennwert * zustandszahl;
  energy1      = (double)delta_count * (double)mexp3_per_count * brennwert * zustandszahl;

  dtostrf(total_energy, 12, 3, valueString1);
  dtostrf(energy1, 12, 3, valueString2);
  dtostrf(consumption, 12, 3, valueString3);
  dtostrf(new_count, 10, 0, valueString4);
  dtostrf(energy1, 12, 3, valueString5);
  dtostrf(start_consumption / 1000.0, 12, 3, valueString6);
  dtostrf(start_period / 1000.0, 12, 3, valueString7);
  dtostrf(daily_consumption_l, 12, 0, valueString9);
  dtostrf(daily_volume_m3, 12, 3, valueString10);
  dtostrf(daily_energy_kWh, 12, 3, valueString11);
  sprintf(valueString8, "%02d:%02d:%02d", Gas_hour, Gas_min, Gas_sec);

  // Publish to Thingspeak
  #ifdef THINGSPEAK
    ThingSpeak.setField(1, valueString1);
    ThingSpeak.setField(2, valueString2);
    ThingSpeak.setField(3, valueString3);
    ThingSpeak.setField(4, valueString4);
    ThingSpeak.setField(5, valueString5);
    ThingSpeak.setField(6, valueString9);
    ThingSpeak.setField(7, valueString10);
    ThingSpeak.setField(8, valueString11);
    Send_data();
  #endif

  publish_topic("Time",          valueString8);
  publish_topic("Total_kWh",     valueString1);
  publish_topic("Power_kW",      valueString5);
  publish_topic("Counts",        valueString4);
  publish_topic("Volume",        valueString3);
  publish_topic("Start",         valueString6);
  publish_topic("Period_Start",  valueString7);
  publish_topic("Daily_Liter",   valueString9);
  publish_topic("Daily_m3",      valueString10);
  publish_topic("Daily_kWh",     valueString11);
  publish_topic("Status",        error == true ? "Error" : "OK");

  if (digitalRead(UPDATE_PIN) == HIGH)
  {
    delay(1000);
    ESP.deepSleep(111 * 1000 * 1000);
  }
  else
  {
    delay(3000);
  }
}