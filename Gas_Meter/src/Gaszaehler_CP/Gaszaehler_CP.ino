/**************************************************************************/
/*
  Use RTC as counter and read results with TWI
  Store last value in RAM
*/
/**************************************************************************/
#define THINGSPEAK

#include <Wire.h>
#include <PicoMQTT.h>
#include "Gas_WiFi.h"
#include "Gas_MQTT.h"
#include "Gas_Time.h"
#include <ArduinoOTA.h>
#ifdef THINGSPEAK
  #include "Gas_Thingspeak.h"
#endif

#define RESET_PIN                D7
#define UPDATE_PIN               D6

#define COUNTER_MODE             0x20
#define COUNTER_ADDRESS          0x50

#define CONTROL_ADDRESS          0x00
#define ADDRESS_COUNTER          0x01

#define RAM_OFFSET               0x10
#define ADDRESS_OLD_COUNTER      RAM_OFFSET
#define ADDRESS_OLD_TIME         ADDRESS_OLD_COUNTER      + 4
#define ADDRESS_OLD_DATE         ADDRESS_OLD_TIME         + 4
#define ADDRESS_START_METER      ADDRESS_OLD_DATE         + 4
#define ADDRESS_START_PERIOD     ADDRESS_START_METER      + 4
#define ADDRESS_MONTH_01         ADDRESS_START_PERIOD     + 4

#define ADDRESS_DAY_START_COUNT  ADDRESS_MONTH_01         + ( 4 * 12 )
#define ADDRESS_DAY_DATE         ADDRESS_DAY_START_COUNT  + 4
#define ADDRESS_DAY_INITIALIZED  ADDRESS_DAY_DATE         + 4

#define FLAG_VALID               0xA5
#define ADDRESS_START_METER_FLAG   (ADDRESS_START_METER + 4)
#define ADDRESS_START_PERIOD_FLAG  (ADDRESS_START_PERIOD + 4)
#define ADDRESS_MONTH_FLAG_BASE    (ADDRESS_MONTH_01 + (4 * 12))

bool      sleep                = false;
bool      error                = false;
bool      log_msg              = true;
uint32_t  old_time;
uint8_t   old_date[ 3 ];
uint32_t  new_time;
uint8_t   new_date[ 3 ];
uint32_t  old_count            = 0;
uint32_t  new_count            = 0;
uint32_t  delta_count          = 0;
uint32_t  liter_per_count      = 10;
float     mexp3_per_count      = 0.01;
double    brennwert            = 11.688629;
double    zustandszahl         = 0.93940;
uint16_t  year_offset          = 2000;
float     energy1              = 0;
double    total_energy         = 0;
uint32_t  start_consumption    = 0;
double    consumption          = 0;
uint32_t  start_period         = 0;
uint32_t  period               = 0;
uint32_t  consumption_data     = 0;
String    consumption_string   = "";
uint8_t   month_data           = 0;
uint32_t  temp_data            = 0;
uint32_t day_start_count       = 0;
uint32_t daily_count           = 0;
double daily_consumption_l     = 0;
double daily_volume_m3         = 0;
double daily_energy_kWh        = 0;
uint8_t last_day               = 0;
uint8_t last_month             = 0;
uint16_t last_year             = 0;
uint8_t day_initialized        = 0;

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

uint8_t old_day;
uint8_t old_month;
uint8_t old_hour;
uint8_t old_min;
uint8_t old_sec;

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

uint8_t bcd2byte( uint8_t value )
{
  return (( value >> 4 ) * 10 ) + ( value & 0x0f );
}

uint8_t byte2bcd( uint8_t value )
{
  return (( value / 10) << 4 ) + ( value % 10 );
}

bool set_Counter_Mode( )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( CONTROL_ADDRESS );
  Wire.write( COUNTER_MODE );
  return ( Wire.endTransmission( true ) == 0 );
}

uint8_t get_Mode( )
{
  return get_Byte( CONTROL_ADDRESS );
}

bool set_Byte( int8_t address, int8_t value )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.write( value );
  return ( Wire.endTransmission( true ) == 0 );
}

int8_t get_Byte( int8_t address )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.endTransmission( );
  Wire.requestFrom( COUNTER_ADDRESS, 1 );
  return Wire.read( );
}

bool set_Integer( int8_t address, int16_t value )
{
  int_t  temp_data;
  temp_data.int_data = value;
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.write( temp_data.byte_data[ 0 ] );
  Wire.write( temp_data.byte_data[ 1 ] );
  return ( Wire.endTransmission( true ) == 0 );
}

bool get_Integer( int8_t address, int16_t &value )
{
  int_t  temp_data;

  value = 0;

  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.endTransmission( );
  byte len = Wire.requestFrom( COUNTER_ADDRESS, 2 );
  if ( len == 0 )
  {
    Serial.println("");
    Serial.println("***************** Error occured when reading 2 bytes ******************");
    return false;
  }
  else
  {
    temp_data.byte_data[ 0 ] = Wire.read( );
    temp_data.byte_data[ 1 ] = Wire.read( );
    value                    = temp_data.int_data;
    return true;
  }
}

bool set_My_4_bytes( int8_t address, int32_t value )
{
  union long_t  temp_data;
  temp_data.long_data = value;
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write(  address );
  Wire.write(  temp_data.byte_data[ 0 ] );
  Wire.write(  temp_data.byte_data[ 1 ] );
  Wire.write(  temp_data.byte_data[ 2 ] );
  Wire.write(  temp_data.byte_data[ 3 ] );
  return ( Wire.endTransmission( true ) == 0 );
}

bool get_My_4_bytes( int8_t address, uint32_t &value )
{
  long_t  temp_data;

  value = 0;

  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.endTransmission( );
  byte len = Wire.requestFrom( COUNTER_ADDRESS, 4 );
  if ( len == 0 )
  {
    Serial.println("");
    Serial.println("********************* Error occured when reading My 4 bytes **************");
    return false;
  }
  else
  {
    temp_data.byte_data[ 0 ] = Wire.read( );
    temp_data.byte_data[ 1 ] = Wire.read( );
    temp_data.byte_data[ 2 ] = Wire.read( );
    temp_data.byte_data[ 3 ] = Wire.read( );
    value                    = temp_data.long_data;
    return true;
  }
}

bool get_Counter( int8_t address, uint32_t &count  )
{
  count = 0;
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.endTransmission( );
  byte len = Wire.requestFrom( COUNTER_ADDRESS, 3);

  if (len == 0)
  {
    Serial.println("");
    Serial.println("********************* Error occured when reading Counter *****************");
    return false;
  }
  else
  {
    count =         bcd2byte( Wire.read( ));
    count = count + bcd2byte( Wire.read( )) * 100L;
    count = count + bcd2byte( Wire.read( )) * 10000L;
    return true;
  }
}

bool set_Counter( int8_t address, int32_t value )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.write( byte2bcd(   value % 100L ));
  Wire.write( byte2bcd( ( value / 100L ) % 100L ));
  Wire.write( byte2bcd( ( value / 10000L ) % 100L ));
  return ( Wire.endTransmission( true ) == 0 );
}

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

bool set_Time( int8_t address, uint8_t o_hour, uint8_t o_min, uint8_t o_sec )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address );
  Wire.write( o_hour );
  Wire.write( o_min  );
  Wire.write( o_sec  );
  return ( Wire.endTransmission( true ) == 0 );
}

bool set_Date( int8_t address, uint8_t day, uint8_t month, int year )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address  );
  Wire.write( day ) ;
  Wire.write( month );
  year = year - year_offset ;
  Wire.write( year );
  return ( Wire.endTransmission( true ) == 0 );
}

void get_Date( int8_t address, uint8_t &day, uint8_t &month, uint16_t &year )
{
  Wire.beginTransmission( COUNTER_ADDRESS );
  Wire.write( address  );
  Wire.endTransmission();
  Wire.requestFrom( COUNTER_ADDRESS, 3 );

  day   = Wire.read( );
  month = Wire.read( );
  year  = Wire.read( ) + year_offset;
}

bool set_My_4_bytes_with_flag(int8_t address, uint32_t value, int8_t flag_address)
{
  bool ok = set_My_4_bytes(address, value);
  if (ok) ok = set_Byte(flag_address, FLAG_VALID);
  return ok;
}

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

bool set_month( int8_t month, uint32_t current_count )
{
  uint8_t month_address = ADDRESS_MONTH_01 + ( month - 1 ) * 4;
  uint8_t flag_address  = ADDRESS_MONTH_FLAG_BASE + ( month - 1 );

  if (!set_My_4_bytes(month_address, current_count))
    return false;

  return set_Byte(flag_address, FLAG_VALID);
}

uint32_t get_month( int8_t month )
{
  uint8_t month_address = ADDRESS_MONTH_01 + ( month - 1 ) * 4;
  uint8_t flag_address  = ADDRESS_MONTH_FLAG_BASE + ( month - 1 );
  uint32_t month_start_count = 0;

  if (get_Byte(flag_address) != FLAG_VALID)
    return 0;

  if ( get_My_4_bytes( month_address, month_start_count ) != true )
    return 0;

  return month_start_count;
}

uint32_t get_month_consumption( int8_t month, uint32_t current_count )
{
  uint32_t month_start_count = get_month( month );
  uint32_t consumption = 0;

  if ( current_count >= month_start_count )
  {
    consumption = ( current_count - month_start_count ) * liter_per_count;
  }

  return consumption;
}

char* logprint ( const char* format, ... )
{
  static char sbuf[ 128 ] ;
  va_list     varArgs ;

  va_start ( varArgs, format ) ;
  vsnprintf ( sbuf, sizeof(sbuf), format, varArgs ) ;
  va_end ( varArgs ) ;
  if ( log_msg )
  {
    Serial.print ( sbuf ) ;
  }
  return sbuf ;
}

void Init_counter()
{
  logprint( "***  Set Counter Mode *******" );
  if ( set_Counter_Mode( ) == false )
     logprint( "***  failed to set Counter Mode *******");
  logprint( "***  Clear counter data *******" );
  if ( set_Counter    ( ADDRESS_COUNTER, 0 ) == false )
     logprint( "***  Failed to clear counter *******");
  if ( set_My_4_bytes ( ADDRESS_OLD_COUNTER, 0 ) == false )
     logprint( "***  Failed to clear old counter data *******");
  logprint( "***  Clear Time *******" );
  set_Time( ADDRESS_OLD_TIME, 0, 0, 0 );
  logprint( "***  Set Time *******" );
  set_Time( ADDRESS_OLD_TIME, Gas_hour, Gas_min, Gas_sec );
}

void publish_month( )
{
   uint8_t  cmd_month = 0;
   uint8_t  cmd_value = 0;
   uint8_t  cmd_start = 0;
   uint8_t  cmd_end   = 0;
   uint32_t month_start_count = 0;
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
      uint8_t month_address = ADDRESS_MONTH_01 + ( i - 1 ) * 4;
      uint8_t flag_address  = ADDRESS_MONTH_FLAG_BASE + ( i - 1 );

      if (get_Byte(flag_address) == FLAG_VALID)
      {
        get_My_4_bytes( month_address, month_start_count );
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

void set_period( )
{
   uint32_t cmd_data  = 0;
   String   value3;

   if ( MQTT_command.indexOf(',') > 0 )
   {
      value3    = MQTT_command.substring( MQTT_command.indexOf(',') + 1);
      cmd_data  = value3.toInt();
   }
   if ( set_My_4_bytes( ADDRESS_START_PERIOD, cmd_data ) == false )
   {
      logprint( "Failed to save Start Period %lu\n\r", cmd_data );
   }
   set_Byte(ADDRESS_START_PERIOD_FLAG, FLAG_VALID);
   start_period = cmd_data;
   MQTT_command = "";
}

void set_start( )
{
   uint32_t cmd_data  = 0;
   String   value3;

   if ( MQTT_command.indexOf(',') > 0 )
   {
      value3    = MQTT_command.substring( MQTT_command.indexOf(',') + 1);
      cmd_data  = value3.toInt();
   }
   if ( set_My_4_bytes( ADDRESS_START_METER, cmd_data ) == false )
   {
      logprint( "Failed to save Start Period %lu\n\r", cmd_data );
   }
   set_Byte(ADDRESS_START_METER_FLAG, FLAG_VALID);
   start_consumption = cmd_data;
   MQTT_command = "";
}

void init_counter_and_data( )
{
   uint32_t cmd_data  = 0;
   String   value3;

   Init_counter( );

   if ( MQTT_command.indexOf(',') > 0 )
   {
      value3    = MQTT_command.substring( MQTT_command.indexOf(',') + 1);
      cmd_data  = value3.toInt();
   }

   if ( set_My_4_bytes( ADDRESS_START_METER, cmd_data ) == false )
   {
      logprint( "Failed to save Start Consumption %lu\n\r", cmd_data );
   }

   set_Byte(ADDRESS_START_METER_FLAG, FLAG_VALID);
   start_consumption = cmd_data;

   set_My_4_bytes(ADDRESS_DAY_START_COUNT, 0);
   set_Byte(ADDRESS_DAY_INITIALIZED, 0);
   day_start_count = 0;
   day_initialized = 0;

   MQTT_command = "";
}

void set_month_data( )
{
   uint32_t cmd_data    = 0;
   uint8_t  cmd_month   = 0;
   uint8_t  cmd_address = 0;
   String   value2;
   String   value3;

   value2    = MQTT_command.substring( MQTT_command.indexOf(',') + 1);
   cmd_month = value2.toInt();

   if (( MQTT_command.indexOf(',') > 0   ) &&
       ( cmd_month                       > 0   ) &&
       ( cmd_month                       <= 12 ))
   {
      cmd_data     = 0;
      if ( MQTT_command.indexOf(';') > 0 )
      {
         value3    = MQTT_command.substring( MQTT_command.indexOf(';') + 1);
         cmd_data  = value3.toInt();
      }
      cmd_address  = ADDRESS_MONTH_01  + ( cmd_month - 1 ) * 4;
      set_My_4_bytes( cmd_address, cmd_data );
      set_Byte(ADDRESS_MONTH_FLAG_BASE + (cmd_month - 1), FLAG_VALID);
   }

   publish_month( );
   delay( 3000 );
   MQTT_command = "";
}

void command_parser()
{
  if ( MQTT_command.indexOf( "Month" ) >= 0 )
  {
    publish_month( );
  }

  if ( MQTT_command.indexOf( "InitP" ) >= 0 )
  {
    set_period( );
  }

  if ( MQTT_command.indexOf( "InitS" ) >= 0 )
  {
    set_start( );
  }

  if ( MQTT_command.indexOf( "InitM" ) >= 0 )
  {
    set_month_data( );
  }

  if ( MQTT_command.indexOf( "InitC" ) >= 0 )
  {
    init_counter_and_data( );
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(UPDATE_PIN, INPUT_PULLUP);
  logprint("\r\nStarting Test!\r\n");

  Wire.setClock(50000);
  Wire.begin();

  Gas_WiFi_connect();
  delay(1000);
  logprint("\r\nConnected to WiFi!\r\n");

  begin_time();
  logprint("Setup: Time = %02d.%02d.%04d %02d:%02d:%02d\n\r",
         Gas_day, Gas_month, Gas_year, Gas_hour, Gas_min, Gas_sec);

  delay(1000);

  #ifdef THINGSPEAK
    ThingSpeak.begin(client1);
  #endif

  if (digitalRead(RESET_PIN) == 0)
  {
    logprint("\r\nClear RTC Data in RAM!\r\n");
    Init_counter();
    set_Byte(ADDRESS_DAY_INITIALIZED, 0);
  }

  setup_MQTT();

  if (get_My_4_bytes_with_flag(ADDRESS_START_METER, ADDRESS_START_METER_FLAG, start_consumption) != true)
    start_consumption = 0;

  if (get_My_4_bytes_with_flag(ADDRESS_START_PERIOD, ADDRESS_START_PERIOD_FLAG, start_period) != true)
    start_period = 0;

  if (get_My_4_bytes(ADDRESS_DAY_START_COUNT, day_start_count) != true)
    day_start_count = 0;

  uint8_t ram_day, ram_month;
  uint16_t ram_year;
  get_Date(ADDRESS_DAY_DATE, ram_day, ram_month, ram_year);

  day_initialized = get_Byte(ADDRESS_DAY_INITIALIZED);

  logprint("\r\nsetup: day_initialized=%d, day_start_count=%lu, ram_date=%02d.%02d.%04d\n\r",
           day_initialized, day_start_count, ram_day, ram_month, ram_year);

  if (day_initialized == 0)
  {
    uint32_t initial_count = 0;
    if (get_Counter(ADDRESS_COUNTER, initial_count) == true)
      day_start_count = initial_count;
    else
      day_start_count = 0;

    set_My_4_bytes(ADDRESS_DAY_START_COUNT, day_start_count);
    set_Date(ADDRESS_DAY_DATE, Gas_day, Gas_month, Gas_year);
    set_Byte(ADDRESS_DAY_INITIALIZED, 1);

    logprint("\r\nFirst init: day_start_count = %lu, day = %02d.%02d.%04d\n\r",
              day_start_count, Gas_day, Gas_month, Gas_year);
  }
  else
  {
    if ((ram_day != Gas_day) || (ram_month != Gas_month) || (ram_year != Gas_year))
    {
      logprint("\r\nsetup: Day change detected before first loop: old=%02d.%02d.%04d new=%02d.%02d.%04d\n\r",
               ram_day, ram_month, ram_year, Gas_day, Gas_month, Gas_year);

      uint32_t current_count = 0;
      if (get_Counter(ADDRESS_COUNTER, current_count) == true)
        day_start_count = current_count;
      else
        day_start_count = 0;

      set_My_4_bytes(ADDRESS_DAY_START_COUNT, day_start_count);
      set_Date(ADDRESS_DAY_DATE, Gas_day, Gas_month, Gas_year);

      logprint("\r\nsetup: day_start_count updated to %lu\n\r", day_start_count);
    }
  }

  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname("Gascounter");
  ArduinoOTA.setPasswordHash("8839735356e0412c4735811c23214209");

  ArduinoOTA.onStart([]()
  {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else
      type = "filesystem";
    Serial.println("Starting update" + type);
  });

  ArduinoOTA.onEnd([]()
  {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
}

void loop()
{
  mqtt.loop();
  error = false;

  if (WiFi.status() != WL_CONNECTED)
    Gas_WiFi_reconnect();

  if (MQTT_command != "")
    command_parser();

  if (get_My_4_bytes(ADDRESS_OLD_COUNTER, old_count) != true)
    old_count = 0;

  bool counter_ok = get_Counter(ADDRESS_COUNTER, new_count);
  if (counter_ok != true)
  {
    error = true;
    new_count = 0;
  }

  logprint("***  New Count %10d Old Count %10d\n\r", new_count, old_count);

  if (!get_LocalTime())
  {
    logprint("ERROR: Failed to get local time, skipping this loop\n\r");
    ArduinoOTA.handle();
    return;
  }

  logprint("Datum %02d.%02d.%04d Uhrzeit  %02d:%02d:%02d\n\r",
           Gas_day, Gas_month, Gas_year, Gas_hour, Gas_min, Gas_sec);

  uint8_t old_day_ram, old_month_ram;
  uint16_t old_year_ram;
  get_Date(ADDRESS_DAY_DATE, old_day_ram, old_month_ram, old_year_ram);

  Serial.print("RAM date: "); Serial.print(old_day_ram);
  Serial.print("."); Serial.print(old_month_ram);
  Serial.print("."); Serial.println(old_year_ram);

  if ((Gas_day != old_day_ram) || (Gas_month != old_month_ram) || (Gas_year != old_year_ram))
  {
    logprint("Day change detected: old=%02d.%02d.%04d new=%02d.%02d.%04d\n\r",
             old_day_ram, old_month_ram, old_year_ram,
             Gas_day, Gas_month, Gas_year);

    day_start_count = new_count;
    set_My_4_bytes(ADDRESS_DAY_START_COUNT, day_start_count);
    set_Date(ADDRESS_DAY_DATE, Gas_day, Gas_month, Gas_year);

    logprint("Day change: day_start_count = %lu, new_count = %lu\n\r",
             day_start_count, new_count);
  }
  else
  {
    Serial.print("NO day change: day_start_count="); Serial.println(day_start_count);
    Serial.print("NO day change: new_count="); Serial.println(new_count);
  }

  if (new_count >= day_start_count)
  {
    daily_count = new_count - day_start_count;
  }
  else
  {
    logprint("Counter reset detected: new_count=%lu < day_start_count=%lu\n\r",
             new_count, day_start_count);
    day_start_count = new_count;
    set_My_4_bytes(ADDRESS_DAY_START_COUNT, day_start_count);
    daily_count = 0;
  }

  daily_consumption_l = daily_count * liter_per_count;
  daily_volume_m3     = daily_count * mexp3_per_count;
  daily_energy_kWh    = daily_count * mexp3_per_count * brennwert * zustandszahl;

  if (set_My_4_bytes(ADDRESS_OLD_COUNTER, new_count) == false)
  {
    logprint("Failed to save New to Old Count ***  New Count %10d Old Count %10d\n\r",
             new_count, old_count);
  }

  get_Time(ADDRESS_OLD_TIME, old_hour, old_min, old_sec);

  if (set_Time(ADDRESS_OLD_TIME, Gas_hour, Gas_min, Gas_sec) == false)
  {
    logprint("Failed to save New to Time ***  New Time %02d:%02d:%02d\n\r",
             Gas_hour, Gas_min, Gas_sec);
  }

  uint8_t old_day_tmp, old_month_tmp;
  uint16_t old_year_tmp;
  get_Date(ADDRESS_OLD_DATE, old_day_tmp, old_month_tmp, old_year_tmp);

  if (set_month(Gas_month, new_count) == false)
  {
    logprint("Failed to save monthly data Gas_month %02d New Count:%06d\n\r",
             Gas_month, new_count);
  }

  set_Date(ADDRESS_OLD_DATE, Gas_day, Gas_month, Gas_year);

  period = (Gas_hour - old_hour) * 3600L +
           (Gas_min - old_min) * 60 +
           (Gas_sec - old_sec);

  new_time = (Gas_hour) * 3600L +
             (Gas_min) * 60 +
             (Gas_sec);

  old_time = (old_hour) * 3600L +
             (old_min) * 60 +
             (old_sec);

  logprint("Old Time %02d:%02d:%02d New Time %02d:%02d:%02d Period %10d\n\r",
           old_hour, old_min, old_sec, Gas_hour, Gas_min, Gas_sec, period);
  logprint("Old Time %08d New time %06d  in seconds\n\r", old_time, new_time);

  delta_count = new_count - old_count;

  consumption = (start_consumption + new_count * liter_per_count) / 1000.0;
  total_energy = consumption * brennwert * zustandszahl;
  energy1      = (double)delta_count * (double)mexp3_per_count * brennwert * zustandszahl;

  dtostrf(total_energy, 12, 3, valueString1);
  dtostrf(energy1, 12, 3, valueString2);
  dtostrf(consumption, 12, 3, valueString3);
  dtostrf(new_count, 10, 0, valueString4);
  dtostrf(3600L / (new_time - old_time) * energy1, 12, 3, valueString5);
  dtostrf(start_consumption / 1000.0, 12, 3, valueString6);
  dtostrf(start_period / 1000.0, 12, 3, valueString7);

  dtostrf(daily_consumption_l, 12, 0, valueString9);
  dtostrf(daily_volume_m3, 12, 3, valueString10);
  dtostrf(daily_energy_kWh, 12, 3, valueString11);

  sprintf(valueString8, "%02d:%02d:%02d", Gas_hour, Gas_min, Gas_sec);

  #ifdef THINGSPEAK
  {
    ThingSpeak.setField(1, valueString1);
    ThingSpeak.setField(2, valueString2);
    ThingSpeak.setField(3, valueString3);
    ThingSpeak.setField(4, valueString4);
    ThingSpeak.setField(5, valueString5);
    ThingSpeak.setField(6, valueString9);
    ThingSpeak.setField(7, valueString10);
    ThingSpeak.setField(8, valueString11);
    Send_data();
  }
  #endif

  {
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
    publish_topic("Status", error == true ? "Error" : "OK");
  }

  if (digitalRead(UPDATE_PIN) == 1)
  {
    logprint("Going to sleep\n");
    delay(1000);
    ESP.deepSleep(111 * 1000 * 1000);
  }
  else
  {
    logprint("Waiting 30 seconds\n");
    delay(30000);
  }

  ArduinoOTA.handle();
}