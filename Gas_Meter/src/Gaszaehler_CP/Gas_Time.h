// Time routines for Gas Logger
//
// Time routines for GAS Counter
//

#include "time.h"

struct txx                         // just for a reminder of data available
{
  int tm_sec;
  int tm_min;
  int tm_hour;
  int tm_mday;
  int tm_mon;
  int tm_year;
  int tm_wday;
  int tm_yday;
  int tm_isdst;
#ifdef __TM_GMTOFF
  long  __TM_GMTOFF;
#endif
#ifdef __TM_ZONE
  const char *__TM_ZONE;
#endif
};

const char* ntpServer          = "pool.ntp.org";
const long  gmtOffset_sec      = 3600;
const int   daylightOffset_sec = 3600;

int         Gas_hour;
int         Gas_min;
int         Gas_sec;
int         Gas_month;
int         Gas_day;
int         Gas_year;

//
// Time routines
//
//***********************************************************************************************

// get_LocalTime() returns true if time was successfully obtained
bool get_LocalTime()
{
  struct tm timeinfo;

  if (WiFi.status() != WL_CONNECTED)
  {
    Gas_WiFi_reconnect();
  }

  if (!getLocalTime(&timeinfo))
  {
    return false;
  }

  Gas_hour  = timeinfo.tm_hour;
  Gas_min   = timeinfo.tm_min;
  Gas_sec   = timeinfo.tm_sec;
  Gas_day   = timeinfo.tm_mday;
  Gas_month = timeinfo.tm_mon + 1;
  Gas_year  = timeinfo.tm_year + 1900;

  return true;
}

void begin_time()
{
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Wait for time to be available (up to 5 seconds)
  struct tm timeinfo;
  unsigned long timeout = millis();
  while (!getLocalTime(&timeinfo))
  {
    if (millis() - timeout > 5000)
    {
      Serial.println("Warning: Failed to get time after 5 seconds");
      return;
    }
    delay(100);
  }

  // Copy time to global variables
  get_LocalTime();
}