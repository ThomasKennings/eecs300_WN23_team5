#include <WiFi.h>
#include <esp_task_wdt.h>
#define BUTTON_PIN 0//boot button

void IRAM_ATTR reset_req_TSR();
void measure_delta_time(uint32_t len);//TODO: modify this function (found below) to print
                                     //max, and min delta times in addition to the current one

const char *ssid = "team0005";  // TODO: Fill in with team number, must match in client sketch
const char *password = "password";  // At least 8 chars, must match in client sketch

WiFiServer server(80);
volatile int32_t count = 0;
volatile uint32_t resetRequestFlag = 0;
volatile uint32_t lastResetTime = 0;
volatile uint32_t isFirstMeasurement = 1;

void setup()
{
  Serial.begin(115200);
  
  // WiFi connection procedure
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.print("ESP32 IP as soft AP: ");
  Serial.println(WiFi.softAPIP());
  
  server.begin();

  //watchdog timer with 5s period
  esp_task_wdt_init(5, true); //enable watchdog (which will restart ESP32 if it hangs)
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  
  Serial.println("server started\n");

  // Built-in button, active low
  pinMode(BUTTON_PIN, INPUT);
  attachInterrupt(BUTTON_PIN, reset_req_TSR, FALLING);
  lastResetTime = millis();
}

void loop()
{
  WiFiClient client = server.available();
  client.setTimeout(2);//will wait for maximum of 2 seconds for data
  if (client)
  {
    if (client.connected())
    {
      String line = client.readStringUntil('\n');
      //print updated count or the received line
      //note that if the received line starts with '-', '+', or '#', the code will assume we are decrementing, incrementing, or setting the count, respectively
      //recieved lines starting with any other character will be printed to the serial monitor
      //more cases can be added
      measure_delta_time(line.length());//comment this out if you don't want to see this info printed
      switch(line[0])
      {
        case '-'  : Serial.printf("updated count: %u\r\n", --count);
          break;
        case '+'  : Serial.printf("updated count: %u\r\n", ++count);
          break;
        case '#'  : Serial.printf("updated count: %u\r\n", count = line.substring(1).toInt());
          break;
        case '\0' : //nothing to do if empty String
          break;
        default   : Serial.println(line);
          if(line.indexOf("client started") >= 0) resetRequestFlag = 0;//indicates reset was sucessful
      }
      //if flag is set, we send a reset request
      if (resetRequestFlag)
      {
        client.print("r\n");
        Serial.println("client reset!");
        lastResetTime = millis(); 
      }
      else client.print("\n");//print something to client so it doesn't have to wait for entirety of timeout when checking for reset
      client.stop();
    }
  }
  esp_task_wdt_reset();
}

//set reset flag if boot button is pressed
void IRAM_ATTR reset_req_TSR()
{
  resetRequestFlag = 1;
}

//TODO: modify this function to print max, and min delta times instead of current ones
//if length is > 0 (in other words. not empty String) calculate dela time and print
void measure_delta_time(uint32_t len)
{
  static uint32_t prevTime = 0;
  uint32_t currTime = millis();
  if(len && !isFirstMeasurement && !resetRequestFlag)
  {
    uint32_t deltaTime = currTime - prevTime;
    Serial.printf("Delta time: %u ms\r\n", (uint32_t) deltaTime);
  }
  isFirstMeasurement = 0;
  prevTime = currTime;
}
