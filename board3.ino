#include <WiFi.h>
#include "Esp32MQTTClient.h"
#include <WiFiManager.h> 
#include "driver/pcnt.h"   //Pulse counter library
#include <ArduinoJson.h>
#include <ezTime.h>     
#include <driver/adc.h>

//// PULSE COUNTER MODULES ////
#define PCNT0_FREQ_UNIT      PCNT_UNIT_0     // select ESP32 pulse counter unit 0 (out of 0 to 7 indipendent counting units) for FL2
#define PCNT1_FREQ_UNIT      PCNT_UNIT_1     // select ESP32 pulse counter unit 1 for FL3
                                            // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html
int16_t PulseCounter0 =     0;                                // pulse counter 0 for FL2, max. value is 65536
int16_t PulseCounter1 =     0;                                // pulse counter 1 for FL3, max. value is 65536
int OverflowCounter0 =      0;                                // pulse counter overflow counter 0
int OverflowCounter1 =      0;                                // pulse counter overflow counter 1
int PCNT_H_LIM_VAL =       30000;                            // upper limit of counting  max. 32767, write +1 to overflow counter, when reached 
uint16_t PCNT_FILTER_VAL=  0;                             // filter (damping, inertia) value for avoiding glitches in the count, max. 1023
pcnt_isr_handle_t user_isr_handle0 = NULL;                 // user interrupt handler (not used)
pcnt_isr_handle_t user_isr_handle1 = NULL;                 // user interrupt handler (not used)
RTC_DATA_ATTR int FL2_liters = 0;                             // number of liters that pass through the flux sensor FL2
RTC_DATA_ATTR int FL3_liters = 0;                             // number of liters that pass through the flux sensor FL3
#define PULSES_PER_LITER 165                              // 165 pulses from the flux sensor tell that a liter has passed
//// Deep sleep params ////
#define uS_TO_S_FACTOR 1000000ULL   /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60           /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;

//// Electrovalves status variables ////
int EV2_status = 0;
int EV3_status = 0;
int EV4_status = 0;
int EV5_status = 0;
int RA1_status = 0;
//// Conducibility sensors readouts ////
float SR1_value = 0;
float SR2_value = 0;
float SR3_value = 0;
//// Temperature sensors readouts ////
float ST2_temp = 0;
float ST3_temp = 0;
float ST4_temp = 0;
//// firmware version of the device and device id ////
#define SW_VERSION "0.1"
#define DEVICE_ID "geniale board 3"     
//// Other handy variables ////
volatile bool new_request = false;                        // flag that tells if a new request has arrived from the hub
volatile int received_msg_id = 0;                         // used for ack mechanism
volatile int received_msg_type = -1;                      // if 0 the device is sending its status
                                                          // if 1 the HUB wants to change the status of the device (with the values passed in the message)
                                                          // if 2 the device ACKs the HUB in response to a command
// defines for message type 
#define STATUS 0
#define SET_VALUES 1
#define ACK_HUB 2

// STATUS LED HANDLING
#define LED_CHANNEL 0
#define RESOLUTION 8
#define LED_PWM_FREQ 10
#define OFF 0
#define BLINK_5HZ 128
#define ON 255

//// PH sensor reading variable ////
float pH_value = 4.6;                                       // pH value read from the pH sensor (typical range: 4.5 <-> 9.5)
// Variables for voltages corresponding to pH values:
//1107.5 mV correspond to a pH of 4.5 and an ADC read of ...
//1392.5 mV correspond to a pH of 9.5 and an ADC read of ...
//pH = Mp * ADC + Qp
float Mp = 1;   //values still to be computed        
float Qp = 0;
#define pH_SAMPLES 10                                  // Number of samples taken to give a pH value
#define pH_INTERVAL 100                                // Interval of time in ms between two successive samples

// Variables for voltages corresponding to temperature ranges, for PT100:
//1190 mV correspond to 0 deg C and an ADC read of 1308
//1610 mV correspond to 100 deg C and an ADC read of 1823
//Temperature = M * ADC + Q
float M = 0.194174;        
float Q = -259.980583;
#define TEMP_SAMPLES 10                                // Number of samples taken to give a temperature value
#define TEMP_INTERVAL 100                              // Interval of time in ms between two successive samples

// Variables for voltages corresponding to conductiviy ranges, for ECDICPT/1:
//227 mV correspond to 5 mS and an ADC read of ...
//417 mV correspond to 2.5 mS and an ADC read of ...
//Conductivity = Mc * ADC + Qc
float Mc = 1;   //values still to be computed        
float Qc = 0;
#define COND_SAMPLES 10                                  // Number of samples taken to give a conductivity value
#define COND_INTERVAL 100                                // Interval of time in ms between two successive samples

////  MICROSOFT AZURE IOT DEFINITIONS   ////
static const char* connectionString = "HostName=geniale-iothub.azure-devices.net;DeviceId=00000001;SharedAccessKey=Cn4UylzZVDZD8UGzCTJazR3A9lRLnB+CbK6NkHxCIMk=";
static bool hasIoTHub = false;
static bool hasWifi = false;
#define INTERVAL 10000               // IoT message sending interval in ms
#define MESSAGE_MAX_LEN 256
RTC_DATA_ATTR int messageCount = 1;                // tells the number of the sent message
//static bool messageSending = true;
//static uint64_t send_interval_ms;

////  I/Os definitions    ////
#define FL2_GPIO   27       // Flow sensor FL2 connected to GPIO27
#define FL3_GPIO   26       // Flow sensor FL3 connected to GPIO26
#define EV2_GPIO   15       // Electrovalve EV2 connected to GPIO15
#define EV3_GPIO   13       // Electrovalve EV3 connected to GPIO13
#define EV4_GPIO   14       // Electrovalve EV4 connected to GPIO14
#define EV5_GPIO   12       // Electrovalve EV5 connected to GPIO12
#define RA1_GPIO   21       // Electrovalve RA1 connected to GPIO21
#define ST2_FORCE_GPIO 23   // Temperature sensor ST2 force pin
#define ST3_FORCE_GPIO 22   // Temperature sensor ST3 force pin
#define ST4_FORCE_GPIO 19   // Temperature sensor ST4 force pin
#define SR1_FORCE_GPIO 18   // Conductivity sensor SR1 force pin
#define SR2_FORCE_GPIO 17   // Conductivity sensor SR2 force pin
#define SR3_FORCE_GPIO 16   // Conductivity sensor SR3 force pin
#define ST2_MEASURE_GPIO 39 // Temperature sensor ST2 measure pin
#define ST3_MEASURE_GPIO 34 // Temperature sensor ST3 measure pin
#define ST4_MEASURE_GPIO 35 // Temperature sensor ST4 measure pin
#define SR1_MEASURE_GPIO 32 // Temperature sensor SR1 measure pin
#define SR2_MEASURE_GPIO 33 // Temperature sensor SR2 measure pin
#define SR3_MEASURE_GPIO 25 // Temperature sensor SR3 measure pin
#define SPH1_GPIO 36        // pH sensor SPH1 connected to VP 
#define LED   5             // Status led connected to GPIO5

static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
  if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
  {
    //Serial.println("Send Confirmation Callback finished.");
  }
}

static void MessageCallback(const char* payLoad, int size)
{
  ledcWrite(LED_CHANNEL, ON);
  Serial.println("Received message from HUB");
  if (size < 256) { 
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payLoad);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
    }
    else {  
    new_request = true;
    received_msg_id = doc["message_id"];
    received_msg_type = doc["message_type"];
      if(received_msg_type == SET_VALUES) {
          EV2_status = doc["EV2"];
          EV3_status = doc["EV3"];
          EV4_status = doc["EV4"];
          EV5_status = doc["EV5"];
          RA1_status = doc["RA1"];
      }
    }
  }
  else Serial.println("Cannot parse message, too long!");
}

/* NOT USED - DEVICE TWIN CALLBACK
static void DeviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payLoad, int size)
{
  char *temp = (char *)malloc(size + 1);
  if (temp == NULL)
  {
    return;
  }
  memcpy(temp, payLoad, size);
  temp[size] = '\0';
  // Display Twin message.
  Serial.println(temp);
  free(temp);
}
*/
/* NOT USED - DEVICE METHOD CALLBACK
  static int  DeviceMethodCallback(const char *methodName, const unsigned char *payload, int size, unsigned char **response, int *response_size)
  {
    LogInfo("Try to invoke method %s", methodName);
    const char *responseMessage = "\"Successfully invoke device method\"";
    int result = 200;

    if (strcmp(methodName, "start") == 0)
    {
      LogInfo("Start sending temperature and humidity data");
      messageSending = true;
    }
    else if (strcmp(methodName, "stop") == 0)
    {
      LogInfo("Stop sending temperature and humidity data");
      messageSending = false;
    }
    else
    {
      LogInfo("No method %s found", methodName);
      responseMessage = "\"No method found\"";
      result = 404;
    }

    *response_size = strlen(responseMessage) + 1;
    *response = (unsigned char *)strdup(responseMessage);

    return result;
  }
*/

//// PULSE COUNTER OVERFLOW ISR FOR FL2 ////
  void IRAM_ATTR CounterOverflow0(void *arg) {                  // Interrupt for overflow of pulse counter
    OverflowCounter0 = OverflowCounter0 + 1;                     // increase overflow counter
    PCNT.int_clr.val = BIT(PCNT0_FREQ_UNIT);                    // clear overflow flag
    pcnt_counter_clear(PCNT0_FREQ_UNIT);                        // zero and reset of pulse counter unit
  }

  //// PULSE COUNTER OVERFLOW ISR FOR FL3 ////
  void IRAM_ATTR CounterOverflow1(void *arg) {                  // Interrupt for overflow of pulse counter
    OverflowCounter1 = OverflowCounter1 + 1;                     // increase overflow counter
    PCNT.int_clr.val = BIT(PCNT1_FREQ_UNIT);                    // clear overflow flag
    pcnt_counter_clear(PCNT1_FREQ_UNIT);                        // zero and reset of pulse counter unit
  }

  void initPulseCounters (){                                    // initialise pulse counters
  //// INITIALIZE PULSE COUNTER FOR FL2 ////
    pcnt_config_t pcntFreqConfig = { };                        // Instance of pulse counter
    pcntFreqConfig.pulse_gpio_num = FL2_GPIO;                  // pin assignment for pulse counter
    pcntFreqConfig.pos_mode = PCNT_COUNT_INC;                  // count rising edges (=change from low to high logical level) as pulses
    pcntFreqConfig.neg_mode = PCNT_COUNT_DIS;                  // do nothing on falling edges
    pcntFreqConfig.counter_h_lim = PCNT_H_LIM_VAL;             // set upper limit of counting 
    pcntFreqConfig.unit = PCNT0_FREQ_UNIT;                      // select ESP32 pulse counter unit 0
    pcntFreqConfig.channel = PCNT_CHANNEL_0;                   // select channel 0 of pulse counter unit 0
    pcnt_unit_config(&pcntFreqConfig);                         // configure rigisters of the pulse counter
    pcnt_counter_pause(PCNT0_FREQ_UNIT);                        // pause pulse counter unit
    pcnt_counter_clear(PCNT0_FREQ_UNIT);                        // zero and reset of pulse counter unit
    pcnt_event_enable(PCNT0_FREQ_UNIT, PCNT_EVT_H_LIM);         // enable event for interrupt on reaching upper limit of counting
    pcnt_isr_register(CounterOverflow0, NULL, 0, &user_isr_handle0);  // configure register overflow interrupt handler
    pcnt_intr_enable(PCNT0_FREQ_UNIT);                          // enable overflow interrupt
    pcnt_set_filter_value(PCNT0_FREQ_UNIT, PCNT_FILTER_VAL);    // set damping, inertia 
    pcnt_filter_enable(PCNT0_FREQ_UNIT);                        // enable counter glitch filter (damping)
    pcnt_counter_resume(PCNT0_FREQ_UNIT);                       // resume counting on pulse counter unit

  //// INITIALIZE PULSE COUNTER FOR FL3 ////
    pcntFreqConfig = { };                                      // Instance of pulse counter
    pcntFreqConfig.pulse_gpio_num = FL3_GPIO;                  // pin assignment for pulse counter
    pcntFreqConfig.pos_mode = PCNT_COUNT_INC;                  // count rising edges (=change from low to high logical level) as pulses
    pcntFreqConfig.neg_mode = PCNT_COUNT_DIS;                  // do nothing on falling edges
    pcntFreqConfig.counter_h_lim = PCNT_H_LIM_VAL;             // set upper limit of counting 
    pcntFreqConfig.unit = PCNT1_FREQ_UNIT;                      // select ESP32 pulse counter unit 1
    pcntFreqConfig.channel = PCNT_CHANNEL_0;                   // select channel 0 of pulse counter unit 0
    pcnt_unit_config(&pcntFreqConfig);                         // configure rigisters of the pulse counter
    pcnt_counter_pause(PCNT1_FREQ_UNIT);                        // pause pulse counter unit
    pcnt_counter_clear(PCNT1_FREQ_UNIT);                        // zero and reset of pulse counter unit
    pcnt_event_enable(PCNT1_FREQ_UNIT, PCNT_EVT_H_LIM);         // enable event for interrupt on reaching upper limit of counting
    pcnt_isr_register(CounterOverflow1, NULL, 0, &user_isr_handle1);  // configure register overflow interrupt handler
    pcnt_intr_enable(PCNT1_FREQ_UNIT);                          // enable overflow interrupt
    pcnt_set_filter_value(PCNT1_FREQ_UNIT, PCNT_FILTER_VAL);    // set damping, inertia 
    pcnt_filter_enable(PCNT1_FREQ_UNIT);                        // enable counter glitch filter (damping)
    pcnt_counter_resume(PCNT1_FREQ_UNIT);                       // resume counting on pulse counter unit
  }
   
  void Reset_PCNT0() {                                          // function resetting counter 
    OverflowCounter0 = 0;                                       // set overflow counter to zero
    pcnt_counter_clear(PCNT0_FREQ_UNIT);                        // zero and reset of pulse counter unit
  }
  void Reset_PCNT1() {                                          // function resetting counter 
  OverflowCounter1 = 0;                                       // set overflow counter to zero
  pcnt_counter_clear(PCNT1_FREQ_UNIT);                        // zero and reset of pulse counter unit
  }
  int get_FL2_liters(){                                              // converts the pulses received from fl1 to liters
      pcnt_get_counter_value(PCNT0_FREQ_UNIT, &PulseCounter0);     // get pulse counter value - maximum value is 16 bit
      return ( OverflowCounter0*PCNT_H_LIM_VAL + PulseCounter0 ) / PULSES_PER_LITER;
  }
  int get_FL3_liters(){                                              // converts the pulses received from fl1 to liters
    pcnt_get_counter_value(PCNT1_FREQ_UNIT, &PulseCounter1);         // get pulse counter value - maximum value is 16 bit
    return ( OverflowCounter1*PCNT_H_LIM_VAL + PulseCounter1 ) / PULSES_PER_LITER;
}

void setup_with_wifi() {
  // configure status LED PWM functionalitites
  ledcSetup(LED_CHANNEL, LED_PWM_FREQ, RESOLUTION);
  ledcAttachPin(LED, LED_CHANNEL);                              // Attach PWM module to status LED
  ledcWrite(LED_CHANNEL, BLINK_5HZ);                            // LED initially blinks at 5Hz
  Serial.println("ESP32 Device");
  Serial.println("Reading sensor values...");
  // Sample sensors before enabling wifi (ADC on pin 25 does not work with wifi on)
  ST2_temp = read_temperature(ST2_FORCE_GPIO, ST2_MEASURE_GPIO);
  ST3_temp = read_temperature(ST3_FORCE_GPIO, ST3_MEASURE_GPIO);
  ST4_temp = read_temperature(ST4_FORCE_GPIO, ST4_MEASURE_GPIO);
  SR1_value = read_conductivity(SR1_FORCE_GPIO, SR1_MEASURE_GPIO);
  SR2_value = read_conductivity(SR2_FORCE_GPIO, SR2_MEASURE_GPIO);
  SR3_value = read_conductivity(SR3_FORCE_GPIO, SR3_MEASURE_GPIO);
  pH_value = read_pH(SPH1_GPIO);
  Serial.println("Starting WiFi connection...");
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  WiFiManager wm;
  //wm.resetSettings();  // reset settings - wipe stored credentials for testing
  bool res;
  res = wm.autoConnect("GENIALE brd3 setup"); // Generates a pwd-free ap for the user to connect and tell Wi-Fi credentials
  //res = wm.autoConnect("AutoConnectAP","password"); // Generates a pwd-protected ap for the user to connect and tell Wi-Fi credentials
  if(!res) {
      Serial.println("Failed to connect to wifi");
      delay(10000);
      ESP.restart();
  } 
  else {
      //if you get here you have connected to the WiFi    
      Serial.println("Connected to wifi!");
      ledcWrite(LED_CHANNEL, ON);
      // Wait for ezTime to get its time synchronized
	    waitForSync();
      Serial.println("UTC Time in ISO8601: " + UTC.dateTime(ISO8601));
      hasWifi = true;
    }
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println(" > IoT Hub");
  if (!Esp32MQTTClient_Init((const uint8_t*)connectionString, true))
  {
    hasIoTHub = false;
    Serial.println("Initializing IoT hub failed.");
    return;
  }
  hasIoTHub = true;
  Esp32MQTTClient_SetSendConfirmationCallback(SendConfirmationCallback);
  Esp32MQTTClient_SetMessageCallback(MessageCallback);
  //Esp32MQTTClient_SetDeviceTwinCallback(DeviceTwinCallback);
  //Esp32MQTTClient_SetDeviceMethodCallback(DeviceMethodCallback);
  
  randomSeed(analogRead(0));
  //send_interval_ms = millis();
  ledcWrite(LED_CHANNEL, OFF);
  Serial.println("Sending status message to HUB...");
  send_message(STATUS, messageCount++);
  Serial.println("Waiting for a message from HUB and going to sleep...");
}

void setup_just_to_update_flux() {
  int old_FL2_liters = 0;
  int old_FL3_liters = 0;
  int current_FL2_liters;
  int current_FL3_liters;
  while(true){
    delay(8000);
    current_FL2_liters = get_FL2_liters();
    current_FL3_liters = get_FL3_liters();
    if(current_FL2_liters>old_FL2_liters || current_FL3_liters>old_FL3_liters) {
      Serial.println("FL2: "+ String(current_FL2_liters));
      Serial.println("FL3: "+ String(current_FL3_liters));
    }
    else {
      Serial.println("Water not flushed anymore, going back to sleep");
      FL2_liters += current_FL2_liters;
      FL3_liters += current_FL3_liters;
      esp_sleep_enable_ext1_wakeup((1 << (int)FL2_GPIO) | (1 << (int)FL3_GPIO), ESP_EXT1_WAKEUP_ANY_HIGH);                
      esp_deep_sleep_start();
    } 
    old_FL2_liters = current_FL2_liters;
    old_FL3_liters = current_FL3_liters;
  }
}

float read_temperature(int GPIO_FORCE, int GPIO_MEASURE) {
  digitalWrite(GPIO_FORCE, HIGH);
  float mean = 0;
  float val, temperaturec;
    // acquire TEMP_SAMPLES samples and compute mean
    for(int i = 0; i < TEMP_SAMPLES; i++)  {
        val = analogRead(GPIO_MEASURE);
        temperaturec = M * val + Q;
        mean += temperaturec;
        delay(TEMP_INTERVAL); 
      }
  digitalWrite(GPIO_FORCE, LOW);
  //Serial.println(String("Temperature in deg C: ") + String(mean/TEMP_SAMPLES, 2));
  return roundf(mean/(TEMP_SAMPLES/10)) / 10;   //return the temperature with a single decimal place
}

float read_conductivity(int GPIO_FORCE, int GPIO_MEASURE) {
  digitalWrite(GPIO_FORCE, HIGH);
  float mean = 0;
  float val, conductivity;
    // acquire COND_SAMPLES samples and compute mean
    for(int i = 0; i < COND_SAMPLES; i++)  {
        val = analogRead(GPIO_MEASURE);
        conductivity = Mc * val + Qc;
        mean += conductivity;
        delay(COND_INTERVAL); 
      }
  Serial.println(String("Raw cond val: ") + String(mean/COND_SAMPLES) + String(" On GPIO: ") + String(GPIO_MEASURE));
  digitalWrite(GPIO_FORCE, LOW);
  //Serial.println(String("Conductivity in milliSiemens: ") + String(mean/COND_SAMPLES, 2));
  return roundf(mean/(COND_SAMPLES/10)) / 10;   //return the conductivity with a single decimal place
}

float read_pH(int GPIO_MEASURE) {
  float mean = 0;
  float val, pH;
    // acquire pH_SAMPLES samples and compute mean
    for(int i = 0; i < pH_SAMPLES; i++)  {
        val = analogRead(GPIO_MEASURE);
        pH = Mp * val + Qp;
        mean += pH;
        delay(pH_INTERVAL); 
      }
  Serial.println(String("Raw ph val ") + String(mean/pH_SAMPLES) + String(" On GPIO: ") + String(GPIO_MEASURE));    
  //Serial.println(String("pH value: ") + String(mean/pH_SAMPLES, 2));
  return roundf(mean/(pH_SAMPLES/10)) / 10;   //return the pH with a single decimal place
}

void setup() {
  pinMode(FL2_GPIO, INPUT);                            // the output of the FL2 flow sensor is open collector (MUST USE EXTERNAL PULL UP!!)
  pinMode(FL3_GPIO, INPUT);                            // the output of the FL3 flow sensor is open collector (MUST USE EXTERNAL PULL UP!!)
  initPulseCounters();
  pinMode(EV2_GPIO, OUTPUT);
  pinMode(EV3_GPIO, OUTPUT);     
  pinMode(EV4_GPIO, OUTPUT);
  pinMode(EV5_GPIO, OUTPUT);
  pinMode(RA1_GPIO, OUTPUT);
  pinMode(ST2_FORCE_GPIO, OUTPUT);
  pinMode(ST3_FORCE_GPIO, OUTPUT);
  pinMode(ST4_FORCE_GPIO, OUTPUT);
  pinMode(SR1_FORCE_GPIO, OUTPUT);
  pinMode(SR2_FORCE_GPIO, OUTPUT);
  pinMode(SR3_FORCE_GPIO, OUTPUT);
  pinMode(ST2_MEASURE_GPIO, INPUT);
  pinMode(ST3_MEASURE_GPIO, INPUT);
  pinMode(ST4_MEASURE_GPIO, INPUT);
  pinMode(SR1_MEASURE_GPIO, INPUT);
  pinMode(SR2_MEASURE_GPIO, INPUT);
  pinMode(SR3_MEASURE_GPIO, INPUT);
  pinMode(SPH1_GPIO, INPUT);
  digitalWrite(EV2_GPIO, LOW);                                  // All electrovalves are initially off
  digitalWrite(EV3_GPIO, LOW);
  digitalWrite(EV4_GPIO, LOW);
  digitalWrite(EV5_GPIO, LOW);
  digitalWrite(RA1_GPIO, LOW);
  digitalWrite(ST2_FORCE_GPIO, LOW);                            // All force pins initially off
  digitalWrite(ST3_FORCE_GPIO, LOW);
  digitalWrite(ST4_FORCE_GPIO, LOW);
  digitalWrite(SR1_FORCE_GPIO, LOW);
  digitalWrite(SR2_FORCE_GPIO, LOW);
  digitalWrite(SR3_FORCE_GPIO, LOW);

  Serial.begin(115200);
  delay(500);                                                   //Take some time to open up the Serial Monitor
  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  // PERFORM ACTIONS DEPENDING ON WAKEUP REASON //
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); 
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT1 : 
    Serial.println("Wakeup caused by flux sensor");     
    setup_just_to_update_flux();
    break;
    case ESP_SLEEP_WAKEUP_TIMER : 
    Serial.println("Wakeup caused by timer"); 
    setup_with_wifi();
    break;
    default:
    Serial.println("It's the first boot");
    setup_with_wifi();
    break;
  }
}

void send_message(int reply_type, int msgid) {
if (hasWifi && hasIoTHub)
  {
      StaticJsonDocument<512> msgtosend;            // pre-allocate 512 bytes of memory for the json message
      msgtosend["message_id"] = msgid;
      msgtosend["timestamp"] = UTC.dateTime(ISO8601);
      msgtosend["message_type"] = reply_type;
      msgtosend["device_id"] = DEVICE_ID;
      msgtosend["iot_module_software_version"] = SW_VERSION;
      msgtosend["SR1"] = SR1_value;
      msgtosend["SR2"] = SR2_value;
      msgtosend["SR3"] = SR3_value;
      msgtosend["ST2"] = ST2_temp;
      msgtosend["ST3"] = ST3_temp;
      msgtosend["ST4"] = ST4_temp;
      msgtosend["SPH1"] = pH_value;
      msgtosend["FL2"] = FL2_liters;
      msgtosend["FL3"] = FL3_liters;      
      msgtosend["EV2"] = EV2_status;
      msgtosend["EV3"] = EV3_status;                   
      msgtosend["EV4"] = EV4_status;   
      msgtosend["EV5"] = EV5_status;   
      msgtosend["RA1"] = RA1_status;   

      char out[512];
      int msgsize =serializeJson(msgtosend, out);
      //Serial.println(msgsize);
      Serial.println("Sending message to HUB:");
      Serial.println(out);
      EVENT_INSTANCE* message = Esp32MQTTClient_Event_Generate(out, MESSAGE);
      Esp32MQTTClient_SendEventInstance(message);
      ledcWrite(LED_CHANNEL, OFF);
  }
}


void loop() {
  Esp32MQTTClient_Check();
  // if a request has arrived from the hub, process it and send a reply
  if(new_request == true){
    new_request = false;
    switch (received_msg_type)  {
      case SET_VALUES: 
        digitalWrite(EV2_GPIO, EV2_status);
        digitalWrite(EV3_GPIO, EV3_status);                   
        digitalWrite(EV4_GPIO, EV4_status);   
        digitalWrite(EV5_GPIO, EV5_status);   
        digitalWrite(RA1_GPIO, RA1_status);
        send_message(ACK_HUB, received_msg_id);
        break;
      case STATUS:
        send_message(STATUS, received_msg_id);
        break;
      default:
        Serial.println("Invalid message type!");
        ledcWrite(LED_CHANNEL, OFF);
        break;
    }
      Serial.println("Going into deep sleep");
      esp_sleep_enable_ext1_wakeup((1 << (int)FL2_GPIO) | (1 << (int)FL3_GPIO), ESP_EXT1_WAKEUP_ANY_HIGH);         
      esp_deep_sleep_start();
  }
}