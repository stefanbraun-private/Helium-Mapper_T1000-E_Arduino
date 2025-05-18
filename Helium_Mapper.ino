////////////////////////////////////////////////////////////////////////////////
// Helium Mapper
//
// Experimental Helium IoT coverage mapping device,
// coded in Arduino running on Seeed Studio T1000-E LoRaWAN GPS tracker.
//////////////////////////////////////////////////////////////////////////////// 
// 
// Usage:      This software turns a T1000-E into a custom OpenSource Helium mapping device.
//             It sends the current GPS location via Helium LoRaWAN to community projects like
//             https://docs.helium.com/iot/coverage-mapping/
//             https://www.coveragemap.net/2024/02/28/mapping-with-chirpstack/
//             https://mappers.hexato.io/docs
//
// Remark:     The LoRaWAN uplink payload decoder works only on ChirpStack-based
//             LNS (LoRaWAN Network Server),
//             and at the moment not all community projects can handle those uplink metadata...
//
//
// Disclaimer: This code is based on Seeed Studio's Arduino example code
//             https://wiki.seeedstudio.com/t1000_e_arduino_examples/
//             and contains code snippets from many sources...
//             It's an experiment for proof-of-concept,
//             as a starting point for other developers :-)
//
//
// Stefan Braun, May 18th 2025
////////////////////////////////////////////////////////////////////////////////
// 
//
// LoRaWAN uplinks are encoded with CayenneLPP
//         =>it's possible to decode LoRaWAN Uplink by using this payload decoder,
//           and manually expose the decoded values into ChirpStack object
//           https://github.com/RAKWireless/RAKwireless_Standardized_Payload/blob/main/RAKwireless_Standardized_Payload.js
//
////////////////////////////////////////////////////////////////////////////////
//
// FIXME: I'm using an Cayenne LPP Encoder from TTN https://github.com/TheThingsNetwork/arduino-device-lib
//        and added a few missing datatypes...
//        Arduino library "CayenneLPP" v.1.4.0 by Electronic Cats didn't compile...
//        it contains a huge dependency to "ArduinoJson" by Benoit Blanchon
//        compilation error was "expected unqualified-id before string constant"...
//
// FIXME: at the moment it's not possible to shutdown GPS tracker... it runs and runs...
//
// FIXME: at the moment GPS tracker sends LoRaWAN uplinks with ADR... I don't know how send with fixed Datarate...
//        =>perhaps this is done by narrowing ADR custom channel list? "adr_custom_list_eu868_default"
//        possible solution: uint8_t adr_custom_data[16] = { 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02 };
//        according to https://github.com/Lora-net/SWSD001/blob/bb1f258c5c5188540f2fc2774f9d66233c4c6d24/lora_basics_modem/lora_basics_modem/smtc_modem_core/modem_services/lorawan_certification.c#L233
//        and https://github.com/Lora-net/SWL2001/issues/52#issuecomment-2059559227
//
//
// Further information:
// -base of current LoRaWAN payload decoder: https://github.com/RAKWireless/RAKwireless_Standardized_Payload/blob/main/RAKwireless_Standardized_Payload.js
// -library for "Semtech LR1110": https://github.com/Lora-net/SWL2001/blob/master/lbm_lib/README.md
//  WiFi and BLE scanning: https://github.com/Lora-net/SWL2001/blob/master/lbm_lib/smtc_modem_core/geolocation_services/README.md
// -in detail library manual: https://www.mouser.com/pdfDocs/lorabasicsmodemusermanual.pdf
//
////////////////////////////////////////////////////////////////////////////////

#include "LoRaWAN_OTAA_Keys.h"

#include <Tracker_T1000_E_LoRaWAN_Examples.h>

// "Protothreads": cooperative multithreading on Arduino
// with a Python-Generator-like handling
// =>ATTENTION: no switch-statement, only static local variables!
// (the C-preprocessor builds a huge switch-statement over all PT_THREADs)
#include "protothreads.h"

// LoRaWAN chip framework
#include <LbmT1000E.hpp>
#include <Lbmx.hpp>


// comment this out if tracker should apply Adaptive Data Rate
// (for Helium Mapping it's recommended to disable ADR, for having constant signal conditions)
#define LORAWAN_DISABLE_ADR

#ifdef LORAWAN_DISABLE_ADR
// T1000-E sends with fixed datarate (and disabled ADR), if minimum and maximum DR has the same value
static const uint8_t adr_custom_list_eu868_default[16] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}; // SF10,SF10,SF10,SF10,SF10,SF10,SF10,SF10,SF10,SF10,SF10,SF10,SF10,SF10,SF10,SF10
#else
static const uint8_t adr_custom_list_eu868_default[16] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5}; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7
#endif

// FIXME: implement disabled ADR in all frequencies, and someone has to test these frequency bands!
static const uint8_t adr_custom_list_us915_default[16] = {1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3}; // SF9,SF9,SF9,SF9,SF9,SF8,SF8,SF8,SF8,SF8,SF7,SF7,SF7,SF7,SF7
static const uint8_t adr_custom_list_au915_default[16] = {3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5}; // SF9,SF9,SF9,SF9,SF9,SF8,SF8,SF8,SF8,SF8,SF7,SF7,SF7,SF7,SF7
static const uint8_t adr_custom_list_as923_default[16] = {3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5}; // SF9,SF9,SF9,SF9,SF9,SF8,SF8,SF8,SF8,SF8,SF7,SF7,SF7,SF7,SF7
static const uint8_t adr_custom_list_kr920_default[16] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5}; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7
static const uint8_t adr_custom_list_in865_default[16] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5}; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7
static const uint8_t adr_custom_list_ru864_default[16] = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 5}; // SF12,SF12,SF12,SF11,SF11,SF11,SF10,SF10,SF10,SF9,SF9,SF9,SF8,SF8,SF7,SF7


enum class LorawanStateType
{
    Startup,
    Joining,
    Joined,
    Failed,
};

#define UPLINK_MINIMUM_INTERVAL 30000   // [ms]
static constexpr smtc_modem_region_t REGION = SMTC_MODEM_REGION_EU_868;
static constexpr uint8_t LORAWAN_UPLINK_FPORT = 2;
static constexpr bool LORAWAN_CONFIRMED_MSG_ON = false;
static constexpr uint32_t LORAWAN_APP_DATA_MAX_SIZE = 242; // [bytes]

static LbmT1000E &lbmT1000E = LbmT1000E::getInstance();
static LorawanStateType lorawanState = LorawanStateType::Startup;
static uint8_t lorawan_data_buffer[LORAWAN_APP_DATA_MAX_SIZE];

// WiFi SSID scanning
static constexpr uint32_t WIFI_AP_RSSI_SIZE = 1;         // [bytes]
static constexpr uint32_t WIFI_AP_ADDRESS_SIZE = 6;      // [bytes]

static wifi_scan_all_result_t wifi_results = {0};




// parsing NMEA data streams provided by GPS modules
#include "TinyGPS++.h"
TinyGPSPlus gps;

// Cayenne LPP encoding of LoRaWAN Uplinks
// example usage: https://github.com/TheThingsNetwork/arduino-device-lib/blob/master/examples/CayenneLPP/CayenneLPP.ino
#include "CayenneLPP.h"
CayenneLPP lpp(51);


// handling of T1000-E button: based on example "ClickVersusDoubleClickUsingBoth.ino"
#include <AceButton.h>
using namespace ace_button;

#define BUTTON_LONG_PRESS_DURATION 3000

// buzzer melody selection
#define MELODY_NONE 0
#define MELODY_NOKIA_RINGTONE 1
#define MELODY_STARWARS_THEME 2


////////////////////////////////////////////////////////////////////////////////
// global variables

// LoRaWAN uplinks
static double location_lat = 0.0;
static double location_lng = 0.0;
static double location_alt = 0.0;

static uint8_t battery_percent = 0;


// communication between "Threads"
static int buzzer_melody_choice = MELODY_NONE;
static bool buzzer_melody_request = false;

static bool gnss_keep_scanning = true;

static bool lorawan_uplink_done = false;

////////////////////////////////////////////////////////////////////////////////

AceButton button;

// Forward reference to prevent Arduino compiler becoming confused.
void handleButtonEvent(AceButton*, uint8_t, uint8_t);


////////////////////////////////////////////////////////////////////////////////
// mainloop time analysis
static bool main_loop_analysis_enabled = false;
static unsigned long loop_counter = 1;
static unsigned long loop_micros_curr = 0;
static unsigned long loop_micros_last = micros();
static unsigned int loop_micros_delta = 0;
static double loop_micros_avg = 0;
#define NOF_MAIN_LOOPS 1000000
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// Buzzer: playing melodies with music notes
// =>special thanks to Robson Couto for his cool code!
// copied from https://github.com/robsoncouto/arduino-songs 

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

pt ptBuzzer;
int buzzerThread(struct pt* pt) {

    // which melody is getting played?
    // (local copy of global variable, not allowing changing during melody play by other threads)
    static int curr_melody_choice = MELODY_NONE;

    // change this to make the song slower or faster
    static int tempo = 180;

    // melody array:
    // notes of the melody followed by the duration.
    // a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
    // !!negative numbers are used to represent dotted notes,
    // so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!

    // there are two values per note (pitch and duration)
    static int *melodyPointer;
    static int nof_notes = 0;
    static int thisNote = 0;

    // this calculates the duration of a whole note in ms
    static int wholenote = 0;

    static int divider = 0;
    static int noteDuration = 0;

    PT_BEGIN(pt);

    // Loop forever
    while(true) {
        PT_WAIT_UNTIL(pt, buzzer_melody_request and (buzzer_melody_choice != MELODY_NONE));
        curr_melody_choice = buzzer_melody_choice;
        nof_notes = 0;
        
        if (curr_melody_choice == MELODY_NOKIA_RINGTONE) {
            Serial.println("Playing Nokia Ringtone on buzzer!");

            static int melody[] = {
                // Nokia Ringtone 
                // Score available at https://musescore.com/user/29944637/scores/5266155
                // source code from https://github.com/robsoncouto/arduino-songs/blob/master/nokia/nokia.ino
                
                NOTE_E5, 8, NOTE_D5, 8, NOTE_FS4, 4, NOTE_GS4, 4, 
                NOTE_CS5, 8, NOTE_B4, 8, NOTE_D4, 4, NOTE_E4, 4, 
                NOTE_B4, 8, NOTE_A4, 8, NOTE_CS4, 4, NOTE_E4, 4,
                NOTE_A4, 2, 
            };
            melodyPointer = melody;

            nof_notes = 13;
            tempo = 180;

        } else if (curr_melody_choice == MELODY_STARWARS_THEME) {
            Serial.println("Playing Star Wars Theme on buzzer!");

            static int melody[] = {
                // Star Wars Main Theme 
                // source code from https://github.com/robsoncouto/arduino-songs/blob/master/starwars/starwars.ino
                
                NOTE_AS4,8, NOTE_AS4,8, NOTE_AS4,8, //1
                NOTE_F5,2, NOTE_C6,2,
                NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F6,2, NOTE_C6,4,  
                NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F6,2, NOTE_C6,4,  
                NOTE_AS5,8, NOTE_A5,8, NOTE_AS5,8, NOTE_G5,2, NOTE_C5,8, NOTE_C5,8, NOTE_C5,8,
                NOTE_F5,2, NOTE_C6,2,
                NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F6,2, NOTE_C6,4,  
                
                NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F6,2, NOTE_C6,4, //8  
                NOTE_AS5,8, NOTE_A5,8, NOTE_AS5,8, NOTE_G5,2, NOTE_C5,-8, NOTE_C5,16, 
                NOTE_D5,-4, NOTE_D5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F5,8,
                NOTE_F5,8, NOTE_G5,8, NOTE_A5,8, NOTE_G5,4, NOTE_D5,8, NOTE_E5,4,NOTE_C5,-8, NOTE_C5,16,
                NOTE_D5,-4, NOTE_D5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F5,8,
                
                NOTE_C6,-8, NOTE_G5,16, NOTE_G5,2, REST,8, NOTE_C5,8, //13
                NOTE_D5,-4, NOTE_D5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F5,8,
                NOTE_F5,8, NOTE_G5,8, NOTE_A5,8, NOTE_G5,4, NOTE_D5,8, NOTE_E5,4,NOTE_C6,-8, NOTE_C6,16,
                NOTE_F6,4, NOTE_DS6,8, NOTE_CS6,4, NOTE_C6,8, NOTE_AS5,4, NOTE_GS5,8, NOTE_G5,4, NOTE_F5,8,
                NOTE_C6,1, 
            };
            melodyPointer = melody;

            nof_notes = 87;
            tempo = 108;
        }

        if (nof_notes > 0) {
            // playing current melody

            digitalWrite(PIN_BUZZER_EN, HIGH);
            wholenote = (60000 * 4) / tempo;

            // iterate over the notes of the melody. 
            // Remember, the array is twice the number of notes (notes + durations)
            thisNote = 0;
            while(buzzer_melody_request and (thisNote < nof_notes * 2)){

                // calculates the duration of each note
                divider = melodyPointer[thisNote + 1];
                if (divider > 0) {
                    // regular note, just proceed
                    noteDuration = (wholenote) / divider;
                } else if (divider < 0) {
                    // dotted notes are represented with negative durations!!
                    noteDuration = (wholenote) / abs(divider);
                    noteDuration *= 1.5; // increases the duration in half for dotted notes
                }

                // we only play the note for 90% of the duration, leaving 10% as a pause
                tone(PIN_BUZZER_PWM, melodyPointer[thisNote], noteDuration*0.9);

                // Wait for the specief duration before playing the next note.
                PT_SLEEP(pt, noteDuration);
                
                // stop the waveform generation before the next note.
                noTone(PIN_BUZZER_PWM);

                thisNote = thisNote + 2;
            }
        }
        // reset melody play request
        digitalWrite(PIN_BUZZER_EN, LOW);
        buzzer_melody_request = false;
    }

    PT_END(pt);
}
////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////
// sensor reading: battery state

#define BATTERY_READING_INTERVAL 300000

pt ptBattery;
int batteryThread(struct pt* pt) {

    PT_BEGIN(pt);

    // Loop forever
    while(1) {
        // no need to manually powering and shutdown of sensor: it's done in library
        battery_percent = (uint8_t)sensor_bat_sample();

        Serial.printf("Battery level: %d%%\n", battery_percent);

        PT_SLEEP(pt, BATTERY_READING_INTERVAL);
    }

    PT_END(pt);
}
////////////////////////////////////////////////////////////////////////////////







////////////////////////////////////////////////////////////////////////////////
// getting GPS location
//
// with help from Seeed Studio T1000-E example sketch
// and code from their library file "ag3335.cpp"
// and Meshtastic https://github.com/meshtastic/firmware/blob/master/src/gps/GPS.cpp

// state machine
#define GNSS_STATE_OFF 0
#define GNSS_STATE_INIT 1
#define GNSS_STATE_RUNNING 2
#define GNSS_STATE_DEINIT 3
static int gnss_state = GNSS_STATE_OFF;


// copied from "ag3335.cpp"
static uint8_t app_nmea_check_sum(const char *buf)
{
    uint8_t i = 0;
    uint8_t chk = 0;
    uint8_t len = strlen(buf);

    for (chk = buf[1], i = 2; i < len; i++)
    {
        chk ^= buf[i];
    }

    return chk;
}

static void gnss_send_nmea_string(const char *command)
{
    // FIXME: after debugging set to "Serial2"
    // NMEA format: <ASCII command> <1 byte checksum> <\r\n>
    Serial2.printf("%s%02X\r\n", command, app_nmea_check_sum(command));
}


pt ptGnss;
int gnssThread(struct pt* pt) {

    PT_BEGIN(pt);

    // Loop forever
    while (1) {
        if (gnss_state == GNSS_STATE_OFF){
            // GNSS chip is powered off for battery saving

            // here it's possible to implement wakeup condition
            
            gnss_state = GNSS_STATE_INIT;
            PT_YIELD(pt);

        } else if (gnss_state == GNSS_STATE_INIT){
            // initialize communication to GNSS chip
            Serial2.begin(115200);



            // powering chip
            digitalWrite(PIN_GNSS_VRTC_EN, HIGH);
            PT_SLEEP(pt, 500)
            digitalWrite(PIN_GNSS_POWER_EN, HIGH);
            PT_SLEEP(pt, 1000)

            // reset
            digitalWrite(PIN_GNSS_RESET, HIGH);
            PT_SLEEP(pt, 10)
            digitalWrite(PIN_GNSS_RESET, LOW);
            PT_SLEEP(pt, 10)
            



            // perhaps a reset for RealTimeClock?
            digitalWrite(PIN_GNSS_RTC_INT, HIGH);
            PT_SLEEP(pt, 3)
            digitalWrite(PIN_GNSS_RTC_INT, LOW);
            PT_SLEEP(pt, 50)


            // sending multiple NMEA commands to GNSS chip
            // (FIXME: Are these in right order? Are these the right ones? Is somewhere documentation for this chip?)
            //
            // "gnss_scan_lock_sleep"
            gnss_send_nmea_string("$PAIR382,1");

            // Enable GPS+GALILEO+NAVIC
            gnss_send_nmea_string("$PAIR066,1,0,1,0,0,1");

            // Configure NMEA (sentences will output once per fix)
            gnss_send_nmea_string("$PAIR062,0,1"); // GGA ON
            gnss_send_nmea_string("$PAIR062,1,0"); // GLL OFF
            gnss_send_nmea_string("$PAIR062,2,0"); // GSA OFF
            gnss_send_nmea_string("$PAIR062,3,0"); // GSV OFF
            gnss_send_nmea_string("$PAIR062,4,1"); // RMC ON
            gnss_send_nmea_string("$PAIR062,5,0"); // VTG OFF
            gnss_send_nmea_string("$PAIR062,6,0"); // ZDA ON
            PT_SLEEP(pt, 250);

            // save configuration
            gnss_send_nmea_string("$PAIR513");

            gnss_state = GNSS_STATE_RUNNING;
            PT_YIELD(pt);

        } else if (gnss_state == GNSS_STATE_RUNNING){
            // extracting useful information from GNSS NMEA strings

            if (gnss_keep_scanning) {
                PT_YIELD_UNTIL(pt, Serial2.available() > 0);
                // feed NMEA into parser object
                gps.encode(Serial2.read());
            } else {
                // change state
                gnss_state = GNSS_STATE_DEINIT;
            }
            
            PT_YIELD(pt);

        } else if (gnss_state == GNSS_STATE_DEINIT){
            // stop GNSS handling, power off chip

            // sending multiple NMEA commands to GNSS chip
            // (FIXME: Are these in right order? Are these the right ones? Is somewhere documentation for this chip?)
            //
            // "unlock_sleep"
            gnss_send_nmea_string("$PAIR382,0");

            // "enter_rtc_mode"
            gnss_send_nmea_string("$PAIR650,0");
            PT_SLEEP(pt, 50);
            
            // shutdown main power (I assume RTC is still powered)
            digitalWrite(PIN_GNSS_POWER_EN, LOW);
            PT_SLEEP(pt, 50);

            // FIXME: ist this needed? Or is it better to have possibility for GPS warmstart?
            //digitalWrite(PIN_GNSS_VRTC_EN, LOW);

            Serial2.end();

            gnss_state = GNSS_STATE_OFF;
            PT_YIELD(pt);
        }
    }

    PT_END(pt);
}
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// analyzing GPS data
// (based on "FullExample" of TinyGPSPlus library)


pt ptGpsAnalyzer;
int gpsAnalyzerThread(struct pt* pt) {

    static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

    PT_BEGIN(pt);

    // give GNSS chip some start time...
    PT_SLEEP(pt, 15000);

    // no further handling if there's no serial debug attached (CDCACM USB serial port)
    // with help from https://forum.arduino.cc/t/led-on-if-serial-disconnected/216477/2
    PT_YIELD_UNTIL(pt, Serial);

    Serial.println("analyzing GPS data - based on FullExample.ino");
    Serial.println("An extensive example of many interesting TinyGPSPlus features");
    Serial.print("Testing TinyGPSPlus library v. "); Serial.println(TinyGPSPlus::libraryVersion());
    Serial.println("by Mikal Hart");
    Serial.println();
    Serial.println("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum");
    Serial.println("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail");
    Serial.println("----------------------------------------------------------------------------------------------------------------------------------------");


    // Loop forever
    while(true) {
        // printing one row

        // no further handling if there's no serial debug attached (CDCACM USB serial port)
        // with help from https://forum.arduino.cc/t/led-on-if-serial-disconnected/216477/2
        PT_YIELD_UNTIL(pt, Serial);


        printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
        printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
        printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
        printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
        printInt(gps.location.age(), gps.location.isValid(), 5);
        printDateTime(gps.date, gps.time);
        printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
        printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
        printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
        printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

        unsigned long distanceKmToLondon =
            (unsigned long)TinyGPSPlus::distanceBetween(
            gps.location.lat(),
            gps.location.lng(),
            LONDON_LAT, 
            LONDON_LON) / 1000;
        printInt(distanceKmToLondon, gps.location.isValid(), 9);

        double courseToLondon =
            TinyGPSPlus::courseTo(
            gps.location.lat(),
            gps.location.lng(),
            LONDON_LAT, 
            LONDON_LON);

        printFloat(courseToLondon, gps.location.isValid(), 7, 2);

        const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

        printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

        printInt(gps.charsProcessed(), true, 6);
        printInt(gps.sentencesWithFix(), true, 10);
        printInt(gps.failedChecksum(), true, 9);
        Serial.println();

        PT_SLEEP(pt, 60000);
    }

    PT_END(pt);
}


// helper functions
static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
}


////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// update location: query GPS, update location
//
// with help from https://arduiniana.org/libraries/tinygpsplus/
// and https://github.com/mikalhart/TinyGPSPlus/blob/master/examples/FullExample/FullExample.ino

#define LOCATION_UPDATE_INTERVAL 5000

// threshold [ms] for saying GPS fix is lost
#define GNSS_LOCATION_MAX_AGE 1500

pt ptLocation;
int locationThread(struct pt* pt) {

    PT_BEGIN(pt);

    // Loop forever
    while(true) {
        // wait for new information from GPS object
        PT_YIELD_UNTIL(pt, gps.location.isUpdated());
        if (gps.location.isValid() and (gps.location.age() < GNSS_LOCATION_MAX_AGE)) {
            // got fresh GPS location
            location_lat = gps.location.lat();
            location_lng = gps.location.lng();
            location_alt = gps.altitude.meters();
        } else {
            // set location to "Null Island" ;-)
            location_lat = 0.0;
            location_lng = 0.0;
            location_alt = 0.0;
        }

        PT_SLEEP(pt, LOCATION_UPDATE_INTERVAL);
    }

    PT_END(pt);
}
////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////////////
// LED blinker: showing "is this thing on?"

pt ptBlink;
int blinkThread(struct pt* pt) {
    PT_BEGIN(pt);

    // Loop forever
    while(1) {
        digitalWrite(LED_GREEN, HIGH);
        PT_SLEEP(pt, 100);
        digitalWrite(LED_GREEN, LOW);
        PT_SLEEP(pt, 5000);
    }

    PT_END(pt);
}
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// LoRaWAN event engine

pt ptLorawan;
int lorawanThread(struct pt* pt) {

    #define LORAWAN_MAXIMUM_EXECUTION_INTERVAL 1000
    static uint32_t sleepTime = 0;

    PT_BEGIN(pt);

    // Loop forever
    while(true) {
        sleepTime = LbmxEngine::doWork();
        PT_SLEEP(pt, min(sleepTime, LORAWAN_MAXIMUM_EXECUTION_INTERVAL));
    }

    PT_END(pt);
}
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// MyLbmxEventHandlers

class MyLbmxEventHandlers : public LbmxEventHandlers
{
protected:
    void reset(const LbmxEvent &event) override;
    void joined(const LbmxEvent &event) override;
    void joinFail(const LbmxEvent &event) override;
    void alarm(const LbmxEvent &event) override;
    void txDone(const LbmxEvent &event) override;
    void downData(const LbmxEvent &event) override;
    void wifiScanDone(const LbmxEvent &event) override;
};

void MyLbmxEventHandlers::reset(const LbmxEvent &event)
{
    if (LbmxEngine::setRegion(REGION) != SMTC_MODEM_RC_OK)
        abort();
    if (LbmxEngine::setOTAA(DEV_EUI, JOIN_EUI, APP_KEY) != SMTC_MODEM_RC_OK)
        abort();

    Serial.println("Joining the LoRaWAN network...");
    if (LbmxEngine::joinNetwork() != SMTC_MODEM_RC_OK)
        abort();

    if((REGION == SMTC_MODEM_REGION_EU_868) || (REGION == SMTC_MODEM_REGION_RU_864))
    {
        smtc_modem_set_region_duty_cycle( true );
    }

    lorawanState = LorawanStateType::Joining;
}

void MyLbmxEventHandlers::joined(const LbmxEvent &event)
{
    lorawanState = LorawanStateType::Joined;

    // Enable ADR
    uint8_t adr_custom_list_region[16] = {0};
    smtc_modem_region_t region;
    smtc_modem_get_region(0, &region);
    switch (region)
    {
    case SMTC_MODEM_REGION_EU_868:
        memcpy(adr_custom_list_region, adr_custom_list_eu868_default, sizeof(adr_custom_list_region));
        break;

    case SMTC_MODEM_REGION_US_915:
        memcpy(adr_custom_list_region, adr_custom_list_us915_default, sizeof(adr_custom_list_region));
        break;

    case SMTC_MODEM_REGION_AU_915:
        memcpy(adr_custom_list_region, adr_custom_list_au915_default, sizeof(adr_custom_list_region));
        break;

    case SMTC_MODEM_REGION_AS_923_GRP1:
    case SMTC_MODEM_REGION_AS_923_GRP2:
    case SMTC_MODEM_REGION_AS_923_GRP3:
    case SMTC_MODEM_REGION_AS_923_GRP4:
        memcpy(adr_custom_list_region, adr_custom_list_as923_default, sizeof(adr_custom_list_region));
        break;

    case SMTC_MODEM_REGION_KR_920:
        memcpy(adr_custom_list_region, adr_custom_list_kr920_default, sizeof(adr_custom_list_region));
        break;

    case SMTC_MODEM_REGION_IN_865:
        memcpy(adr_custom_list_region, adr_custom_list_in865_default, sizeof(adr_custom_list_region));
        break;

    case SMTC_MODEM_REGION_RU_864:
        memcpy(adr_custom_list_region, adr_custom_list_ru864_default, sizeof(adr_custom_list_region));
        break;

    default:
        break;
    }
    if (smtc_modem_adr_set_profile(0, SMTC_MODEM_ADR_PROFILE_CUSTOM, adr_custom_list_region) != SMTC_MODEM_RC_OK)
        abort();

}

void MyLbmxEventHandlers::joinFail(const LbmxEvent &event)
{
    lorawanState = LorawanStateType::Failed;
}

void MyLbmxEventHandlers::alarm(const LbmxEvent &event)
{
    // not used: timer event from LoRaWAN engine =>we use own task scheduling
}

void MyLbmxEventHandlers::txDone(const LbmxEvent &event)
{
    // flag for uplinkThread
    lorawan_uplink_done = true;
}

void MyLbmxEventHandlers::downData(const LbmxEvent &event)
{
    Serial.println("Downlink received:");
    Serial.printf("  - LoRaWAN Fport = %d\n", event.event_data.downdata.fport);
    Serial.printf("  - Payload size  = %d\n", event.event_data.downdata.length);
    Serial.printf("  - RSSI          = %d dBm\n", event.event_data.downdata.rssi - 64);
    Serial.printf("  - SNR           = %d dB\n", event.event_data.downdata.snr >> 2);

    switch (event.event_data.downdata.window)
    {
    case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX1:
    {
        Serial.printf("  - Rx window     = %s\n", xstr(SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX1));
        break;
    }
    case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX2:
    {
        Serial.printf("  - Rx window     = %s\n", xstr(SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RX2));
        break;
    }
    case SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXC:
    {
        Serial.printf("  - Rx window     = %s\n", xstr(SMTC_MODEM_EVENT_DOWNDATA_WINDOW_RXC));
        break;
    }
    default:
        break;
    }

    if (event.event_data.downdata.length != 0)
    {
        Serial.printf("Payload", event.event_data.downdata.data, event.event_data.downdata.length);
    }
}

void MyLbmxEventHandlers::wifiScanDone(const LbmxEvent &event)
{
    wifi_mw_get_event_data_scan_done(&wifi_results);

    Serial.printf("Wifi scan done. Number of results %u.\n", wifi_results.nbr_results);
    for (uint8_t i = 0; i < wifi_results.nbr_results; i++)
    {
        for (uint8_t j = 0; j < WIFI_AP_ADDRESS_SIZE; j++)
            Serial.printf("%02X ", wifi_results.results[i].mac_address[j]);
        Serial.printf(" -- Channel: %d", wifi_results.results[i].channel);
        Serial.printf(" -- Type: %d", wifi_results.results[i].type);
        Serial.printf(" -- RSSI: %d\n", wifi_results.results[i].rssi);
    }
}
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// LoRaWAN ModemEventHandler

static void ModemEventHandler()
{
    static LbmxEvent event;
    static MyLbmxEventHandlers handlers;

    while (event.fetch())
    {
        //Serial.printf("----- %s -----\n", event.getEventString().c_str());

        handlers.invoke(event);
    }
}
////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////
// LoRaWAN uplink

pt ptUplink;
int uplinkThread(struct pt* pt) {

    static bool uplink_went_wrong = false;
    static int lppChannel = 1;

    PT_BEGIN(pt);

    // Loop forever
    while(true) {
        PT_YIELD_UNTIL(pt, lorawanState == LorawanStateType::Joined);
        lorawan_uplink_done = false;

        // prepare data for uplink
        lpp.reset();
        lppChannel = 1;
        lpp.addGPS_highPrec(lppChannel++, location_lat, location_lng, location_alt);
        lpp.addCapacityBattery(lppChannel++, battery_percent);
        

        // Send it off
        static int app_data_size = lpp.getSize();
        memcpy(lorawan_data_buffer, lpp.getBuffer(), app_data_size);
        uplink_went_wrong = (LbmxEngine::requestUplink(LORAWAN_UPLINK_FPORT, LORAWAN_CONFIRMED_MSG_ON, lorawan_data_buffer, app_data_size) != SMTC_MODEM_RC_OK);

        // wait until LoRaWAN uplink is sent
        // (event handler of LoRaWAN engine will set this flag)
        // or try next uplink if something went wrong
        PT_YIELD_UNTIL(pt, lorawan_uplink_done or uplink_went_wrong);

        PT_SLEEP(pt, UPLINK_MINIMUM_INTERVAL);
    }

    PT_END(pt);
}
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// powersaving: CPU powernap until next millisecond-tick happens
//
// intention: insert short "CPU powernaps" for increased battery lasting
// ATTENTION: too long sleep cycles means buffer overrun of Serial2 UART receive buffer
//            =>"TinyGPSPlus" can't get all NMEA sentences, it shows checksum failures

pt ptCpuPowernap;
int cpuPowernapThread(struct pt* pt) {

    #define WORK_CYCLES_BEFORE_SLEEP 20
    static int work_cycles = 0;

    PT_BEGIN(pt);

    // Loop forever
    while(true) {

        for (work_cycles = 0; work_cycles < WORK_CYCLES_BEFORE_SLEEP; work_cycles += 1) {
            // let other threads run
            PT_YIELD(pt);
        }

        // "System_ON_Sleep" until next timer-tick (timer interrupt every 1ms)
        // help from https://devzone.nordicsemi.com/f/nordic-q-a/490/how-do-you-put-the-nrf51822-chip-to-sleep
        __SEV();    // "Send Event" => makes sure there's an event in event register
        __WFE();    // "Wait For Event" => clears event register
        __WFE();    // "Wait For Event" => "System_ON_Sleep" until timer interrupt after 1ms wakes up cpu
    }

    PT_END(pt);
}
////////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////////////////
// The event handler for the button.
void handleButtonEvent(AceButton* /* button */, uint8_t eventType, uint8_t /* buttonState */)
{
    switch (eventType) {
        case AceButton::kEventClicked:
        case AceButton::kEventReleased:
            // handling of user button event 'Click or Release'

            if (buzzer_melody_request and (buzzer_melody_choice != MELODY_NONE)) {
                // stop melody playing
                buzzer_melody_request = false;
                buzzer_melody_choice = MELODY_NONE;
            }
            break;
        case AceButton::kEventDoubleClicked:
            // handling of user button event 'DoubleClicked'

            // play a short melody
            buzzer_melody_choice = MELODY_NOKIA_RINGTONE;
            buzzer_melody_request = true;
            break;
        case AceButton::kEventLongPressed:
            // handling of user button event 'LongPressed'");

            // play a long melody
            buzzer_melody_choice = MELODY_STARWARS_THEME;
            buzzer_melody_request = true;
            break;
  }
}
////////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////////////////
// Setup and mainloop

void setup()
{
    // debugging
    Serial.begin(9600);
    
    // Sensor reading (e.g. battery voltage)
    pinMode(PIN_SENSE_POWER_EN, OUTPUT);
    digitalWrite(PIN_SENSE_POWER_EN, LOW);
    // Set the analog reference to 3.0V
    analogReference(AR_INTERNAL_3_0);


    // LED pin for blinker
    pinMode(LED_GREEN, OUTPUT);
    digitalWrite(LED_GREEN, LOW);


    // preparing handling of GNSS chip
    // general chip powering
    pinMode(PIN_GNSS_POWER_EN, OUTPUT);
    digitalWrite(PIN_GNSS_POWER_EN, LOW);
    
    // power for keeping the onboard RTC running
    pinMode(PIN_GNSS_VRTC_EN, OUTPUT);
    digitalWrite(PIN_GNSS_VRTC_EN, LOW);
    
    // some reset pins
    pinMode(PIN_GNSS_RESET, OUTPUT);
    digitalWrite(PIN_GNSS_RESET, LOW);
    
    pinMode(PIN_GNSS_SLEEP_INT, OUTPUT);
    digitalWrite(PIN_GNSS_SLEEP_INT, LOW);
    
    pinMode(PIN_GNSS_RTC_INT, OUTPUT);
    digitalWrite(PIN_GNSS_RTC_INT, LOW);
    pinMode(PIN_GNSS_RESETB, INPUT_PULLUP);


    // Button handling initialization
    pinMode(PIN_BUTTON1, INPUT);
    // T1000-E has inverted input (based on example "SingleButtonPullDown.ino")
    button.init(PIN_BUTTON1, LOW);

    ButtonConfig* buttonConfig = button.getButtonConfig();
    buttonConfig->setEventHandler(handleButtonEvent);
    buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterDoubleClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterLongPress);
    
    // from example "Stopwatch.ino"
    buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
    buttonConfig->setLongPressDelay(BUTTON_LONG_PRESS_DURATION);


    // Buzzer initialization
    pinMode(PIN_BUZZER_EN, OUTPUT);
    digitalWrite(PIN_BUZZER_EN, LOW);

    // LoRaWAN initialization
    lbmT1000E.begin();
    LbmxEngine::begin(lbmT1000E.getRadio(), ModemEventHandler);


    // Protothreads
    PT_INIT(&ptBuzzer);
    PT_INIT(&ptBattery);
    PT_INIT(&ptGnss);
    PT_INIT(&ptBlink);
    PT_INIT(&ptGpsAnalyzer);
    PT_INIT(&ptLocation);
    PT_INIT(&ptLorawan);
    PT_INIT(&ptUplink);
    PT_INIT(&ptCpuPowernap);
}

void loop()
{
    PT_SCHEDULE(buzzerThread(&ptBuzzer));
    PT_SCHEDULE(batteryThread(&ptBattery));
    PT_SCHEDULE(gnssThread(&ptGnss));
    PT_SCHEDULE(blinkThread(&ptBlink));
    PT_SCHEDULE(gpsAnalyzerThread(&ptGpsAnalyzer));
    PT_SCHEDULE(locationThread(&ptLocation));
    PT_SCHEDULE(lorawanThread(&ptLorawan));
    PT_SCHEDULE(uplinkThread(&ptUplink));
    PT_SCHEDULE(cpuPowernapThread(&ptCpuPowernap));

    // button event loop
    // Should be called every 4-5ms or faster, for the default debouncing time of ~20ms.
    button.check();

    
    // mainloop time analysis
    if (main_loop_analysis_enabled == true) {
        if (loop_counter >= NOF_MAIN_LOOPS) {
            loop_micros_curr = micros();
            loop_micros_delta = (int)(loop_micros_curr - loop_micros_last);
            loop_micros_avg = loop_micros_delta / NOF_MAIN_LOOPS;

            Serial.printf("mainloop time analysis over %i loops: average is %f4.0 us\n", NOF_MAIN_LOOPS, loop_micros_avg);
            
            loop_counter = 0;
            loop_micros_last = loop_micros_curr;
        }      

        loop_counter += 1;
    }

}

////////////////////////////////////////////////////////////////////////////////
