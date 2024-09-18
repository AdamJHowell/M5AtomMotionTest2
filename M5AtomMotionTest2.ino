/*
Requires:
  https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/arduino/package_m5stack_index.json
  https://github.com/m5stack/M5Atom
  https://github.com/m5stack/M5Stack/
  https://github.com/m5stack/M5-DLight
  https://github.com/FastLED/FastLED

M5Stack ATOM Lite:
   ATOM Lite HY2.0-4P:
      G, 5V, G26, G32
      PORT.A (onboard), 32 White/SCL, 26 Yellow/SDA
   Default I2C:
      SCL 21
      SDA 25
      Seems to use this button library: https://docs.m5stack.com/en/api/atom/button

M5Stack ATOM Motion (http://docs.m5stack.com/en/atom/atom_motion):
   https://github.com/m5stack/M5Atom/tree/master/examples/ATOM_BASE/ATOM_Motion
   PORT.B (black connector), 23 white, 33 yellow
   PORT.C (blue connector), 19 yellow, 22 white
   Servo angle range 0 ~ 180
   DC motor speed range -127~127
   APDS-9960 default address: 0x39
   ATOM Motion servo:
      Has four servo ports, numbered from 1 to 4.
      uint8_t SetServoPulse( uint8_t Servo_CH, uint16_t width );
         Seems to return 0 on success, and returns 1 on error.
      uint8_t SetServoAngle( uint8_t Servo_CH, uint8_t angle );
         Seems to return 0 on success, and returns 1 on error.
      Servo(1~4)   angle: 0-180   pulse: 500-2500   R/W

M5Stack PaHUB2 (https://docs.m5stack.com/en/unit/pahub2):
   https://github.com/m5stack/M5Stack/tree/master/examples/Unit/PaHUB_TCA9548A
   This project the PaHUB2 connected to PORT.A (the onboard HY2.0 port of the ATOM Lite).

M5Stack Dlight sensor (https://docs.m5stack.com/en/unit/dlight):
   I2C address 0x23
   https://github.com/m5stack/M5-DLight
   Values from 1-65535, greater values represent brighter light.
   This project uses 4 M5Stack DLight sensors connected to the PaHUB2.
*/


#include <M5_DLight.h>
#include "AtomMotion.h"
#include <cstdint>
#include <Arduino.h>


// Constants
const int SDA_GPIO = 26;                     // Use this to set the SDA GPIO if your board uses a non-standard GPIOs for the I2C bus.
const int SCL_GPIO = 32;                     // Use this to set the SCL GPIO if your board uses a non-standard GPIOs for the I2C bus.
const int PCA_ADDRESS = 0x70;                // The I2C address of the Pa.HUB.
const long RED = 0xFF0000;                   // The RGB code for the color red.
const long ORANGE = 0xFF8000;                // The RGB code for the color orange.
const long YELLOW = 0xFFFF00;                // The RGB code for the color yellow.
const long GREEN = 0x00FF00;                 // The RGB code for the color green.
const long BLUE = 0x0000FF;                  // The RGB code for the color blue.
const long INDIGO = 0x4B0082;                // The RGB code for the color indigo.
const long VIOLET = 0xEE82EE;                // The RGB code for the color violet.
const long BLACK = 0x000000;                 // The RGB code for the color black.
const long MAGENTA = 0xFF00FF;               // The RGB code for the color magenta.
const long CYAN = 0x00FFFF;                  // The RGB code for the color cyan.
const long WHITE = 0xFFFFFF;                 // The RGB code for the color white.
const unsigned int PORT_B = 23;              // The address of port B.
const unsigned int PORT_C = 22;              // The address of port C.
const uint8_t AZIMUTH_SERVO = 3;             // The ATOM Motion port that the azimuth servo is plugged into.
const uint8_t ALTITUDE_SERVO = 4;            // The ATOM Motion port that the altitude servo is plugged into.
const unsigned int NUM_SENSORS = 4;          // The number of sensors.
const unsigned int SERVO_MIN = 500;          // The minimum pulse width for the servos.
const unsigned int SERVO_MAX = 2500;         // The maximum pulse width for the servos.
const unsigned int DEAD_BAND = 100;          // The minimum delta before a servo will move.
const unsigned long LOOP_DELAY = 10;         // The maximum value of 4,294,967,295 allows for a delay of about 49.7 days.
const unsigned long PRINT_LOOP_DELAY = 1000; // The minimum time between serial printing of the lux values.
// Variables
uint16_t azimuthSpeed = 1500;                                 // The pulse width of the azimuth servo signal in microseconds.
uint16_t altitudeSpeed = 1500;                                // The pulse width of the altitude servo signal in microseconds.
unsigned int buttonCount = 0;                                 // Tracks the number of times the ATOM button was pressed.
unsigned long lastLoop = 0;                                   // Tracks the last time the main loop executed.
unsigned long lastPrintLoop = 0;                              // Tracks the last time the print loop executed.
unsigned long ledColor = 0xFFFFFF;                            // Tracks the color of the onboard LED.
uint16_t luxValues[NUM_SENSORS] = { 2112, 2112, 2112, 2112 }; // An array of light readings: top left, top right, bottom left, bottom right.
uint8_t sensorAddresses[NUM_SENSORS] = { 0, 1, 4, 5 };        // An array of the Pa.HUB ports with DLIGHT sensors attached.
// Class instances
AtomMotion atomMotion;                   // An object to manage the ATOM Motion.
M5_DLight sensorArray[NUM_SENSORS] = {}; // An array of DLIGHT sensor objects.

const unsigned int SERVO_LOOP_DELAY = 100;
unsigned long lastServoLoop = 0;
bool incrementing = false;


/*
 * The channelSelect function changes the current active channel of the PaHUB I2C multiplexer.
 */
void channelSelect( const uint8_t i )
{
   if( i > 7 )
      return;
   Wire.beginTransmission( PCA_ADDRESS );
   Wire.write( 1 << i );
   Wire.endTransmission();
} // End of channelSelect()


/*
 * pulseWidth is a global that is updated in loop().
 * pvParameters is not used.
 */
void TaskMotion( void *pvParameters )
{
   while( true )
   {
      M5.dis.drawpix( 0, ledColor );
      atomMotion.SetServoPulse( AZIMUTH_SERVO, azimuthSpeed );
      atomMotion.SetServoPulse( ALTITUDE_SERVO, altitudeSpeed );
      Serial.print( "AZ speed " );
      Serial.println( azimuthSpeed );
      // Give other threads a chance to take control of the core.
      vTaskDelay( 0 );
   }
} // End of TaskMotion()


void setup()
{
   // Wire.begin( SDA_GPIO, SCL_GPIO );  // I'm pretty sure this breaks the AtomMotion servos.
   // M5.begin( SerialEnable, I2CEnable, DisplayEnable );
   M5.begin( true, false, true );
   Serial.println( "\nBeginning setup()." );
   atomMotion.Init();

   // Establish the two ports as inputs.
   pinMode( PORT_B, INPUT_PULLUP );
   pinMode( PORT_C, INPUT_PULLUP );

   // Pin the TaskMotion() function to core 0.
   xTaskCreatePinnedToCore(
       TaskMotion,   // Pointer to the task entry function.
       "TaskMotion", // A descriptive name for the task.
       4096,         // The size of the task stack specified as the number of bytes.
       nullptr,      // Pointer that will be used as the parameter for the task being created.
       2,            // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
       nullptr,      // Used to pass back a handle by which the created task can be referenced.
       0 );          // Values 0 or 1 indicate the CPU core which the task will be pinned to.

   // Turn on the LED and set it to white.
   M5.dis.drawpix( 0, WHITE );

   // Configure every M5Stack DLIGHT sensor.
   for( uint8_t i = 0; i < NUM_SENSORS; i++ )
   {
      channelSelect( sensorAddresses[i] );
      sensorArray[i].begin();
      sensorArray[i].setMode( CONTINUOUSLY_H_RESOLUTION_MODE );
   }

   Serial.println( "\nFinished setup().\n" );
} // End of setup()


void loop()
{
   // M5.update() seems to only call M5.Btn.read(), which reads the state of the in-built button.
   M5.update();

   // If the down stop is tripped, prevent the servo from moving downward.
   if( !digitalRead( PORT_C ) )
   {
      // ToDo: Determine if this should be 1500-2500 instead.
      altitudeSpeed = constrain( altitudeSpeed, 500, 1500 );
      ledColor = BLUE;
      Serial.printf( "-- Hit limit C down stop!\n" );
   }

   // If the up stop is tripped, prevent the servo from moving upward.
   if( !digitalRead( PORT_B ) )
   {
      // ToDo: Determine if this should be 500-1500 instead.
      altitudeSpeed = constrain( altitudeSpeed, 1500, 2500 );
      ledColor = GREEN;
      Serial.printf( "-- Hit limit B up stop!\n" );
   }

   if( ( lastServoLoop == 0 ) || ( millis() - lastServoLoop ) > SERVO_LOOP_DELAY )
   {
      if( azimuthSpeed == 2500 )
      {
         azimuthSpeed = 1500;
         altitudeSpeed = 1500;
         incrementing = false;
      }
      else if( azimuthSpeed == 500 )
      {
         azimuthSpeed = 1500;
         altitudeSpeed = 1500;
         incrementing = true;
      }
      if( incrementing )
      {
         azimuthSpeed += 10;
         altitudeSpeed += 2;
      }
      else
      {
         azimuthSpeed -= 10;
         altitudeSpeed -= 2;
      }
      lastServoLoop = millis();
   }

   if( ( lastLoop == 0 ) || ( millis() - lastLoop ) > LOOP_DELAY )
   {
      if( M5.Btn.lastChange() > lastLoop )
      {
         buttonCount++;
         Serial.printf( "  Button press!\n" );
         if( buttonCount > 9 )
            buttonCount = 0;
         switch( buttonCount )
         {
            case 0:
               ledColor = RED;
               break;
            case 1:
               ledColor = ORANGE;
               break;
            case 2:
               ledColor = YELLOW;
               break;
            case 3:
               ledColor = GREEN;
               break;
            case 4:
               ledColor = BLUE;
               break;
            case 5:
               ledColor = INDIGO;
               break;
            case 6:
               ledColor = VIOLET;
               break;
            case 7:
               ledColor = MAGENTA;
               break;
            case 8:
               ledColor = CYAN;
               break;
            case 9:
               ledColor = BLACK;
               break;
            default:
               break;
         }
      }
      lastLoop = millis();
   }
} // End of loop()
