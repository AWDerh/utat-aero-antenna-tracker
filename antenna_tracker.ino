/**
 * File: antenna_tracker.sketch
 * Purpose: Arduino C program for tracking a device.
 * Author: Abdul. D
 * Date: 2016-10-20
 * 
 * TODO: optimize to use as little memory as possible
 * Changes:
 * - v1.1 - added home bearing support, inverted the pitch, adding a command line based processing system; verbosity; compartmentalized some tasks.
 * - v1.0 - initial release
 */

#include <Servo.h>
#include <Math.h>

#define PIN_GPS 1
#define PIN_SERVO_YAW 10
#define PIN_SERVO_PITCH 11
#define SERVO_PWM_MIN 1000
#define SERVO_PWM_MAX 2000
#define GPS_LAT 43.659334
#define GPS_LONG -79.400632
#define EARTH_RADIUS_METERS 6367990

#define VERBOSE

// dif in radius of earth at 43.6 to 43.7 (11km) is 6368.012 - 6367.974km is 38meters
// so have a setup function to calculate radius at given lat and long
// 43.659962 -79.400639
#define EARTH_RADIUS 6378137

Servo servoYaw;   // Servo yaw
Servo servoPitch; // Servo pitch

// default GPS data
float gpsLat = GPS_LAT * 71 / 4068;
float gpsLong = GPS_LONG * 71 / 4068;
float homeBearing = 45;

struct Point
{
    float x, y, z = 0;  // lat, long, height
    float bearing = 0;  // considered over a sphere
    float distance = 0; // considered over a sphere
};

void printDecimalPoint(float &toPrint, const char *msg);
void computePoint(Point &input, Point &output);
void printInput(float &toPrint);
void handleCoords();
void updateServos();

Point latLong; // IN DEGREES
Point quadPt;
String command;

void loop()
{
    while (Serial && Serial.available() > 0)
    {
        command = Serial.readStringUntil(' ');

#ifdef VERBOSE
        Serial.print("Processing Command: ");
        Serial.println(command);
#endif

        if (command == "coords")
        {
            handleCoords();
            updateServos();
        }
        else if (command == "homebearing")
        {
            homeBearing = Serial.parseFloat();
#ifdef VERBOSE
            Serial.println("----- UPDATING HOME BEARING -----");
            printDecimalPoint(homeBearing, "New home bearing: ");
            ;
            Serial.println("--------------------------------");
#endif
            updateServos();
        }
#ifdef VERBOSE
        else
        {
            Serial.println("Unsupported operation.");
        }
#endif
    }
}

void handleCoords()
{
    // read coords from serial
    latLong.x = Serial.parseFloat(); // lat
    latLong.y = Serial.parseFloat(); // lon
    latLong.z = Serial.parseFloat(); // height ABOVE the ground

#ifdef VERBOSE
    Serial.println("----- HANDLING COORDINATES -----");
    printDecimalPoint(latLong.x, "Latitude: ");
    printDecimalPoint(latLong.y, "Longitude: ");
    printDecimalPoint(latLong.z, "Height: ");
    Serial.println("--------------------------------");
#endif

    // Radians conversion
    latLong.x = latLong.x * 71 / 4068;
    latLong.y = latLong.y * 71 / 4068;

    // compute x y given lat long
    computePoint(latLong, quadPt);
}

void updateServos()
{
    // calculate pitch and yaw
    float pitch = atan2(latLong.z, quadPt.distance) * (4068 / 71);
    float yaw = quadPt.bearing;

#ifdef VERBOSE
    printDecimalPoint(quadPt.distance, "Quadcopter distance: ");
    printDecimalPoint(quadPt.bearing, "Bearing from geographic north: ");
    printDecimalPoint(pitch, "Pitch (deg) from ground: ");
#endif

    // move servos
    // centered at homebearing, we go 90 less and 90 more, where home bearing is CW from geographic north
    // so 90 less is 180 (all the way to the left) and 90 more is 0 (all the way to the right).
    servoYaw.write(map(quadPt.bearing, homeBearing - 90, homeBearing + 90, 180, 0));
    servoPitch.write(pitch);
}

/*
Point &input - the latitude and logitude that the quadcopter is at
return: mutates output's x and y to the cartesian coordinate x,y 
*/
void computePoint(Point &input, Point &output)
{
    // using equirectangular approximation because of arduino's limited precision
    // works for small distances; max of 3/6371 is what I call small.
    output.x = (input.y - gpsLong) * cos((input.x + gpsLat) / 2);
    output.y = (input.x - gpsLat);
    output.distance = sqrt((output.x * output.x) + (output.y * output.y)) * EARTH_RADIUS_METERS;

    output.bearing = atan2(sin(input.y - gpsLong) * cos(input.x),
                           cos(gpsLat) * sin(input.x) -
                               sin(gpsLat) * cos(input.x) * cos(input.y - gpsLong));
    output.bearing = (output.bearing * 4068) / 71; // to degrees
}

void setup()
{

    // Servo setup
    servoYaw.attach(PIN_SERVO_YAW);
    servoYaw.write(0);
    servoPitch.attach(PIN_SERVO_PITCH);
    servoPitch.write(0);
    delay(2000);

    // GPS setup
    // Optional functionality, for now we omitted; assume constant GPS
    // Coordinates in #define header

    //
    Serial.begin(9600); // baud rate
    Serial.print("Current coordinates: ");
    printDecimalPoint(gpsLat, "(");
    printDecimalPoint(gpsLong, ")");
    Serial.println("Ready");
}

void printDecimalPoint(float &toPrint, const char *msg)
{
    static char outstr[9];

    Serial.print(msg);
    dtostrf(toPrint, 9, 6, outstr);
    Serial.println(outstr);
}

void printInput(float &toPrint)
{
    printDecimalPoint(toPrint, "Recieved: ");
}
