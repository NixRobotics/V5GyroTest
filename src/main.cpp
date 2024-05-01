/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       nickr                                                     */
/*    Created:      11/16/2023, 9:12:27 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <string>
#include <iostream>

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

#define ISGYRO  1
#ifdef ISGYRO
gyro gyroD = gyro(Brain.ThreeWirePort.D);
#else
analog_in gyroD = analog_in(Brain.ThreeWirePort.D);
#endif
motor Motor1 = motor(PORT1, ratio18_1, false);

// define your global instances of motors and other devices here

struct tGyroLog {
    int32_t time;
    int32_t val;
};

#define MAXLOG 20000
tGyroLog LOGARRAY[MAXLOG];
int32_t logId = 0;

double rawBias = 0.0;
void rawCalibrate() {
    uint32_t startTime = vex::timer::system();
    int32_t accum = 0;
    for (int i = 0; i < 2000; i++) {
        accum += gyroD.value(analogUnits::range12bit);
        wait(1, msec);
    }
    rawBias = (double) accum / 2000.0;
    uint32_t endTime = vex::timer::system();
    printf("Calibration %ld %f (%ld msec)\n", accum, rawBias, endTime - startTime);
}

double rawAccum = 0;
uint32_t lastTime = 0;

void rawChanged()
{
    uint32_t thisTime = vex::timer::system();
    if (lastTime != 0) {
        if (thisTime - lastTime > 1) printf("Time %lu\n", thisTime - lastTime);
    }
    lastTime = thisTime;
    rawAccum += (double) gyroD.value(analogUnits::range12bit) - rawBias;
}

void gyroChanged()
{
    if (logId < MAXLOG) {
        LOGARRAY[logId].time = vex::timer::system();
        LOGARRAY[logId].val = gyroD.value(analogUnits::range12bit);
        logId++;
    } else if (logId == MAXLOG) {
        printf("buffer overflow\n");
        logId++;
    }
}

void DumpLog()
{
    if (!Brain.SDcard.isInserted()) {
        printf("No SDCARD\n");
    }
    if (logId >= MAXLOG) logId = MAXLOG;
    std::string str = "";
    char tempStr[100];
    str += "time, yaw\n";
    for (int i = 0; i < logId; i++) {
        sprintf(&(tempStr[0]), "%ld, %ld\n", LOGARRAY[i].time, LOGARRAY[i].val);
        str += tempStr;
        // str += LOGARRAY[i].time + LOGARRAY[i].val + "\n";
    }

    const char *cStr = str.c_str();
    int len = strlen(cStr);

    if (Brain.SDcard.isInserted()) {
        int saved = Brain.SDcard.savefile("data.csv", (uint8_t *) cStr, len);

        printf("%d of %d bytes written to file\n", saved, len);
        if (Brain.SDcard.exists("data.csv")) {
            printf("File size: %ld\n", Brain.SDcard.size("data.csv"));        }
    } else {
        printf("%s", cStr);
    }
    // printf("%s", str.c_str());
    // Brain.SDcard.savefile
}

void PrintRotation()
{
#ifdef ISGYRO
    printf("rotation = %f\n", gyroD.rotation());
#else
    double scaleFactor = 90.0 / 120000.00;
    printf("ratation = %f\n", rawAccum * scaleFactor);
#endif
}

int RawSampler()
{
    while (1) {
        rawChanged();
        this_thread::sleep_for(1);
    }
}

// PID Controller
struct tPID
{
  float kP, kI, kD;
  float error, integral, derivative, previousError;
  float dT;
  float max;
  float scale;
  float decay;
  uint32_t lastTime;
};

void initPID(tPID *pid, float max)
{
    pid->kP = 10.0;
    pid->kD = 0.0;
    pid->kI = 2.5; //6
    pid->decay = 0.0;
    pid->error = 0.0;
    pid->integral = 0.0;
    pid->derivative = 0.0;
    pid->previousError = 0.0;
    pid->dT = 0.0;
    pid->lastTime = 0;
    pid->max = max;
    pid->scale = 1.0 / 360.0;
}

float computePID(tPID* pid, float error)
{
    uint32_t thisTime = vex::timer::system();
    error = error * pid->scale;
    if (error > 1.0) error = 1.0;
    if (error < -1.0) error = -1.0;

    if (pid->lastTime != 0) {
        pid->dT = ((float)(thisTime - pid->lastTime)) / 1000.0;
    }
    pid->lastTime = thisTime;
    pid->error = error;
    if (pid->dT != 0.0) {
        pid->integral += error * pid->dT;
        float decay = 1.0 - (pid->decay * pid->dT);
        pid->integral *= decay;
        pid->derivative = (error - pid->previousError) / pid->dT;
    }
    pid->previousError = error;

    float prop = pid->kP * pid->error;
    float deriv = pid->kD * pid->derivative;
    float integ = pid->kI * pid->integral;

    float pidval = prop + deriv + integ;

    // printf("pid: %f %f %f\n", prop, pid->integral, pid->derivative);

    if (pidval > pid->max) pidval = pid->max;
    if (pidval < -pid->max) pidval = -pid->max;

    return pidval;
}

#define MAX_VOLTAGE 11.0

void motorSpinTo(motor *thisMotor, double position, double speed)
{
    tPID pid;
    initPID(&pid, MAX_VOLTAGE  * speed / 100.0);
    uint32_t startTime = vex::timer::system();
    bool bDone = false;
    int smallErrorCount = 0;
    double error = 0.0;
    float pidval = 0.0;
    while (!bDone) {
        error = position - thisMotor->position(rotationUnits::deg);
        pidval = computePID(&pid, error);
        // printf("%f %f %f\n", thisMotor->position(rotationUnits::deg), error, pidval);
        thisMotor->spin(directionType::fwd, pidval, voltageUnits::volt);
        this_thread::sleep_for(10);
        if ((vex::timer::system() - startTime) > 20000) bDone = true;
        if ((fabs(pidval) < 0.5) && (fabs(thisMotor->velocity(velocityUnits::dps)) < 1.0)) smallErrorCount++;
        else smallErrorCount = 0;
        if (smallErrorCount > 100) bDone = true;        
    }
    printf("motor: %f, error: %f, pid %f\n", thisMotor->velocity(velocityUnits::dps), error, pidval);
    thisMotor->stop();
}

//Somewhere else
//setArmMotor(computePID(&pid, some_error));

int main() {

    Brain.Screen.printAt( 10, 100, "Hello V5" );
    printf("SDCard: %d\n", Brain.SDcard.isInserted());

    int32_t id = this_thread::get_id();

    printf("%0lx\n", id);

#ifdef ISGYRO
    wait (500, msec);
    gyroD.calibrate(2);
    while (gyroD.isCalibrating()) {
        wait (50, msec);
    }
    gyroD.changed(gyroChanged);
#else
    gyroD.changed(gyroChanged);

    wait (500, msec);

    rawCalibrate();

    vex::thread t(RawSampler);

#endif
    // int32_t val1 = gyroD.value(analogUnits::range12bit);
    // printf("%ld", val1);
    // while(true) {
    //     wait(1, msec);
    //     int32_t val2 = gyroD.value(analogUnits::range12bit);
    //     if (val1 != val2) {
    //         printf("%ld\n", val2);
    //         val1 = val2;
    //     }
    // }

    gyroChanged();

    double gear = 84.0 / 12.0;

    double motorStartPos = Motor1.position(rotationUnits::deg);

    Motor1.setBrake(coast);
    Motor1.setStopping(coast);

    if (true) {
        PrintRotation();
        motorSpinTo(&Motor1, 360.0 * gear, 50.0);
        PrintRotation();
        printf("motor pos = %f\n", Motor1.position(rotationUnits::deg) / gear);
        wait(500, msec);
        //Motor1.spin(directionType::fwd, -4.0, voltageUnits::volt);
        //wait(5, seconds);
        //Motor1.stop();
        Motor1.spinToPosition(motorStartPos, rotationUnits::deg, 25.0, velocityUnits::pct, true);
        PrintRotation();
        printf("motor pos = %f\n", Motor1.position(rotationUnits::deg) / gear);
    }

    if (false) {

        // printf("rotation=%f\n", gyroD.rotation());
        PrintRotation();
        Motor1.spinFor(directionType::fwd, 360.0 * gear, rotationUnits::deg, 25.0, velocityUnits::pct, true);
        PrintRotation();
        wait(2, seconds);
        PrintRotation();
        Motor1.spinFor(directionType::rev, 360.0 * gear, rotationUnits::deg, 25.0, velocityUnits::pct, true);
        PrintRotation();
        wait(2, seconds);
        PrintRotation();
        Motor1.spinFor(directionType::rev, 360.0 * gear, rotationUnits::deg, 25.0, velocityUnits::pct, true);
        PrintRotation();
        wait(2, seconds);
        PrintRotation();
        Motor1.spinFor(directionType::fwd, 360.0 * gear, rotationUnits::deg, 25.0, velocityUnits::pct, true);
        PrintRotation();
        // wait(2, seconds);
        // printf("rotation=%f\n", gyroD.rotation());
        // Motor1.spinFor(directionType::rev, 360.0 * gear, rotationUnits::deg, 25.0, velocityUnits::pct, true);
        // printf("rotation=%f\n", gyroD.rotation());

    }

    DumpLog();
   
    while(true) {
        
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
