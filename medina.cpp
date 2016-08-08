#include "Motor.h"
#include "Nxt.h"
#include "Clock.h"
#include "Lcd.h"
#include "Usb.h"
#include "SonarSensor.h"
#include "TouchSensor.h"
#include "CompassSensor.h"

#include "communication.cpp"
#include "movement.cpp"

#include "slam.cpp"

#include <cstdlib>

using namespace medina;
using namespace ecrobot;

extern "C"
{
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

Clock clock;
Usb usb;
Communication comm(&usb, &clock);


#define COUNT 501
#define MOVEMENT_SPEED 70

// Motor has to be global according to ecrobot API
Motor leftMotor(PORT_A);
Motor rightMotor(PORT_B);

// Global variables for safety_controller
SonarSensor sonar(PORT_1);
TouchSensor touch(PORT_2);

CompassSensor compass(PORT_4);

Movement move(&leftMotor, &rightMotor, &comm, &touch, &sonar);
Slam slam(&comm, &clock);

int compassData; int cmMoved;

DeclareTask(TaskSlam);
DeclareTask(TaskSafety);
DeclareTask(TaskImage);

void user_1ms_isr_type2(void) {
	SleeperMonitor(); // Needed for I2C device and Clock classes
    comm.handler();
}

void cb(unsigned short *data){
    slam.begin(data, compassData, cmMoved);
    comm.lock = false;
}

void imageRecognition(unsigned short *data) {
    // Callback for img
    if(data[0] != 0 && data[1] != 0) {
        comm.logFormat("x: %hu y: %hu", data[1], data[0]);

        slam.updateMapWithObject(data);
    }
    comm.lock = false;
}

TASK(TaskSlam) {
    comm.setKinectDataCallback(&cb);
	Nxt nxt;
	Lcd lcd;

    compass.beginCalibration();
    compass.endCalibration();
    while(1) {
        clock.wait(1000); // Wait for compass to stabilize
        compassData = (int)compass.getHeading();

        comm.lock = true;
        comm.send(0, 0x0, 0x4);
        while(comm.lock) { clock.wait(250); }

        clock.sleep(1);
    }
}

TASK(TaskSafety) {
    clock.wait(1000);

    while(1) {
        move.rotateRight(20);
        cmMoved = 0;
        slam.tooClose = 0;
        clock.sleep(1);
    }

}

TASK(TaskImage) {
    comm.setImageRecognitionCallback(&imageRecognition);

    while(1) {
        comm.lock = true;
        comm.ImageRecognition();

        while(comm.lock == true) {
            clock.wait(250);
        }
        clock.sleep(1);
    }
}
}
