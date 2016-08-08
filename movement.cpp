#ifndef MOVEMENT
#define MOVEMENT

#include "Motor.h"
#include "Nxt.h"
#include "Clock.h"
#include <cstdlib>
#include "TouchSensor.h"
#include "SonarSensor.h"

#include "communication.cpp"

#define MOTOR_CENTIMETERS_PR_DEGREE 0.0294159265
#define MAX_TURN_DEGREES 360
#define CALIBRATION_PRECISION_IN_DEGREES 5
#define MOTOR_ALIGN_PRECISION 5

#define CENTIMETERS_TO_MOVE_FORWARD 10
#define MOVEMENT_SPEED 70

namespace medina {
    using namespace ecrobot;

    class Movement {
        Motor* left;
        Motor* right;
        Communication* comm;
        TouchSensor* touch;
        SonarSensor* sonar;

        float ROBOT_WEIGHT_INFLUENCE_ON_DEGREES;

    public:
        Movement(Motor* _left, Motor* _right, Communication* _comm,
            TouchSensor* _touch, SonarSensor* _sonar) {

            left    = _left;
            right   = _right;
            comm    = _comm;
            touch   = _touch;
            sonar   = _sonar;

            ROBOT_WEIGHT_INFLUENCE_ON_DEGREES   = 7; // Before 5.6
        }

        void stopMotor() {
            left->reset();
            right->reset();
            comm->log("Stop");
        }

        void moveForwards(float moveCentimeters) {
            //comm->log("Moving forwards");
            float degrees = moveCentimeters / (float)MOTOR_CENTIMETERS_PR_DEGREE;
            moveStraight(degrees, abs(MOVEMENT_SPEED));
        }

        /*
        void rotateLeft(float degrees) {
            if (degrees <= 0) {
                comm->log("Negative turn degree value is not valid! (Left Turn)");
                systick_wait_ms(2000); //Some time to read the error
            } else if (degrees > MAX_TURN_DEGREES) {
                comm->log("Turn degree above MAX_TURN_DEGREES not valid! (Left Turn)");
                systick_wait_ms(2000); //Some time to read the error
            } else {
                left->setPWM(-abs(MOVEMENT_SPEED));
                right->setPWM(abs(MOVEMENT_SPEED));
                rotationHandler(degrees, right);
                stopMotor();
            }
        }
        */

        void rotateRight(int degrees) {
            if (degrees <= 0) {
                //comm->log("Negative turn degree value is not valid! (Right Turn)");
                systick_wait_ms(2000); //Some time to read the error
            } else if (degrees > MAX_TURN_DEGREES) {
                //comm->logFormat("Turn degree above %d not valid! (Right Turn)", (int)MAX_TURN_DEGREES);
                systick_wait_ms(2000); //Some time to read the error
            } else {
                right->setPWM(-abs(MOVEMENT_SPEED));
                left->setPWM(abs(MOVEMENT_SPEED));
                rotationHandler(degrees, left);
                stopMotor();
            }
        }

        void rotationHandler(int degrees, Motor *motor) {
            while(true) {
                // When the motors has turned the desired amount of degrees the loop breaks
                if (motor->getCount() > (int)(degrees * (float)this->ROBOT_WEIGHT_INFLUENCE_ON_DEGREES)) {
                    break;
                }
            }
        }

   private:
        void moveStraight(float degrees, int _pwm) {
            left->setPWM(_pwm);
            right->setPWM(_pwm);
            int Pwm = _pwm;
            // comm->log("Moving straight");
            while (true) {
                float avgDegrees = (((left->getCount()) + (right->getCount())) / 2);
                if (!areMotorsAligned()) {
                    //comm->log("Motors not aligned. Forcing alignment...");
                    if(left->getCount() > right->getCount()) {
                        left->setPWM(Pwm - 5);
                    } else if (right->getCount() > left->getCount()) {
                        left->setPWM(Pwm + 5);
                    }
                }

                // Handles backward drive
                if (degrees < 0 && avgDegrees <= degrees) {
                    break;
                // Handles forward drive
                } else if (degrees >= 0 && avgDegrees >= degrees) {
                    break;
                }

                //Security
                /*if(alertMotorStopAndTurn()) {
                    // One or more sensors were activated
                    break;
                }*/
            }

            stopMotor();
        }

        bool areMotorsAligned() {
            int leftMotorCount = left->getCount();
            int rightMotorCount = right->getCount();
            int motorDiff = leftMotorCount - rightMotorCount;

            if (abs(motorDiff) < 1) {
                return true;
            } else {
                return false;
            }
        }

        /*Safety*/
        //If Touch or Sonar sensor is activated --> Stop motor, move back 10 cm and turn 90 degrees.
        bool alertMotorStopAndTurn() {
            if (alertSonar() || alertTouch()) {
                stopMotor();
                moveForwards(-10);
                rotateRight(90);
                stopMotor();

                return true;
            }

            return false;
        }

        bool alertTouch() {
            if (touch->isPressed()) {
                return true;
            } else {
                return false;
            }
        }

        //If Sonar get a distance over 2 cm, stop get the value true
        bool alertSonar() {
            if (sonar->getDistance() > 20) {
                comm->log("A sonar sensor was triggered!");
                return true;
            } else {
                return false;
            }
        }
    };
}
#endif
