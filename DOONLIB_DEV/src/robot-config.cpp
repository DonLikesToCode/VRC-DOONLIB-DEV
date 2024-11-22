#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

controller Controller1 = controller(primary);
motor LeftMotorA = motor(PORT11, ratio6_1, false);
motor LeftMotorB = motor(PORT12, ratio6_1, true);
motor LeftMotorC = motor(PORT13, ratio6_1, true);
motor_group LeftMotorGroup = motor_group(LeftMotorA, LeftMotorB, LeftMotorC);
motor RightMotorA = motor(PORT1, ratio6_1, true);
motor RightMotorB = motor(PORT2, ratio6_1, false);
motor RightMotorC = motor(PORT3, ratio6_1, false);
motor_group RightMotorGroup = motor_group(RightMotorA, RightMotorB, RightMotorC);
motor IntakeMotor = motor(PORT14, ratio18_1, false);


void vexcodeInit(void) {
  // Nothing to initialize
}