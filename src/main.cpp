#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/logger/stdout.hpp"
#include "pros/misc.h"

// Variables

// Motor Definitions

pros::Motor left_back_mtr(9, pros::E_MOTOR_GEAR_RED, true,
                          pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor left_middle_mtr(8, pros::E_MOTOR_GEAR_RED, false,
                            pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor left_front_mtr(5, pros::E_MOTOR_GEAR_RED, false,
                           pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor right_back_mtr(7, pros::E_MOTOR_GEAR_RED, false,
                           pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor right_middle_mtr(3, pros::E_MOTOR_GEAR_RED, true,
                             pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor right_front_mtr(4, pros::E_MOTOR_GEAR_RED, true,
                            pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor intake(11, pros::E_MOTOR_GEAR_GREEN, true,
                   pros::E_MOTOR_ENCODER_DEGREES);

pros::Controller cntrlr(pros::E_CONTROLLER_MASTER);

pros::Motor_Group left({9, 8, 5});
pros::Motor_Group right({7, 3, 4});
#define BackWingsPort 'A'
#define IntakePort 'B'
#define RightWingPort 'C'
#define LeftWingPort 'H'
#define deScore 'G'

pros::ADIDigitalOut BackWings(BackWingsPort, false);
pros::ADIDigitalOut IntakePull(IntakePort, true);
pros::ADIDigitalOut RightWing(RightWingPort, false);
pros::ADIDigitalOut LeftWing(LeftWingPort, false);
pros::ADIDigitalOut descore(deScore, false);

pros::Imu imuSensor(12);

// drivetrain settings
lemlib::Drivetrain drivetrain(
    &left,                      // left motor group
    &right,                     // right motor group
    13,                         // 15 inch track width
    lemlib::Omniwheel::NEW_275, // using new 3.25" omnis
    600,                        // drivetrain rpm is 600
    10 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings
    linearController(10, // proportional gain (kP)
                     0,
                     1, // derivative gain (kD)
                     0,
                     0.1,  // small error range, in inches
                     500,  // small error range timeout, in milliseconds
                     0.5,  // large error range, in inches
                     1000, // large error range timeout, in milliseconds
                     17    // maximum acceleration (slew)
    );

// angular motion controller
lemlib::ControllerSettings
    angularController(2.5, // proportional gain (kP)
                      0, 14, 0,
                      1,    // small error range, in degrees
                      500,  // small error range timeout, in milliseconds
                      2,    // large error range, in degrees
                      1000, // large error range timeout, in milliseconds
                      17    // maximum acceleration (slew)
    );

// sensors for odometry
// note that in this example we use internal motor encoders, so we don't pass
// vertical tracking wheels
lemlib::OdomSensors sensors(
    nullptr,   // vertical tracking wheel 1, set to nullptr as we don't have one
    nullptr,   // vertical tracking wheel 2, set to nullptr as we don't have one
    nullptr,   // horizontal tracking wheel 1
    nullptr,   // horizontal tracking wheel 2, set to nullptr as we don't have a
               // second one
    &imuSensor // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController,
                        sensors);

void checkMotorsAndReturnTemperature() {
  std::vector<pros::Motor> motors = {
      left_back_mtr,  left_middle_mtr,  left_front_mtr, right_front_mtr,
      right_back_mtr, right_middle_mtr, intake};

  while (true) {
    double totalTemp = 0.0;
    int count = 0;

    for (auto &motor : motors) {
      double temp = motor.get_temperature();
      if (temp ==
          PROS_ERR_F) { // PROS_ERR_F is returned when the motor is unplugged
        cntrlr.set_text(
            0, 0, "Motor " + std::to_string(motor.get_port()) + " unplugged.");
        pros::delay(250);
        cntrlr.rumble("---");
      }

      if (count < 6) {
        totalTemp += temp;
      }
      ++count;
    }

    if (count == 0)
      cntrlr.set_text(0, 0, "No motors found.");

    double averageTempCelsius = totalTemp / count;
    double averageTempFahrenheit = averageTempCelsius * 9.0 / 5.0 + 32.0;
    cntrlr.set_text(0, 0, "Avg Temp: " + std::to_string(averageTempFahrenheit));

    pros::delay(250);
  }
}

void initialize() {

  // task to make sure all motors are plugged in and check the temperature of
  // the drivetrain
  pros::Task motorCheck(checkMotorsAndReturnTemperature);
  // selector::init();

  pros::lcd::initialize();

  chassis.calibrate();

  // thread to for brain screen and position logging
  pros::Task screenTask([&]() {
    lemlib::Pose pose(0, 0, 0);
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // log position telemetry
      lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
      // delay to save resources
      pros::delay(50);
    }
  });
}
void openFrontWings() {
  LeftWing.set_value(true);
  RightWing.set_value(true);
}
void closeFrontWings() {
  LeftWing.set_value(false);
  RightWing.set_value(false);
}

void disabled() {}
ASSET(longAutonNormal_txt);
ASSET(longAutonRushClose_txt);
ASSET(longAutonRushFar_txt);
ASSET(ShortAutonDisruptFoothill_txt);

ASSET(ShortAutonGrabOneClose_txt);
ASSET(ShortAutonGrabOneFar_txt);
ASSET(ShortAutonSafe_txt);

void ShortAutonDisruptStatesVersion() {
  chassis.setPose(-36, -55, 0);
  intake.move(127);
  chassis.moveToPose(-28, -3, 20, 2000);
  chassis.waitUntilDone();

  pros::delay(2000);
  intake.move(0);
  chassis.turnTo(-10, -2, 2000);
  // ^ turns to the long barrier
  chassis.waitUntilDone();
  LeftWing.set_value(true);
  // hits the ball into the other side to disrupt
  chassis.moveToPoint(-7, -7, 1200);
  chassis.waitUntilDone();
  LeftWing.set_value(false);
  // ^ consistent up to here!

  chassis.setPose(-7, -4, 90);
  // turn to the match load bar - needs to not hit the circle thing on the way!
  chassis.turnTo(-50, -59, 1100);

  // moving drivetrain back a bit so it does not slam into the match load bar

  chassis.moveToPoint(-42, -51, 2000, true, 70);
  chassis.waitUntilDone();
  // ^ turn and move to the match load bar

  chassis.setPose(-53, -46, 250);

  // ^ gets to the matchload bar -> now need to move towards goal to then rotate
  // back and knock ball out of match load.
  // should be turning towards the end of the match load zone and lining up for
  // descoring

  // testing field stops

  chassis.turnTo(-55, -27, 1000);

  // should be moving through the match load zone and descore.
  descore.set_value(true);
  // doing the descore movement
  // just changed from 65 -> 75 to move further back! Everything else was fine
  // before! - also just > speed by 20 for more power to knock it out
  chassis.moveToPoint(-52, -75, 1000, false, 88);
  chassis.waitUntilDone();

  descore.set_value(false);
  // descoring the triball in the match load zone - uncomment the descore
  // function after consistent movement to avoid damage
  // turn towards the alleyway for getting to the elevation bar
  chassis.turnTo(-8, -110, 1000);
  // move to elevation bar
  // need to reduce the x value below - it moves too much into the alley
  chassis.moveToPoint(-45, -150, 1000);
  chassis.waitUntilDone();
  //
  chassis.cancelAllMotions();
  // chassis.turnTo(-63.044, -73.238, 600);
  // chassis.waitUntilDone();
  //

  //  chassis.turnTo(-8.5, -61, 1000);++
  //  chassis.moveToPoint(-8.5, -61, 2000, true, 70);
  //  LeftWing.set_value(true);
  //  intake.move(-127);
}

// void sixBallNormal() {
//     chassis.setPose(10, -59, 270);

//     //get ball under the elevation bar
//     intake.move(127);
//     chassis.moveToPoint(9, -59, 300);
//     chassis.waitUntilDone();
//     intake.move(0);

//     //backing up to the Matchload bar
//     chassis.moveToPoint(39, -59, 1000, {.forwards = false});
//     //deloading the matchload bar ball
//     chassis.moveToPose(61, -45, 235, 2000, {.forwards = false});
//     chassis.waitUntil(18);
//     BackWings.set_value(true);
//     pros::delay(500);
//     BackWings.set_value(false);
//     //scoring the balls
//     chassis.moveToPose(61, -19.5, 180, 1000, {.forwards = false});
//     //going forward to turn around and scoring intaked ball
//     chassis.moveToPoint(62, -42.5, 1000);
//     //turning to face the goal and push ball
//     chassis.turnToHeading(0, 1000);
//     chassis.moveToPoint(61, -19.5, 2000);
//     //back up and face toward first middle triball
//     chassis.moveToPoint(62, -42.5, 1000, {.forwards = false});
//     chassis.turnToHeading(285, 1000);
//     chassis.waitUntilDone();
//     //intake the ball
//     intake.move(127);
//     chassis.moveToPoint(11, -26.5, 2000);
//     chassis.waitUntilDone();
//     intake.move(0);
//     //turning to goal and letting it go
//     chassis.turnToHeading(60, 1000);
//     chassis.waitUntilDone();
//     intake.move(-127);
//     chassis.moveToPoint(22.5, -20.5, 2000);
//     chassis.waitUntilDone();
//     intake.move(0);
//     //turning to middle ball touching the bar and turning and pushing hte two
//     in chassis.turnToHeading(320, 1000); chassis.waitUntilDone();
//     intake.move(127);
//     chassis.moveToPoint(10, -6, 4000);
//     chassis.turnToHeading(90, 1000);
//     chassis.waitUntilDone();
//     openFrontWings();
//     chassis.moveToPoint(42, -6, 1000);
//     //backing up and facing direction to stop bowling
//     chassis.moveToPoint(30, -6, 1000, {.forwards = false});
//     chassis.turnToHeading(300,1000);

// }

void tune() {
  chassis.setPose(34, -61.5, 0);
  chassis.moveToPoint(34, -15, 4000);
}

void autonomous() { ShortAutonDisruptStatesVersion(); }

void screen() {}

void competition_initialize() {}

// double calculateMotorOutput(int input, int t) {
//     double exp_t = exp(-((double)t / 10.0));
//     double exp_input = exp(((double)(abs(input) - 127) / 10.0));
//     double term = 1 - exp_t;

//     return (exp_t + exp_input * term) * input;
// }

// void arcadeDriveJoyStickCurve() {
//     int forwardInput = cntrlr.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//     int turnInput = cntrlr.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
//     int time = 2;

//     double left_output = calculateMotorOutput(forwardInput + turnInput,
//     time); double right_output = calculateMotorOutput(forwardInput -
//     turnInput, time);

//     left.move_velocity(left_output);
//     right.move_velocity(right_output);
// }

void arcadeDrive() {
  int forward = cntrlr.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int turn = cntrlr.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

  // Calculate the left and right motor velocities
  int left_velocity = forward + turn;
  int right_velocity = forward - turn;

  // Set the left and right motor velocities
  left.move(left_velocity);
  right.move(right_velocity);
}

void opcontrol() {

  autonomous();

  bool wingsFrontOut = true;
  bool wingsBackOut = true;
  bool wingsFrontLeftOut = true;
  bool wingsFrontRightOut = true;
  bool intakeUp = false;

  while (true) {

    arcadeDrive();

    if (cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.move(127);

    } else if (cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move(-127);
    } else {
      intake.move(0);
    }
    if (cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && wingsFrontOut) {
      openFrontWings();
      pros::delay(200);
      wingsFrontOut = false;
    } else if (cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
               !wingsFrontOut) {
      closeFrontWings();
      pros::delay(200);
      wingsFrontOut = true;
    }

    if (cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_B) && wingsFrontLeftOut) {
      LeftWing.set_value(true);
      pros::delay(200);
      wingsFrontLeftOut = false;
    } else if (cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_B) &&
               !wingsFrontLeftOut) {
      LeftWing.set_value(false);
      pros::delay(200);
      wingsFrontLeftOut = true;
    }
    if (cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_Y) &&
        wingsFrontRightOut) {
      RightWing.set_value(true);
      pros::delay(200);
      wingsFrontRightOut = false;
    } else if (cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_Y) &&
               !wingsFrontRightOut) {
      RightWing.set_value(false);
      pros::delay(200);
      wingsFrontRightOut = true;
    }

    if (cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_A) && intakeUp) {
      IntakePull.set_value(true);
      pros::delay(200);
      cntrlr.rumble("-");
      intakeUp = false;
    } else if (cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_A) && !intakeUp) {
      IntakePull.set_value(false);
      pros::delay(200);
      cntrlr.rumble(".");
      intakeUp = true;
    }

    if (cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && wingsBackOut) {
      BackWings.set_value(true);
      pros::delay(200);
      wingsBackOut = false;
    } else if (cntrlr.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) &&
               !wingsBackOut) {
      BackWings.set_value(false);
      pros::delay(200);
      wingsBackOut = true;
    }

    pros::delay(10);
  }
}