#include "main.h"

#define BASE_OFFSET 6.55_in
#define GOAL_OFFSET 10.3_in
#define RING_OFFSET 10.05_in
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

bool clamp_bool = true;
bool arm_bool = true;
bool hang_bool = true;

std::shared_ptr<OdomChassisController> drive =
    ChassisControllerBuilder()
        .withMotors({-5, 6, -7, 8}, {1, -2, 3, -4})
        // Green gearset, 4 in wheel diam, 11.5 in wheel track
        .withDimensions(AbstractMotor::gearset::blue, {{10.9_in, 2.75_in}, imev5BlueTPR})
		.withOdometry()
        .buildOdometry();

std::shared_ptr<AsyncVelocityController<double, double>> intakeController = 
	AsyncVelControllerBuilder()
	.withMotor(13)//intake motor
	.build();

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::ADIDigitalOut arm('C');
pros::ADIDigitalOut clamp('B');
pros::ADIDigitalOut hang('A');
pros::Motor intake(13);

void printOdom()
{
	while (true)
	{
		std::cout << drive->getState().str()<< '\n';
		pros::delay(100);
	}
}

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	

	drive->setDefaultStateMode({StateMode::CARTESIAN});

	pros::Task odom(printOdom);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


void autonomous() 
{
	//chasing that bag or smth idk

	//MAXIMIZE RINGZ PATH MAIN PERMUTATION
	//setting initial position
	drive->setState({30.5_in,18.25_in, 180_deg});
	drive->driveToPoint({48_in, 48_in}, true, GOAL_OFFSET);
	drive->waitUntilSettled();//ensuring we are in position to clamp mogo 
	//CLAMP CODE TO GRAB-----------------------------------------------
	//clamp->waitUntilSettled();//ensuring mogo is secured ------------
	intakeController->controllerSet(1);//sets intake motor to full speed, scoring the pre-load.
	drive->driveToPoint({27.5_in, 68.5_in}, false, RING_OFFSET);//intake and score
	drive->driveToPoint({36_in, 60_in}, true, 0_in);//subject to change for optimization
	drive->driveToPoint({24_in, 48_in}, false, RING_OFFSET);//intake and score
	drive->driveToPoint({20.5_in, 68.5_in}, false, RING_OFFSET);//intake and score
	//the final movement is one of the following two intake-movements. Their consistencies must be compared
	drive->driveToPoint({0_in, 0_in}, false, RING_OFFSET);//intake and score
	//drive->driveToPoint({72_in, 24_in}, false, RING_OFFSET);//intake and score(top ring?)
	








	//3RD MOGO RUSH MAIN PERMUTATION
	//setting initial position
	drive->setState({9.5_in,18.25_in, 180_deg});
	//starting movement
	drive->driveToPoint({9.5_in, 48_in}, true, 0_in);//NO OFFSET, this is just the end of the straight segment, no object is being manipulated
	drive->driveToPoint({24_in, 72_in}, true, GOAL_OFFSET);//MUST BE REPLACED WITH A CURVE TO POINT-------------------
	drive->waitUntilSettled();//ensuring we are in position to clamp mogo 
	//CLAMP CODE TO GRAB-----------------------------------------------
	//clamp->waitUntilSettled();//ensuring mogo is secured ------------
	intakeController->controllerSet(1);//sets intake motor to full speed, scoring the pre-load.
	drive->driveToPoint({24_in, 48_in}, false, RING_OFFSET);//intake and score
	drive->driveToPoint({72_in, 24_in}, false, RING_OFFSET);//intake and score(TOP RING?)(kinda sketch)
	//drive->driveToPoint({--_in, --_in}, false, RING_OFFSET);//Move to finish pos.(not yet choosen)
	

	//SOLO AWP MAIN PERMUTATION
	drive->setState({30.5_in,18.25_in, 180_deg});
	//starting movement
	drive->driveToPoint({48_in, 48_in}, true, GOAL_OFFSET);
	drive->waitUntilSettled();
	//CLAMP CODE TO GRAB-----------------------------------------------
	//clamp->waitUntilSettled();//ensuring mogo is secured ------------
	intakeController->controllerSet(1);//sets intake motor to full speed, scoring the pre-load. 
	drive->driveToPoint({27.5_in, 68.5_in}, false, RING_OFFSET);//collect and score
	drive->driveToPoint({30_in, 60_in}, false, 0_in);//moving back to ensure we dont cross auton line. no object being manipulated, SuBJ  to change
	drive->driveToPoint({24_in, 48_in}, false, RING_OFFSET);//collect and score
	drive->driveToPoint({20.5_in, 68.5_in}, false, RING_OFFSET);//collect and score
	drive->driveToPoint({84_in, 36_in}, true, GOAL_OFFSET);//MUST BE REPLACED WITH A CURVE TO POINT-------------------
	drive->waitUntilSettled();//waiting until we reach the right position 
	//CLAMP CODE TO DROP-----------------------------------------------
	//clamp->waitUntilSetted();//ensuring clamp is dropped before we turn
	drive->driveToPoint({96_in, 48_in}, true, GOAL_OFFSET);
	drive->waitUntilSettled();
	//CLAMP CODE TO GRAB-----------------------------------------------
	//clamp->waitUntilSetted();//ensuring clamp is grabbed before we turn
	drive->driveToPoint({120_in, 48_in}, false, RING_OFFSET);
	drive->driveToPoint({72_in, 48_in}, false, 0_in);//ramming into the cage, subj to change 

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	while (true)
	{
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
				(pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
				(pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		cout<<

		drive->getModel()->arcade(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0,
					controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0);

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
		{
			clamp.set_value(clamp_bool);
			clamp_bool = !clamp_bool;
		}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			arm.set_value(arm_bool);
			arm_bool = !arm_bool;
		}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
		{
			hang.set_value(hang_bool);
			hang_bool = !hang_bool;
		}
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
			intake.move_velocity(-600);
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
			intake.move_velocity(600);
		else
			intake.move_velocity(0);
		pros::delay(10);
	}
}