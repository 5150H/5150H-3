#include "main.h"
#include "hbot.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"



/*
void customTurn(Robot* robot, double degrees) {
    double kb = 0; // 2.1

    double setpoint = degrees;
    double kp = 0.5;
	double kd = 20;

    unsigned long last = 0;
    double error = setpoint;
    double deriv = 0;

    double prev_error = error;

    while (true) {
        error = setpoint - robot->odom.heading();
        deriv = (error - prev_error) / 20.0;

        double output = error * kp + kd * deriv + std::copysign(kb, error);
        
        robot->chassis.turn(output);

        if (pros::millis() > last + 50) {
            std::cout << "error: " << error << ", deriv: " << deriv << ", voltage: "<< output << "\n";
			last = pros::millis();
        }
   
        prev_error = error;
        pros::delay(20);
    } 

    robot->chassis.stop();
    std::cout << "Heading: " << robot->odom.heading() << "\n";
}*/

std::unique_ptr<Robot> robot = nullptr;
std::unique_ptr<Controller> controller = Controller::create(pros::Controller(pros::E_CONTROLLER_MASTER));

void initialize() {
	pros::lcd::initialize();

	auto controllers = Controllers::create(
		PID::create(500, 0, 40, 0, 0, 20),
		PID::create(175, 0, 13.5, 0, 0, 20),
		PID::create(0, 0, 0, 0, 0, 20),
		Odom::create(
			pros::Rotation(8), pros::Rotation(6, true), 
			2.75 * INCH_TO_CM, 5.3 * INCH_TO_CM)
	);

	auto chassis = Chassis::create(
		{19, -17, -18}, 
		{-12,11, 13},
		1, 5);

	auto intake = Intake::create(
		{-1});

	auto flywheel = Flywheel::create(
		{-10},
		pros::Rotation(9),
		PID::create(10.5, 0, 0, 800, 3, 10)
		);

	auto indexer = Indexer::create(
		pros::ADIDigitalOut('B'), 
		50, 350);

	auto anglechg = Anglechg::create(
		pros::ADIDigitalOut('A'));

	auto endgame = Endgame::create(
		pros::ADIDigitalOut('C'));

	robot = Robot::create(
		std::move(chassis),
		std::move(controllers),
		std::move(intake),
		std::move(flywheel),
		std::move(indexer),
		std::move(anglechg),
		std::move(endgame));
}

void print_loop() {
	while (true) {
		auto pos = robot->controllers->odom->position();
		auto rpm = robot->flywheel->rpm();

		pros::lcd::print(0, "X: %f cm", pos.x);
		pros::lcd::print(1, "Y: %f cm", pos.y);
		pros::lcd::print(2, "Heading: %f degrees", pos.heading);
		pros::lcd::print(4, "Flywheel: %f RPM", rpm);

		pros::delay(20);
	}
}

void drive_loop() {
	robot->flywheel->move(2000);
	robot->flywheel->enable();

	while (true) {
        auto turn = controller->analog(ANALOG_RIGHT_X);
        auto power = controller->analog(ANALOG_LEFT_Y);
        robot->chassis->move_joystick(power, turn);

        if (controller->pressed(DIGITAL_R1) && 
			controller->pressed(DIGITAL_R2) && 
			controller->pressed(DIGITAL_L1) && 
			controller->pressed(DIGITAL_L2)) {
            robot->endgame->fire();
        }

        if(controller->pressed(DIGITAL_L1)) {
            robot->intake->move_voltage(12000);
        } else if (controller->pressed(DIGITAL_L2)) {
            robot->intake->move_voltage(-12000);
        } else {
            robot->intake->move_voltage(0);
        }
        
        if(controller->newly_pressed(DIGITAL_A)) {
            robot->flywheel->toggle();
			
        }

        if(controller->newly_pressed(DIGITAL_B)) {
            robot->anglechg->toggle();

            if(robot->anglechg->toggled()) {
                robot->flywheel->move(1900);
            } else {
                robot->flywheel->move(2000);
            }
        }

        pros::delay(5);
    }
}

void fire_loop() {
	while (true) {
		if(controller->newly_pressed(DIGITAL_R1)) {
            if(controller->newly_pressed(DIGITAL_R2)) {
                robot->indexer->index();
            } else {
                robot->indexer->repeat(3);
            }
        }
		pros::delay(5);
	}
}

void autonomous() {
	pros::Task fl(fire_loop);
	pros::Task pl(print_loop);

	
}

void opcontrol() {
	pros::Task fl(fire_loop);
	pros::Task pl(print_loop);
	drive_loop();
}
