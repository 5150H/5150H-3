#include "main.h"
#include "hbot.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"


std::unique_ptr<Robot> robot = nullptr;
std::unique_ptr<Controller> controller = Controller::create(pros::Controller(pros::E_CONTROLLER_MASTER));



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
	robot->flywheel->move(2100);
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
                robot->flywheel->move(2000);
            } else {
                robot->flywheel->move(2100);
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

void initialize() {
	pros::lcd::initialize();
	pros::Task pl(print_loop);
	pros::Task fl(fire_loop);
	auto controllers = Controllers::create(
		PID::create(500, 0, 40, 0, 0, 20),
		PID::create(500, 0, 50, 0, 0, 20),
		PID::create(0, 0, 0, 0, 0, 20),
		Odom::create(
			pros::Rotation(8), pros::Rotation(6, true), 
			2.75 * INCH_TO_CM, 5.25 * INCH_TO_CM)
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
		100, 300);

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

void shoot() {
	robot->indexer->extend();
	pros::delay(100);
	robot->indexer->retract();

}

void auto_left() {

}

void auto_right() {
	robot->flywheel->enable();
	robot->flywheel->move(2650);

	robot->drive_to_point(-50, 0, true);
	robot->turn_to_angle(90);
	robot->drive_dist(-10);
	robot->intake->move_for_voltage(500, 12000);
	robot->drive_dist(5);

	robot->turn_to_angle(109);
	robot->indexer->repeat(2, 1000);
	robot->flywheel->move(2200);
	robot->intake->move_voltage(12000);

	robot->turn_to_angle(220);

	robot->chassis->set_voltage_percent(50);
	robot->drive_to_point(76, 124.5, true);
	robot->chassis->set_voltage_percent(100);

	robot->drive_dist(10);
	robot->turn_to_angle(132.5);
	robot->indexer->repeat(3, 1000);

	robot->drive_dist(-20);
	pros::delay(1000);
	robot->indexer->index();
}

void auto_right_special() {
	robot->flywheel->enable();
	robot->flywheel->move(2600);

	robot->intake->move_voltage(12000);
	robot->drive_to_point(-75, 0, true);
	robot->turn_to_angle(90);
	robot->drive_dist(-10);
	robot->intake->move_for_voltage(500, 12000);
	robot->drive_dist(5);

	robot->turn_to_angle(109);
	robot->indexer->repeat(3, 1000);
	robot->flywheel->move(2300);
	robot->intake->move_voltage(12000);

	robot->turn_to_angle(225);
	robot->chassis->set_voltage_percent(50);
	robot->drive_to_point(42, 112, true);
	robot->chassis->set_voltage_percent(100);

	robot->drive_dist(10);
	robot->turn_to_angle(132.5);
	robot->indexer->repeat(3, 1000);

	robot->drive_dist(-40);
	pros::delay(1000);
	robot->indexer->index();
}

void auto_skills() {
	/*
	robot->flywheel->enable();
	robot->flywheel->move(2000);
	
	pros::delay(3000);

	robot->indexer->repeat(9, 1000);*/

	// heading = 23.5

	/*
	robot->chassis->turn_voltage(12000);
	pros::delay(500);
	robot->chassis->stop();*/

	robot->drive_to_point(38.5, -1.5);

	robot->intake->move_voltage(12000);

	robot->turn_to_angle(15);

	robot->chassis->set_voltage_percent(50);
	robot->drive_to_point(-129, -52, true);
	robot->chassis->set_voltage_percent(100);

	robot->turn_to_angle(0);
	robot->drive_dist(-30);
	pros::delay(750);
	robot->drive_dist(40);

	robot->turn_to_angle(-88);
	robot->drive_dist(-30);
	pros::delay(750);
	robot->drive_dist(40);
}

void auto_skills_old() {
	/*
	robot->flywheel->enable();
	robot->flywheel->move(2000);
	pros::delay(3000);

	robot->indexer->repeat(9, 1000);*/

	robot->drive_to_point(147.5, 0);

	robot->drive_to_point(151, -238.25, true);
	
}

void autonomous() {
	auto_right();
}

void opcontrol() {
	drive_loop();
}
