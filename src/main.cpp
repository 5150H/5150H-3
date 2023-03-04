#include "main.h"
#include "hbot.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"


std::unique_ptr<Robot> robot = nullptr;
std::unique_ptr<Controller> controller = Controller::create(pros::Controller(pros::E_CONTROLLER_MASTER));


constexpr int32_t FLYWHEEL_NORMAL = 2100;
constexpr int32_t FLYWHEEL_ANGLECHG = 2000;
constexpr int32_t FLYWHEEL_OVERFILL = 1900;

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
	robot->chassis->set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);

	int32_t flywheel_normal = 2100;
	int32_t flywheel_anglechg = 2000;
	bool toggle_overfill = false;

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
                robot->flywheel->move(flywheel_normal);
            } else {
                robot->flywheel->move(flywheel_anglechg);
            }
        }

		if(controller->newly_pressed(DIGITAL_X)) {
			toggle_overfill = !toggle_overfill;

			if (toggle_overfill) {
				flywheel_anglechg = FLYWHEEL_OVERFILL;
			} else {
				flywheel_anglechg = FLYWHEEL_ANGLECHG;
			}
		}
		
        pros::delay(5);
    }
}

void fire_loop() {
	while (true) {
		if(controller->newly_pressed(DIGITAL_R1)) {
            if(controller->pressed(DIGITAL_R2)) {
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
		PID::create(500, 50, 80, 0, 0, 20),
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
		50, 250);

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
	// start flywheel
	robot->flywheel->enable();
	robot->flywheel->move(2575);
	robot->flywheel->use_pidf();

	// drive to roller
	robot->drive_to_point(-50, 0, true);
	// turn to face roller
	robot->turn_to_angle(90);
	// drive into roller
	robot->drive_dist(-11, 5);
	// spin for 250ms
	robot->intake->move_for_voltage(250, 12000);
	// drive away
	robot->drive_dist(5);

	// turn to goal
	robot->turn_to_angle(107.5);
	// shoot 2 preloads
	robot->indexer->repeat(2, 1000, 100);
	// prepare lower flywheel velocity for next shots
	robot->flywheel->move(2225);
	
	
	// start intake
	robot->intake->move_voltage(12000);
	// turn to 3line
	robot->turn_to_angle(222);

	
	// drive and intake 3 line
	robot->chassis->set_voltage_percent(65);
	robot->drive_to_point(69.69, 107, true);
	robot->chassis->set_voltage_percent(100);

	// drive back
	robot->drive_dist(15);

	robot->turn_to_angle(138);
	robot->indexer->repeat(3, 1000, 100);
	robot->flywheel->move(2275);

	// drive into bomerang
	robot->drive_dist(-25, 7.5);

	// drive back
	robot->drive_to_point(57.4, 101.5);
	// turn to shoot
	robot->turn_to_angle(139);
	
	// shoot
	robot->indexer->repeat(3, 1000, 100);
}

void auto_right_special() {
	// start flywheel
	robot->flywheel->enable();
	robot->flywheel->move(2575);
	robot->flywheel->use_pidf();

	// start intake
	robot->intake->move_voltage(12000);
	// drive to roller
	robot->drive_to_point(-68.5, 0, true);
	// turn to face roller
	robot->turn_to_angle(90);
	// drive into roller
	robot->drive_dist(-11, 5);
	// spin for 250ms
	robot->intake->move_for_voltage(250, 12000);
	// drive away
	robot->drive_dist(5);

	// turn to goal
	robot->turn_to_angle(107.5);
	// shoot 2 preloads
	robot->indexer->repeat(3, 1000, 100);
	// prepare lower flywheel velocity for next shots
	robot->flywheel->move(2225);
	
	// turn to 3line
	robot->intake->move_voltage(12000);
	robot->turn_to_angle(222);

	// drive and intake 3 line
	robot->chassis->set_voltage_percent(65);
	robot->drive_to_point(51.19, 107, true);
	robot->chassis->set_voltage_percent(100);

	// drive back
	robot->drive_dist(15);

	robot->turn_to_angle(138);
	robot->indexer->repeat(3, 1000, 100);
	robot->flywheel->move(2275);

	// drive into bomerang
	robot->drive_dist(-25, 7.5);

	// drive back
	robot->drive_to_point(38.9, 101.5);
	// turn to shoot
	robot->turn_to_angle(139);
	
	// shoot
	robot->indexer->repeat(3, 1000, 100);
}

void auto_skills() {
	
	robot->flywheel->enable();
	robot->flywheel->move(1800);
	
	pros::delay(3000);

	robot->indexer->repeat(9, 1000);

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
	robot->drive_dist(-16.5, 5);
	pros::delay(750);
	robot->drive_dist(30);

	robot->turn_to_angle(-88);
	robot->drive_dist(-37.5, 5);
	pros::delay(750);
	robot->drive_to_point(-112.5, -95);
	robot->indexer->repeat(3, 1000, 100);
	robot->turn_to_angle(-218);
	robot->chassis->set_voltage_percent(50);
	robot->drive_to_point(12, -199, true);
	
	robot->flywheel->move(1950);
	robot->turn_to_angle(-126);
	robot->indexer->repeat(3, 1000, 100);

	robot->turn_to_angle(-218);
	robot->drive_dist(16);
	robot->turn_to_angle(10);
	robot->chassis->set_voltage_percent(35);
	robot->drive_dist(-130);
	//robot->drive_to_point(-78, -199, true); odom boomerang
	
	//robot->drive_dist(-120);


	/* attempt to get second match loader
	robot->intake->move_voltage(0);
	robot->turn_to_angle(-83);
	
	robot->chassis->set_voltage_percent(75);
	robot->drive_to_point(26.7, -303);
	
	robot->turn_to_angle(-190.5);
	
	robot->drive_dist(-25, true);
	
	robot->turn_to_angle(203.5);
	robot->drive_dist(10);
	robot->turn_to_angle(-182.5);
	robot->drive_dist(-10);
	*/
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
	auto_skills();
}

void opcontrol() {
	drive_loop();
}
