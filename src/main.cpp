#include "main.h"
#include "hbot.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"


std::unique_ptr<Robot> robot = nullptr;
std::unique_ptr<Controller> controller = Controller::create(pros::Controller(pros::E_CONTROLLER_MASTER));

constexpr int32_t FLYWHEEL_NORMAL_RPM = 2100;
constexpr int32_t FLYWHEEL_ANGLECHG_RPM = 2000;
constexpr int32_t FLYWHEEL_OVERFILL_RPM = 1900;

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
	robot->flywheel->move(FLYWHEEL_NORMAL_RPM);
	robot->flywheel->enable();
	robot->chassis->set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);

	int32_t flywheel_normal_rpm = FLYWHEEL_NORMAL_RPM;
	int32_t flywheel_anglechg_rpm = FLYWHEEL_ANGLECHG_RPM;
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
                robot->flywheel->move(flywheel_anglechg_rpm);
            } else {
                robot->flywheel->move(flywheel_normal_rpm);
            }
        }

		if(controller->newly_pressed(DIGITAL_X)) {
			toggle_overfill = !toggle_overfill;

			if (toggle_overfill) {
				flywheel_anglechg_rpm = FLYWHEEL_OVERFILL_RPM;
			} else {
				flywheel_anglechg_rpm = FLYWHEEL_ANGLECHG_RPM;
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

		if(controller->newly_pressed(DIGITAL_DOWN)) {
			robot->indexer->repeat(9, 1000, 100);
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
		PID::create(560, 0, 50, 0, 0, 20),//PID::create(490, 10, 45, 950, 0, 20),
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

void driver_matchloads() {
	robot->flywheel->enable();
	robot->flywheel->move(1900);
	pros::delay(3000);
	for(int i = 0; i < 8; i++) {
		if(controller->pressed(DIGITAL_Y)) {
			goto stop;
		}

		robot->indexer->index(100);
		pros::delay(1000);
	}
	robot->indexer->index();
	stop:
	{};
}

void auto_right() {
	// start flywheel
	robot->flywheel->enable();
	robot->flywheel->use_pidf();
	robot->flywheel->move(2400);
	
	// drive to roller
	robot->drive_dist_timeout(-50, 750, true);
	// turn to face roller
	robot->turn_to_angle(90);
	// drive into roller
	robot->drive_dist_timeout(-15, 1000, 5);
	// spin for 250ms
	robot->intake->move_for_voltage(250, 12000);
	// drive away
	robot->drive_dist(5);

	// turn to goal
	robot->turn_to_angle(107.5);
	// shoot 2 preloads
	robot->indexer->repeat(2, 1000, 100);
	// prepare lower flywheel velocity for next shots
	robot->flywheel->move(2230);
	
	
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

	robot->turn_to_angle(145);
	robot->indexer->repeat(3, 1000, 100);
	robot->flywheel->move(2250);

	// drive into bomerang
	robot->drive_dist_timeout(-25, 500, 7.5);

	// drive back
	robot->drive_to_point(57.4, 101.5);
	// turn to shoot
	robot->turn_to_angle(143);
	
	// shoot
	robot->indexer->repeat(3, 1000, 100);
}

void auto_right_special() {
	// start flywheel
	robot->flywheel->enable();
	robot->flywheel->use_pidf();
	robot->flywheel->move(2400);
	
	// drive to roller
	robot->drive_dist(-68,5, true);
	// turn to face roller
	robot->turn_to_angle(90);
	// drive into roller
	robot->drive_dist_timeout(-15, 750, 5);
	// spin for 250ms
	robot->intake->move_for_voltage(250, 12000);
	// drive away
	robot->drive_dist(5);

	// turn to goal
	robot->turn_to_angle(105.5);
	// shoot 2 preloads
	robot->indexer->repeat(3, 750, 100);
	// prepare lower flywheel velocity for next shots
	robot->flywheel->move(2240);
	
	
	// start intake
	robot->intake->move_voltage(12000);
	// turn to 3line
	robot->turn_to_angle(222);

	
	// drive and intake 3 line
	robot->chassis->set_voltage_percent(65);
	robot->drive_to_point(51.14, 107, true);
	robot->chassis->set_voltage_percent(100);

	// drive back
	robot->drive_dist(15);

	robot->turn_to_angle(145);
	robot->indexer->repeat(3, 750, 100);
	robot->flywheel->move(2250);

	// drive into bomerang
	robot->drive_dist_timeout(-6.5, 10000, 7.5);

/*
	// drive back
	robot->drive_to_point(75.4, 101.5, true);
	// turn to shoot
	robot->turn_to_angle(143);
	
	// shoot
	robot->indexer->repeat(3, 1000, 100);*/
}

void auto_skills() {
	robot->flywheel->enable();
	robot->flywheel->move(1800);
	
	
	//pros::delay(2750);

	//robot->indexer->repeat(9, 1000);

	// heading = 23.5	

	robot->drive_to_point(38.5, -1.5);

	robot->intake->move_voltage(12000);

	robot->turn_to_angle(17);

	robot->chassis->set_voltage_percent(80);
	robot->drive_to_point(-129, -52, true);
	robot->chassis->set_voltage_percent(100);


	
	//first roller - fixed
	robot->turn_to_angle(8);
	robot->drive_dist(-18, 5);
	pros::delay(200);
	robot->drive_dist(31);

	//second roller - fixed
	robot->turn_to_angle(-80);
	robot->drive_dist_timeout(-26, 1000, 5);
	pros::delay(250);

	robot->drive_to_point(-100, -121);

	robot->flywheel->move(1900);

	robot->indexer->repeat(1, 1000, 100); //shoot random + extra from match load

	robot->chassis->set_voltage_percent(45);
	robot->turn_to_angle(-210); //turn to intake line of 3
	robot->drive_to_point(11.8, -168.5, true); //intake line of 3

	robot->turn_to_angle(-111); //turn to shoot 3
	robot->indexer->repeat(3, 1000, 100);


	robot->drive_to_point(36, -101, true); //drive to intake first opp line of 3

	robot->drive_dist_timeout(-10, 2000);

	robot->turn_to_angle(-206.5); // align to intake opp line of 3

	robot->flywheel->move(1900); //speed up flywheel

	robot->drive_to_point(120.4, -143.8, true); // intake opp line of 3

	robot->drive_dist(-20);

	robot->turn_to_angle(-260); //line up to shoot 3

	robot->indexer->repeat(3, 1000, 100);

	robot->intake->move_voltage(0);

	robot->turn_to_angle(-206.5); //line up to go to roller
	robot->drive_dist(-75); //drive to roller

	robot->turn_to_angle(-160); //turn into roller

	robot->drive_dist(-6); //drive into roller

	robot->intake->move_voltage(12000); //run roller

	pros::delay(200);

	robot->drive_dist(45); //drive back from roller

	robot->turn_to_angle(-200); // turn to shoot endgame

	robot->endgame->fire();

}

void autonomous() {
	auto_skills();

}

void opcontrol() {
	driver_matchloads();
	drive_loop();

}
