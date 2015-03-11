#include "WPILib.h"
#include "Ports.h"
#include "DS.h"
#include "Mobility.h"
#include "Manipulator.h"
#include "Log.h"
#include "CameraFeeds.h"
#include <cmath>
DS* DS::INSTANCE = NULL;

DS::DS() {
	log = Log::getInstance();
	mobility = Mobility::getInstance();
	manipulator = Manipulator::getInstance();

	main_joystick = Joystick::GetStickForPort(DSPorts::DRIVER_ONE_JOYSTICK);
	secondary_joystick = Joystick::GetStickForPort(DSPorts::DRIVER_TWO_JOYSTICK);
	output_board = Joystick::GetStickForPort(DSPorts::OUTPUT_BOARD);
	input_board = Joystick::GetStickForPort(DSPorts::INPUT_BOARD);

	//camera_feeds = new CameraFeeds(input_board);
	// for testing purposes
	//camera_feeds =  new CameraFeeds(main_joystick);
	//camera_feeds->init();

	manual_lifter_stop_handled = false;
	on_step = false;
	danny_override = false;
	drive_type = false;
	drive_type_handled = false;
	turn_degrees = false;
	turn_degrees_handled = false;
	flip_orientation_handled = false;
	frontCamLatched = false;
	backCamLatched = false;
	frontCamFirstTime = true;
	backCamFirstTime = true;
	frontCamSelect = false;
	backCamSelect = true;
	toggle_rotation = false;
	toggle_cardinal = false;

	output_board->SetOutputs(0);


}

void DS::process() {

	if (secondary_joystick->GetRawButton(JoystickPorts::OVERRIDE_BUTTON)) {
		log->write(Log::INFO_LEVEL, "%s\tOverride button pressed\n", Utils::getCurrentTime());
		danny_override = !danny_override;
	}

	processMobility();
	processManipulator();
	processLEDS();
	//processCameras();
	//camera_feeds->run();

}

void DS::processMobility() {
	// secondary driver has overridden so that they can control movement
	// we might just remove this because the override button is a dumb idea
	toggle_rotation = secondary_joystick->GetRawButton(JoystickPorts::TOGGLE_ROTATION);
	toggle_cardinal = secondary_joystick->GetRawButton(JoystickPorts::CARDINAL_DIRECTION);
	if (danny_override) {
		log->write(Log::TRACE_LEVEL, "%s\tIn override mode\n", Utils::getCurrentTime());
		float x = secondary_joystick->GetX(), y = secondary_joystick->GetY(),
				t = secondary_joystick->GetRawAxis(2);
		// shaping is cubic because we want fine control
		x = fabs(x) < 0.1 ? 0 : x * fabs(x) * fabs(x);
		y = fabs(y) < 0.1 ? 0 : y * fabs(y) * fabs(y);
		t = fabs(t) < 0.1 ? 0 : t * fabs(t) * fabs(t);
		mobility->setDirection(x, y);
		if (toggle_rotation) {
			mobility->setRotationSpeed(t * 0.75);
		}
		else {
			mobility->setRotationSpeed(0.0);
		}
	}

	// normal control by first driver
	else {
		// check if the driver is trying to change to/from field-centric
		drive_type = main_joystick->GetRawButton(JoystickPorts::FIELD_CENTRIC_TOGGLE);

		if (drive_type && !drive_type_handled) {
			log->write(Log::INFO_LEVEL, "%s\tField-centric toggle pressed\n", Utils::getCurrentTime());
			drive_type_handled = true;
			mobility->toggleFieldCentric();
		}
		else if (drive_type_handled && !drive_type) {
			drive_type_handled = false;
		}

		toggle_rotation = main_joystick->GetRawButton(JoystickPorts::TOGGLE_ROTATION);
		toggle_cardinal = main_joystick->GetRawButton(JoystickPorts::CARDINAL_DIRECTION);
		float x = main_joystick->GetX(), y = main_joystick->GetY(),
				t = main_joystick->GetRawAxis(2);
		// shaping and deadzones
		x = fabs(x) < 0.1 ? 0 : x * fabs(x);
		y = fabs(y) < 0.1 ? 0 : y * fabs(y);
		t = fabs(t) < 0.1 ? 0 : t * fabs(t);
		if(toggle_cardinal) {
			if(fabs(x) > fabs(y)) {
				y = 0.0;
			}
			else if(fabs(y) > fabs(x)) {
				x = 0.0;
			}
		}
		mobility->setDirection(x, y);
		// the rotation is really fast, halve the speed
		if (toggle_rotation) {
			mobility->setRotationSpeed(t / 2.0);
		}
		else {
			mobility->setRotationSpeed(0.0);
		}
	}
	turn_degrees = main_joystick->GetRawButton(JoystickPorts::TURN_DEGREES);
	if (turn_degrees && !turn_degrees_handled) {
		log->write(Log::ERROR_LEVEL, "Starting turn Degrees\n");
		turn_degrees_handled = true;
		mobility->setRotationDegrees(90);
	}
	else if (!turn_degrees && turn_degrees_handled) {
		turn_degrees_handled = false;
	}

	if (main_joystick->GetRawButton(JoystickPorts::FLIP_ORIENTATION)) {
		if (!flip_orientation_handled) {
			flip_orientation_handled = true;
			mobility->flipOrientation();
		}
	}
	else {
		flip_orientation_handled = false;
	}
}

void DS::processManipulator() {

	// surface switch
	if (input_board->GetRawButton(InputBoardPorts::STACK_ON_STEP_SWITCH)) {
		manipulator->setSurface(Manipulator::STEP);
	}
	else if (input_board->GetRawButton(InputBoardPorts::STACK_ON_PLATFORM_SWITCH)) {
		manipulator->setSurface(Manipulator::SCORING_PLATFORM);
	}
	else {
		manipulator->setSurface(Manipulator::FLOOR);
	}

	// this assumes the max voltage for the flap position input to be 5
	// also this assumes that we'll be getting this as analog input instead of as a few digital inputs
	log->write(Log::TRACE_LEVEL, "%s\tFlap knob voltage: %f\n", Utils::getCurrentTime(), input_board->GetRawAxis(InputBoardPorts::FLAP_POSITION_KNOB));
	int knob_pos = Utils::convertFromVolts(input_board->GetRawAxis(InputBoardPorts::FLAP_POSITION_KNOB)+1, 6, 2.0);
	log->write(Log::TRACE_LEVEL, "%s\tFlap knob position: %i\n", Utils::getCurrentTime(), knob_pos);
	switch (knob_pos) {
	case 0:
	case 1:
		if(last_set_flap_position != Manipulator::FLAP_ANGLE_LOW){
			manipulator->setFlapPosition(Manipulator::FLAP_ANGLE_LOW);
			last_set_flap_position = Manipulator::FLAP_ANGLE_LOW;
		}
		break;
	case 2:
	case 3:
		if(last_set_flap_position != Manipulator::FLAP_ANGLE_MID){
			manipulator->setFlapPosition(Manipulator::FLAP_ANGLE_MID);
			last_set_flap_position = Manipulator::FLAP_ANGLE_MID;
		}
		break;
	case 4:
	case 5:
		if(last_set_flap_position != Manipulator::FLAP_ANGLE_HIGH){
			manipulator->setFlapPosition(Manipulator::FLAP_ANGLE_HIGH);
			last_set_flap_position = Manipulator::FLAP_ANGLE_HIGH;
		}
		break;
	}

	// lifter preset buttons
	if (input_board->GetRawButton(InputBoardPorts::LIFTER_PRESET_0)) {
		manipulator->setTargetLevel(0);
	}
	else if (input_board->GetRawButton(InputBoardPorts::LIFTER_PRESET_1)) {
		manipulator->setTargetLevel(1);
	}
	else if (input_board->GetRawButton(InputBoardPorts::LIFTER_PRESET_2)) {
		manipulator->setTargetLevel(2);
	}
	else if (input_board->GetRawButton(InputBoardPorts::LIFTER_PRESET_3)) {
		manipulator->setTargetLevel(3);
	}
	else if (input_board->GetRawButton(InputBoardPorts::LIFTER_PRESET_4)) {
		manipulator->setTargetLevel(4);
	}
	else if (input_board->GetRawButton(InputBoardPorts::LIFTER_PRESET_5)) {
		manipulator->setTargetLevel(5);
	}
	else if (input_board->GetRawButton(InputBoardPorts::LIFTER_PRESET_6)) {
		manipulator->setTargetLevel(6);
	}
	else {
		// do nothing
	}

	// manual lifter control buttons
	if (input_board->GetRawButton(InputBoardPorts::LIFTER_UP_BUTTON)) {
		manipulator->liftLifters(Manipulator::MOVING_UP);
		manual_lifter_stop_handled = false;
	}
	else if (input_board->GetRawButton(InputBoardPorts::LIFTER_DOWN_BUTTON)) {
		manipulator->liftLifters(Manipulator::MOVING_DOWN);
		manual_lifter_stop_handled = false;
	}
	else if (!manual_lifter_stop_handled) {
		manipulator->liftLifters(Manipulator::NOT_MOVING);
		manual_lifter_stop_handled = true;
	}

	// rake control buttons
	if (input_board->GetRawButton(InputBoardPorts::LEFT_RAKE_UP_BUTTON)) {
		manipulator->movePortRake(Manipulator::RAKE_LIFTING);
	}
	else if (input_board->GetRawButton(InputBoardPorts::LEFT_RAKE_DOWN_BUTTON)) {
		manipulator->movePortRake(Manipulator::RAKE_LOWERING);
	}
	else {
		manipulator->movePortRake(Manipulator::RAKE_STILL);
	}

	if (input_board->GetRawButton(InputBoardPorts::RIGHT_RAKE_UP_BUTTON)) {
		manipulator->moveStarboardRake(Manipulator::RAKE_LIFTING);
	}
	else if (input_board->GetRawButton(InputBoardPorts::RIGHT_RAKE_DOWN_BUTTON)) {
		manipulator->moveStarboardRake(Manipulator::RAKE_LOWERING);
	}
	else {
		manipulator->moveStarboardRake(Manipulator::RAKE_STILL);
	}

	// normal control of manipulator by driver two
	if (!danny_override) {
		manipulator->moveTote(secondary_joystick->GetY(), secondary_joystick->GetTwist());
/*		if (secondary_joystick->GetY() > 0.4) {
			manipulator->pushTote();
		}
		else if (secondary_joystick->GetY() < -0.4) {
			manipulator->pullTote();
		}

		float t = secondary_joystick->GetTwist();
		if(fabs(t) > .15){
			manipulator->spinTote(t*fabs(t));
		}*/
	}
}

void DS::processLEDS() {
	// lifter height indicators
	doLevelLEDS(manipulator->getLevel());

	/*
	 // selected stacking surface indicators
	 if (IO_board_one->GetRawButton(IOBoardOnePorts::STACK_ON_PLATFORM_SWITCH)) {
	 IO_board_one->SetOutput(IOBoardOnePorts::STACK_ON_FLOOR_INDICATOR, false);
	 IO_board_one->SetOutput(IOBoardOnePorts::STACK_ON_PLATFORM_INDICATOR, true);
	 IO_board_one->SetOutput(IOBoardOnePorts::STACK_ON_STEP_INDICATOR, false);
	 }
	 else if (IO_board_one->GetRawButton(IOBoardOnePorts::STACK_ON_STEP_SWITCH)) {
	 IO_board_one->SetOutput(IOBoardOnePorts::STACK_ON_FLOOR_INDICATOR, false);
	 IO_board_one->SetOutput(IOBoardOnePorts::STACK_ON_PLATFORM_INDICATOR, false);
	 IO_board_one->SetOutput(IOBoardOnePorts::STACK_ON_STEP_INDICATOR, true);
	 }
	 else {
	 IO_board_one->SetOutput(IOBoardOnePorts::STACK_ON_FLOOR_INDICATOR, true);
	 IO_board_one->SetOutput(IOBoardOnePorts::STACK_ON_PLATFORM_INDICATOR, false);
	 IO_board_one->SetOutput(IOBoardOnePorts::STACK_ON_STEP_INDICATOR, false);
	 }
	 // camera select indicators
	 if (IO_board_two->GetRawButton(IOBoardTwoPorts::BACK_CAMERA_SELECT)) {
	 IO_board_one->SetOutput(IOBoardOnePorts::BACK_CAMERA_INDICATOR, true);
	 IO_board_one->SetOutput(IOBoardOnePorts::FRONT_CAMERA_INDICATOR, false);
	 }
	 else if (IO_board_two->GetRawButton(IOBoardTwoPorts::FRONT_CAMERA_SELECT)) {
	 IO_board_one->SetOutput(IOBoardOnePorts::BACK_CAMERA_INDICATOR, false);
	 IO_board_one->SetOutput(IOBoardOnePorts::FRONT_CAMERA_INDICATOR, true);
	 }
	 */
}

void DS::doLevelLEDS(int level) {
	// turns on all leds at or below level, turns off the others
	switch (level) {
	case 0:
		output_board->SetOutput(LEDBoardPorts::LEVEL_0_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_1_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_2_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_3_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_4_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_5_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_6_INDICATOR, false);
		break;
	case 1:
		output_board->SetOutput(LEDBoardPorts::LEVEL_0_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_1_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_2_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_3_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_4_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_5_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_6_INDICATOR, false);
		break;
	case 2:
		output_board->SetOutput(LEDBoardPorts::LEVEL_0_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_1_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_2_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_3_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_4_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_5_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_6_INDICATOR, false);
		break;
	case 3:
		output_board->SetOutput(LEDBoardPorts::LEVEL_0_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_1_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_2_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_3_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_4_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_5_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_6_INDICATOR, false);
		break;
	case 4:
		output_board->SetOutput(LEDBoardPorts::LEVEL_0_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_1_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_2_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_3_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_4_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_5_INDICATOR, false);
		output_board->SetOutput(LEDBoardPorts::LEVEL_6_INDICATOR, false);
		break;
	case 5:
		output_board->SetOutput(LEDBoardPorts::LEVEL_0_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_1_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_2_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_3_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_4_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_5_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_6_INDICATOR, false);
		break;
	case 6:
		output_board->SetOutput(LEDBoardPorts::LEVEL_0_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_1_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_2_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_3_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_4_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_5_INDICATOR, true);
		output_board->SetOutput(LEDBoardPorts::LEVEL_6_INDICATOR, true);
		break;
	}
}
/*
void DS::processCameras() {

	 if (input_board->GetRawButton(InputBoardPorts::CAMERA_SELECT_SWITCH)) {
	 	 frontCamSelect = false;
	 	 backCamSelect = true;
	 }
	 else {
	 	 frontCamSelect = true;
	 	 backCamSelect = false;
	 }
	if (main_joystick->GetRawButton(JoystickPorts::TEMP_CAMERA_TOGGLE_TEST)) {
		log->write(Log::INFO_LEVEL, "Swapped cameras at %s\n", Utils::getCurrentTime());
		frontCamSelect = !frontCamSelect;
		backCamSelect = !backCamSelect;
	}

	if (frontCamSelect || frontCamLatched) {
		if (backCamLatched) {
			if (StopCamera(1)) {
				StartCamera(0);
				frontCamFirstTime = false;
				backCamLatched = false;
			}
		}

		if (frontCamFirstTime) {
			if (StartCamera(0)) {
				frontCamFirstTime = false;
			}
		}

		imaqError = IMAQdxGrab(sessionCam0, frameFrontCam, true, NULL);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "front camera IMAQdxGrab error: %ld\n", (long) imaqError);
		}
		else {
			server->SetImage(frameFrontCam);
			IMAQdxDispose(frameFrontCam);
		}
		backCamLatched = true;
	}

	else if (backCamSelect || backCamLatched) {
		if (frontCamLatched) {
			if (StopCamera(0)) {
				StartCamera(1);
				backCamFirstTime = false;
				frontCamLatched = false;
			}
		}

		if (backCamFirstTime) {
			if (StartCamera(1)) {
				backCamFirstTime = false;
			}

		}

		imaqError = IMAQdxGrab(sessionCam1, frameBackCam, true, NULL);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "back cam IMAQdxGrab error: %ld\n", (long) imaqError);
		}
		else {
			server->SetImage(frameBackCam);
			IMAQdxDispose(frameBackCam);
		}
		backCamLatched = true;
	}
}
bool DS::StartCamera(int cameraNum) {
	if (cameraNum == 0) {
		log->write(Log::INFO_LEVEL, "Starting cam 0");
		imaqError = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController, &sessionCam0);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "front camera IMAQdxOpenCamera error: %ld\n", (long) imaqError);
			return false;
		}
		else {
		 	 log->write(Log::INFO_LEVEL,"%sFront camera started",Utils::getCurrentTime());
		}
		imaqError = IMAQdxConfigureGrab(sessionCam0);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "front camera IMAQdxConfigureGrab error: %ld\n", (long) imaqError);
			return false;
		}
		// acquire images
		IMAQdxStartAcquisition(sessionCam0);
	}

	else if (cameraNum == 1) {
		imaqError = IMAQdxOpenCamera("cam1", IMAQdxCameraControlModeController, &sessionCam1);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "back camera IMAQdxOpenCamera error: %ld\n", (long) imaqError);
			return false;
		}
		else {
			log->write(Log::INFO_LEVEL,"%sBack camera started",Utils::getCurrentTime());
		}
		imaqError = IMAQdxConfigureGrab(sessionCam1);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "back camera IMAQdxConfigureGrab error: %ld\n", (long) imaqError);
			return false;
		}
		IMAQdxStartAcquisition(sessionCam1);

	}

	return true;
}

bool DS::StopCamera(int cameraNum) {
	if (cameraNum == 0) {
		IMAQdxStopAcquisition(sessionCam0);
		imaqError = IMAQdxCloseCamera(sessionCam0);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "front camera IMAQdxCloseCamera error: %ld\n", (long) imaqError);
			return false;
		}
	}

	else if (cameraNum == 1) {
		IMAQdxStopAcquisition(sessionCam1);
		imaqError = IMAQdxCloseCamera(sessionCam1);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "back camera IMAQdxCloseCamera error: %ld\n", (long) imaqError);
			return false;
		}
	}

	return true;
}
*/

DS* DS::getInstance() {
	if (INSTANCE == NULL) {
		INSTANCE = new DS();
	}
	return INSTANCE;
}
