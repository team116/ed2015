#include "WPILib.h"
#include "Ports.h"
#include "DS.h"
#include "Mobility.h"
#include "Manipulator.h"
#include "Log.h"
#include <CameraServer.h>
#include <cmath>
DS* DS::INSTANCE = NULL;

DS::DS() {
	mobility = Mobility::getInstance();
	manipulator = Manipulator::getInstance();
	log = Log::getInstance();

	main_joystick = Joystick::GetStickForPort(DSPorts::DRIVER_ONE_JOYSTICK);
	secondary_joystick = Joystick::GetStickForPort(DSPorts::DRIVER_TWO_JOYSTICK);
	IO_board_one = Joystick::GetStickForPort(DSPorts::DIGITAL_IO_BOARD);
	IO_board_two = Joystick::GetStickForPort(DSPorts::SECOND_IO_BOARD);

	server = CameraServer::GetInstance();
	server->SetQuality(5);
	frameFrontCam = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
	frameBackCam = imaqCreateImage(IMAQ_IMAGE_RGB, 0);

	on_step = false;
	override = false;
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

	IO_board_one->SetOutputs(0);

}

void DS::process() {

	if (secondary_joystick->GetRawButton(JoystickPorts::OVERRIDE_BUTTON)) {
		log->write(Log::INFO_LEVEL, "%s\tOverride button pressed\n", Utils::getCurrentTime());
		override = !override;
	}

	processMobility();
	processManipulator();
	processLEDS();
	//processCameras();

}

void DS::processMobility() {
	// secondary driver has overridden so that they can control movement
	// we might just remove this because the override button is a dumb idea
	if (override) {
		log->write(Log::TRACE_LEVEL, "%s\tIn override mode\n", Utils::getCurrentTime());
		float x = secondary_joystick->GetX(), y = secondary_joystick->GetY(), t = secondary_joystick->GetRawAxis(2);
		// shaping is cubic because we want fine control
		x = fabs(x) < 0.1 ? 0 : x * fabs(x) * fabs(x);
		y = fabs(y) < 0.1 ? 0 : y * fabs(y) * fabs(y);
		t = fabs(t) < 0.1 ? 0 : t * fabs(t) * fabs(t);
		mobility->setDirection(x, y);
		mobility->setRotationSpeed(t * 0.75);
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

		float x = main_joystick->GetX(), y = main_joystick->GetY(), t = main_joystick->GetRawAxis(2);
		// shaping and deadzones
		x = fabs(x) < 0.1 ? 0 : x * fabs(x);
		y = fabs(y) < 0.1 ? 0 : y * fabs(y);
		t = fabs(t) < 0.1 ? 0 : t * fabs(t);
		mobility->setDirection(x, y);
		// the rotation is really fast, halve the speed
		mobility->setRotationSpeed(t / 2.0);
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
	/*
	if (IO_board_one->GetRawButton(IOBoardOnePorts::STACK_ON_STEP_SWITCH)) {
		manipulator->setSurface(Manipulator::STEP);
	}
	else if (IO_board_one->GetRawButton(IOBoardOnePorts::STACK_ON_PLATFORM_SWITCH)) {
		manipulator->setSurface(Manipulator::SCORING_PLATFORM);
	}
	else {
		manipulator->setSurface(Manipulator::FLOOR);
	}
	*/

	// this assumes the max voltage for the flap position input to be 5
	// also this assumes that we'll be getting this as analog input instead of as a few digital inputs
	/*
	switch (Utils::convertFromVolts(IO_board_one->GetRawAxis(IOBoardOnePorts::FLAP_POSITION_KNOB), 3, 5.0)) {
	case 0:
		manipulator->setFlapPosition(Manipulator::FLAP_LOW);
		break;
	case 1:
		manipulator->setFlapPosition(Manipulator::FLAP_MID);
		break;
	case 2:
		manipulator->setFlapPosition(Manipulator::FLAP_HIGH);
		break;
	}
	*/

	if (IO_board_two->GetRawButton(IOBoardTwoPorts::FLAP_POSITION_CLOSE)) {
		manipulator->setFlapPosition(Manipulator::FLAP_LOW);
	}
	else if (IO_board_two->GetRawButton(IOBoardTwoPorts::FLAP_POSTITION_OPEN)) {
		manipulator->setFlapPosition(Manipulator::FLAP_HIGH);
	}
	else {
		manipulator->setFlapPosition(Manipulator::FLAP_MID);
	}

	// lifter preset buttons
	if (IO_board_one->GetRawButton(IOBoardTwoPorts::LIFTER_PRESET_0)) {
		manipulator->setTargetLevel(0);
	}
	else if (IO_board_one->GetRawButton(IOBoardTwoPorts::LIFTER_PRESET_1)) {
		manipulator->setTargetLevel(1);
	}
	else if (IO_board_one->GetRawButton(IOBoardTwoPorts::LIFTER_PRESET_2)) {
		manipulator->setTargetLevel(2);
	}
	else if (IO_board_one->GetRawButton(IOBoardTwoPorts::LIFTER_PRESET_3)) {
		manipulator->setTargetLevel(3);
	}
	else if (IO_board_one->GetRawButton(IOBoardTwoPorts::LIFTER_PRESET_4)) {
		manipulator->setTargetLevel(4);
	}
	else if (IO_board_one->GetRawButton(IOBoardTwoPorts::LIFTER_PRESET_5)) {
		manipulator->setTargetLevel(5);
	}
	else if (IO_board_one->GetRawButton(IOBoardTwoPorts::LIFTER_PRESET_6)) {
		manipulator->setTargetLevel(6);
	}
	else {
		// do nothing
	}

	// manual lifter control buttons
	if (IO_board_one->GetRawButton(IOBoardTwoPorts::LIFTER_UP_BUTTON)) {
		manipulator->liftLifters(Manipulator::MOVING_UP);
	}
	else if (IO_board_one->GetRawButton(IOBoardTwoPorts::LIFTER_DOWN_BUTTON)) {
		manipulator->liftLifters(Manipulator::MOVING_DOWN);
	}
	else {
		manipulator->liftLifters(Manipulator::NOT_MOVING);
	}

	// rake control buttons
	if (IO_board_one->GetRawButton(IOBoardTwoPorts::LEFT_RAKE_UP_BUTTON)) {
		manipulator->movePortRake(Manipulator::RAKE_LIFTING);
	}
	else if (IO_board_one->GetRawButton(IOBoardTwoPorts::LEFT_RAKE_DOWN_BUTTON)) {
		manipulator->movePortRake(Manipulator::RAKE_LOWERING);
	}
	else {
		manipulator->movePortRake(Manipulator::RAKE_STILL);
	}

	if(IO_board_one->GetRawButton(IOBoardTwoPorts::RIGHT_RAKE_UP_BUTTON)){
		manipulator->moveStarboardRake(Manipulator::RAKE_LIFTING);
	}
	else if(IO_board_one->GetRawButton(IOBoardTwoPorts::RIGHT_RAKE_DOWN_BUTTON)){
		manipulator->moveStarboardRake(Manipulator::RAKE_LOWERING);
	}
	else {
		manipulator->moveStarboardRake(Manipulator::RAKE_STILL);
	}

	// normal control of manipulator by driver two
	if (!override) {
		if (secondary_joystick->GetY() > 0.4) {
			manipulator->pushTote();
		}
		else if (secondary_joystick->GetY() < -0.4) {
			manipulator->pullTote();
		}

		float t = secondary_joystick->GetTwist();
		t = fabs(t) < 0.1 ? 0 : t * fabs(t);
		manipulator->spinTote(t);
	}
}

/*
void DS::processManipulator()
{
	if (IO_board_two->GetRawButton(IOBoardTwoPorts::FLAP_POSITION_CLOSE)) {
		manipulator->closeFlaps(true);
		manipulator->setFlapPosition(Manipulator::flap_position::FLAP_LOW);
	}
	else if (IO_board_two->GetRawButton(IOBoardTwoPorts::FLAP_POSTITION_OPEN)) {
		manipulator->closeFlaps(false);
		manipulator->setFlapPosition(Manipulator::FLAP_HIGH);
	}
	else {
		manipulator->setFlapPosition(Manipulator::FLAP_MID);
	}

	if (IO_board_two->GetRawButton(IOBoardTwoPorts::LIFTER_PRESET_0)) {
		manipulator->setTargetLevel(0);
	}
	else if (IO_board)
}
*/

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
	*/
	/*
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
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_0_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_1_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_2_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_3_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_4_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_5_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_6_INDICATOR, false);
		break;
	case 1:
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_0_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_1_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_2_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_3_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_4_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_5_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_6_INDICATOR, false);
		break;
	case 2:
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_0_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_1_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_2_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_3_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_4_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_5_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_6_INDICATOR, false);
		break;
	case 3:
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_0_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_1_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_2_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_3_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_4_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_5_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_6_INDICATOR, false);
		break;
	case 4:
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_0_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_1_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_2_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_3_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_4_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_5_INDICATOR, false);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_6_INDICATOR, false);
		break;
	case 5:
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_0_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_1_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_2_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_3_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_4_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_5_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_6_INDICATOR, false);
		break;
	case 6:
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_0_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_1_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_2_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_3_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_4_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_5_INDICATOR, true);
		IO_board_one->SetOutput(IOBoardOnePorts::LEVEL_6_INDICATOR, true);
		break;
	}
}

void DS::processCameras() {

	/*if (IO_board_two->GetRawButton(IOBoardTwoPorts::FRONT_CAMERA_SELECT)) {
		frontCamSelect = true;
		backCamSelect = false;
	}
	else if (IO_board_two->GetRawButton(IOBoardTwoPorts::BACK_CAMERA_SELECT)) {
		frontCamSelect = false;
		backCamSelect = true;
	}
	// if, somehow, neither switch is on, we'll use the front camera
	else {
		frontCamSelect = false;
		backCamSelect = true;
	}*/

	if(main_joystick->GetRawButton(JoystickPorts::TEMP_CAMERA_TOGGLE_TEST)){
		log->write(Log::INFO_LEVEL,"Swapped cameras at %s\n",Utils::getCurrentTime());
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
		log->write(Log::INFO_LEVEL,"Starting cam 0");
		imaqError = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController, &sessionCam0);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "front camera IMAQdxOpenCamera error: %ld\n", (long) imaqError);
			return false;
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


DS* DS::getInstance() {
	if (INSTANCE == NULL) {
		INSTANCE = new DS();
	}
	return INSTANCE;
}
