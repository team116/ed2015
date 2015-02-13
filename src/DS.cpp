#include "WPILib.h"
#include "Ports.h"
#include "DS.h"
#include "Mobility.h"
#include "Log.h"
#include <CameraServer.h>
#include <cmath>
DS* DS::INSTANCE = NULL;
const int kCam0Button = 1;
const int kCam1Button = 2;

DS::DS() {
	mobility = Mobility::getInstance();
	manipulator = Manipulator::getInstance();
	log = Log::getInstance();

	main_joystick = Joystick::GetStickForPort(DSPorts::DRIVER_ONE_JOYSTICK);
	secondary_joystick = Joystick::GetStickForPort(DSPorts::DRIVER_TWO_JOYSTICK);
	IO_board_one = Joystick::GetStickForPort(DSPorts::DIGITAL_IO_BOARD);
	IO_board_two = Joystick::GetStickForPort(DSPorts::SECOND_IO_BOARD);

	/*server = CameraServer::GetInstance();
	 server->SetQuality(50);
	 server->StartAutomaticCapture("cam0");*/
	frameFrontCam = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
	frameBackCam = imaqCreateImage(IMAQ_IMAGE_RGB, 0);

	on_step = false;
	override = false;
	drive_type = false;
	drive_type_handled = false;
	turn_degrees = false;
	turn_degrees_handled = false;

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
	processCameras();

}

void DS::processMobility() {
	// secondary driver has overridden so that they can control movement
	// I'm halving all input because this is for precision
	// we might just remove this because the override button is a dumb idea
	if (override) {
		log->write(Log::TRACE_LEVEL, "%s\tIn override mode\n", Utils::getCurrentTime());
		mobility->setDirection(secondary_joystick->GetX() / 2.0, secondary_joystick->GetY() / 2.0);
		mobility->setRotationSpeed(secondary_joystick->GetTwist() / 2.0);
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

		float x = main_joystick->GetX(), y = main_joystick->GetY();
		// small deadzones for the x and y movement of the joystick
		x = fabs(x) < 0.05 ? 0 : x;
		y = fabs(y) < 0.05 ? 0 : y;
		mobility->setDirection(x, y);
		mobility->setRotationSpeed(main_joystick->GetTwist());
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
}

void DS::processManipulator() {
	// surface switch
	if (IO_board_one->GetRawButton(IOBoardOnePorts::STACK_ON_STEP_SWITCH)) {
		manipulator->setSurface(Manipulator::STEP);
	}
	else if (IO_board_one->GetRawButton(IOBoardOnePorts::STACK_ON_PLATFORM_SWITCH)) {
		manipulator->setSurface(Manipulator::SCORING_PLATFORM);
	}
	else {
		manipulator->setSurface(Manipulator::FLOOR);
	}

	// this assumes the max voltage for the flap position input to be 5
	// also this assumes that we'll be getting this as analog input instead of as a few digital inputs
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

	// lifter preset buttons
	if (IO_board_one->GetRawButton(IOBoardOnePorts::LIFTER_PRESET_0)) {
		manipulator->setTargetLevel(0);
	}
	else if (IO_board_one->GetRawButton(IOBoardOnePorts::LIFTER_PRESET_1)) {
		manipulator->setTargetLevel(1);
	}
	else if (IO_board_one->GetRawButton(IOBoardOnePorts::LIFTER_PRESET_2)) {
		manipulator->setTargetLevel(2);
	}
	else if (IO_board_one->GetRawButton(IOBoardOnePorts::LIFTER_PRESET_3)) {
		manipulator->setTargetLevel(3);
	}
	else if (IO_board_one->GetRawButton(IOBoardOnePorts::LIFTER_PRESET_4)) {
		manipulator->setTargetLevel(4);
	}
	else if (IO_board_one->GetRawButton(IOBoardOnePorts::LIFTER_PRESET_5)) {
		manipulator->setTargetLevel(5);
	}
	else if (IO_board_one->GetRawButton(IOBoardOnePorts::LIFTER_PRESET_6)) {
		manipulator->setTargetLevel(6);
	}
	else {
		// do nothing
	}

	// manual lifter control buttons
	if (IO_board_one->GetRawButton(IOBoardOnePorts::LIFTER_UP_BUTTON)) {
		manipulator->liftLifters(Manipulator::MOVING_UP);
	}
	else if (IO_board_one->GetRawButton(IOBoardOnePorts::LIFTER_DOWN_BUTTON)) {
		manipulator->liftLifters(Manipulator::MOVING_DOWN);
	}
	else {
		manipulator->liftLifters(Manipulator::NOT_MOVING);
	}

	// rake control buttons
	if (IO_board_one->GetRawButton(IOBoardTwoPorts::RAKES_UP_BUTTON)) {
		manipulator->liftRakes(true);
	}
	else if (IO_board_one->GetRawButton(IOBoardTwoPorts::RAKES_DOWN_BUTTON)) {
		manipulator->liftRakes(false);
	}

	// normal control of manipulator by driver two
	if (!override) {
		if (secondary_joystick->GetY() > 0.25) {
			manipulator->pushTote();
		}
		else if (secondary_joystick->GetY() < -0.25) {
			manipulator->pullTote();
		}

		manipulator->spinTote(secondary_joystick->GetTwist());
	}
}

void DS::processLEDS() {
	// lifter height indicators
	doLevelLEDS(manipulator->getLevel());

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
	bool frontCamLatched = false;
	bool backCamLatched = false;
	bool frontCamFirstTime = true;
	bool backCamFirstTime = true;
	if (IO_board_two->GetRawButton(IOBoardTwoPorts::FRONT_CAMERA_SELECT) || frontCamLatched) {
		if (backCamLatched) {
			if (StopCamera(1)) {
				StartCamera(0);
				frontCamFirstTime = false;
				backCamLatched = false;
			}

			if (frontCamFirstTime) {
				if (StartCamera(0)) {
					frontCamFirstTime = false;
				}
			}

			imaqError = IMAQdxGrab(sessionFrontCam, frameFrontCam, true, NULL);
			if (imaqError != IMAQdxErrorSuccess) {
				log->write(Log::ERROR_LEVEL, "front camera IMAQdxGrab error: %ld\n", (long) imaqError);
			}
			else {
				CameraServer::GetInstance()->SetImage(frameFrontCam);
			}
			backCamLatched = true;
		}
	}
	else if (IO_board_two->GetRawButton(IOBoardTwoPorts::BACK_CAMERA_SELECT) || backCamLatched) {
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

		imaqError = IMAQdxGrab(sessionBackCam, frameBackCam, true, NULL);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "back cam IMAQdxGrab error: %ld\n", (long) imaqError);
		}
		else {
			CameraServer::GetInstance()->SetImage(frameBackCam);
		}
		backCamLatched = true;
	}
}

bool DS::StopCamera(int cameraNum) {
	if (cameraNum == 1) {
		// stop image acquisition
		IMAQdxStopAcquisition(sessionBackCam);
		//the camera name (ex "cam0") can be found through the roborio web interface
		imaqError = IMAQdxCloseCamera(sessionBackCam);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "backcamera IMAQdxCloseCamera error: %ld\n", (long) imaqError);
			return false;
		}
	}
	else if (cameraNum == 0) {
		IMAQdxStopAcquisition(sessionFrontCam);
		imaqError = IMAQdxCloseCamera(sessionFrontCam);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "front camera IMAQdxCloseCamera error: %ld\n", (long) imaqError);
			return false;
		}
	}
	return true;
}

bool DS::StartCamera(int cameraNum) {
	if (cameraNum == 1) {
		imaqError = IMAQdxOpenCamera("backCam", IMAQdxCameraControlModeController, &sessionBackCam);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "back camera IMAQdxOpenCamera error: %ld\n", (long) imaqError);
			return false;
		}
		imaqError = IMAQdxConfigureGrab(sessionBackCam);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "back camera IMAQdxConfigureGrab error: %ld\n", (long) imaqError);
			return false;
		}
		// acquire images
		IMAQdxStartAcquisition(sessionBackCam);

	}
	else if (cameraNum == 0) {
		imaqError = IMAQdxOpenCamera("backCam", IMAQdxCameraControlModeController, &sessionFrontCam);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "back camera IMAQdxOpenCamera error: %ld\n", (long) imaqError);
			return false;
		}
		imaqError = IMAQdxConfigureGrab(sessionFrontCam);
		if (imaqError != IMAQdxErrorSuccess) {
			log->write(Log::ERROR_LEVEL, "front cam IMAQdxConfigureGrab error: %ld\n", (long) imaqError);
			return false;
		}
		// acquire images
		IMAQdxStartAcquisition(sessionFrontCam);

	}
	return true;
}

DS* DS::getInstance() {
	if (INSTANCE == NULL) {
		INSTANCE = new DS();
	}
	return INSTANCE;
}
