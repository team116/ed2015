/*
 * CameraFeeds.cpp
 *
 *  Created on: Feb 28, 2015
 *      Author: robbie
 */

#include "CameraFeeds.h"
#include "Ports.h"
#include <WPILib.h>

CameraFeeds::CameraFeeds(Joystick* newJoy) {
	int imaqError;
	imaqError = IMAQdxOpenCamera(camNameFront, IMAQdxCameraControlModeController, &camFront);
	if (imaqError != IMAQdxErrorSuccess) {
		DriverStation::ReportError("IMAQdxOpenCamera error: " + std::to_string((long) imaqError) + "\n");
	}
	imaqError = IMAQdxOpenCamera(camNameBack, IMAQdxCameraControlModeController, &camBack);
	if (imaqError != IMAQdxErrorSuccess) {
		DriverStation::ReportError("IMAQdxOpenCamera error: " + std::to_string((long) imaqError) + "\n");
	}
	curCam = camFront;
	frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
	server = CameraServer::GetInstance();
	server->SetQuality(imgQuality);
	controller = newJoy;
}

CameraFeeds::~CameraFeeds() {

}
void CameraFeeds::init() {
	changeCam(camFront);
}

void CameraFeeds::end() {
	IMAQdxStopAcquisition(curCam);
}

void CameraFeeds::changeCam(int newId) {
	int imaqError;
	IMAQdxStopAcquisition(curCam);
	imaqError = IMAQdxConfigureGrab(newId);
	if (imaqError != IMAQdxErrorSuccess) {
		DriverStation::ReportError("IMAQdxConfigureGrab error: " + std::to_string((long) imaqError) + "\n");
	}
	IMAQdxStartAcquisition(newId);
	curCam = newId;
}

void CameraFeeds::updateCam() {
	int imaqError;
	imaqError = IMAQdxGrab(curCam, frame, true, NULL);
	if (imaqError != IMAQdxErrorSuccess) {
		DriverStation::ReportError("IMAQdxGrab error: " + std::to_string((long) imaqError) + "\n");
	}
	server->SetImage(frame);

}

void CameraFeeds::run() {
	//if (controller->GetRawButton(1)) {
	if(controller->GetRawButton(InputBoardPorts::CAMERA_SELECT_SWITCH)){
		changeCam(camBack);
	}
	else {
		changeCam(camFront);
	}
	updateCam();
}

