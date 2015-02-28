#include "WPILib.h"

#ifndef CAMERAFEEDS_H_
#define CAMERAFEEDS_H_

class CameraFeeds {
public:
	//const int kBtCamFront = 1;
	//const int CamBackButton = 2;
	CameraFeeds(Joystick *newJoy);
	~CameraFeeds();

	void init();
	void run();
	void end();
	void changeCam(int newId);
	void updateCam();

private:
	IMAQdxSession camFront;
	IMAQdxSession camBack;
	IMAQdxSession curCam;
	Image* frame;
	CameraServer* server;
	const char* camNameFront = "cam0";
	const char* camNameBack = "cam1";
	int imgQuality = 60;
	Joystick* controller;

};
#endif /* CAMERAFEEDS_H_ */
