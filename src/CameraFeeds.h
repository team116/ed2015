#include "WPILib.h"

#ifndef CAMERAFEEDS_H_
#define CAMERAFEEDS_H_

#include "Log.h"

class CameraFeeds {
public:
	//const int kBtCamFront = 1;
	//const int kBtCamBack = 2;
	CameraFeeds(Joystick *newJoy);
	~CameraFeeds();

	void init();
	void run();
	void end();
	void changeCam(int newId);
	void updateCam();

private:
	Log* log;

	IMAQdxSession camFront;
	IMAQdxSession camBack;
	IMAQdxSession curCam;
	Image* frame;
	CameraServer* server;
	const char* camNameFront = "cam0";
	const char* camNameBack = "cam1";
	int imgQuality = 40;
	Joystick* controller;

};
#endif /* CAMERAFEEDS_H_ */
