#ifndef DS_H_
#define DS_H_

#include "Mobility.h"
#include "Manipulator.h"
#include "CameraFeeds.h"
#include <CameraServer.h>

class DS
{
public:
	static DS* getInstance();
	void process();
	void processMobility();
	void processManipulator();
	void processLEDS();
	void processCameras();
	void processCamerasButton(); // to be removed -- DO NOT KEEP, DO NOT SAVE
	bool StopCamera(int cameraNum);
	bool StartCamera(int cameraNum);
	bool StartFrontCamera();
	bool StartBackCamera();
	bool StopFrontCamera();
	bool StopBackCamera();

private:
	DS();
	static DS* INSTANCE;
	Mobility* mobility;
	Manipulator* manipulator;
	Log* log;
	CameraServer* server;
	CameraFeeds* camera_feeds;

	Joystick* main_joystick;
	Joystick* secondary_joystick;
	Joystick* output_board;
	Joystick* input_board;

	bool manual_lifter_stop_handled;
	bool danny_override;
	bool on_step;
	bool drive_type;
	bool drive_type_handled;
	bool turn_degrees;
	bool turn_degrees_handled;
	bool flip_orientation_handled;
	bool frontCamLatched;
	bool backCamLatched;
	bool frontCamFirstTime;
	bool backCamFirstTime;
	bool frontCamSelect;
	bool backCamSelect;
	bool toggle_rotation;
	bool toggle_cardinal;
	int last_set_flap_position;


	void doLevelLEDS(int level);

	const static float LIFTER_BUTTON_CHANGE;

	//cam0 is the front camera, cam1 is the back camera
	IMAQdxSession sessionCam0;
	IMAQdxSession sessionCam1;

	Image *frameFrontCam;
	Image *frameBackCam;

	IMAQdxError imaqError;

};

#endif // DS_H_
