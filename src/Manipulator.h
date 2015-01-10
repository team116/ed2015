#ifndef SRC_MANIPULATOR_H_
#define SRC_MANIPULATOR_H_

#include "WPILib.h"

class Manipulator {
public:
	Manipulator();
	virtual ~Manipulator();

	static Manipulator* getInstance();
	void process();
	void closeArms();
	void moveToHeight(int level);
	int getHeight();
	void openArms();
	void startConveyorBelt();
	void stopConveyorBelt();
private:
	static Manipulator* INSTANCE;

	int height;
	bool belt_moving;

	//motors n' stuff need to go here
};


#endif /* SRC_MANIPULATOR_H_ */
