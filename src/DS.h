#ifndef DS_H_
#define DS_H_

class DS
{
public:
	static DS* getInstance();

private:
	DS();
	static DS* INSTANCE;

};

#endif // DS_H_