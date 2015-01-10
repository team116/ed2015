/*
 * Log.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: Lenovo
 */

#include "Log.h"

Log* Log::INSTANCE = NULL;

Log::Log()
{
	filename = generateLogFilename();
	log_file = fopen(filename, "w");
}

Log::~Log()
{

}

void Log::write(Log::debugLevelType debug_level, const char* str, ...)
{

}

char* Log::generateLogFilename()
{
	return 0;
}

Log* Log::getInstance()
{
	if (INSTANCE == NULL)
	{
		INSTANCE = new Log();
	}
	return INSTANCE;
}
