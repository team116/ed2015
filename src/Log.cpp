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
	char* filepath = generateLogFilename();
	log_file = fopen(filepath, "w");
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
