/*
 * Log.cpp
 *
 *  Created on: Jan 10, 2015
 *      Author: Lenovo
 */

#include "Log.h"
#include <stdio.h>
#include <stdarg.h>
#include <time.h>


Log* Log::INSTANCE = NULL;

Log::Log()
{
	char* filepath = generateLogFilename();
	log_file = fopen(filepath, "w");
	time_t current_time;
	time(&current_time);
	fprintf(log_file,"Began logging at %s\n",ctime(&current_time));
	fflush(log_file);
}

Log::~Log()
{
	if(log_file){
		time_t end_time;
		time(&end_time);
		fprintf(log_file,"Stopped logging at %s\n", ctime(&end_time));
		fprintf(log_file,"--------------------------------------------------\n");
		fclose(log_file);
	}
}

const Log::debugLevelType MAXIMUM_DEBUG_LEVEL = Log::TRACE_LEVEL;
//this can be set to the least urgent type of message that we want to log
//for the moment it's at trace so we'll log EVERYTHING

void Log::write(Log::debugLevelType debug_level, const char* str, ...)
{
	if(log_file && (debug_level<=MAXIMUM_DEBUG_LEVEL)){
		char buffer[256];
		va_list args;

		va_start(args,str);
		vsprintf(buffer,str,args);
		va_end(args);

		fputs(buffer,log_file);
		fflush(log_file);
	}
}

char* Log::generateLogFilename()
{
	const char base[] = "log";
	const char ext[] = ".txt";
	FILE* previous_log_file;

	int file_num = 0;

	while(true){
		sprintf(filename,"/home/lvuser/%s%d%s",base,file_num,ext);

		previous_log_file = fopen(filename,"r");
		if(!previous_log_file){
			break;
		}
		else{
			fclose(previous_log_file);
			file_num++;
		}
	}

	printf("Log File Name = %s\n", filename);
	return filename;
}

Log* Log::getInstance()
{
	if (INSTANCE == NULL)
	{
		INSTANCE = new Log();
	}
	return INSTANCE;
}
