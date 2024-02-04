/**
 * This file is part of the ONPOSIX library. It has been distributed under under the terms of GPL.
 * See http://onposix.sourceforge.net/ for more information.
 * A few modifications have been introduced in order to adapt it to the present library.
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <ostream>
#include <string>
#include <sstream>

/// Comment this line if you don't need multithread support
//#define LOGGER_MULTITHREAD

const int DBG_ERROR	= 0;
const int DBG_WARN	= 1;
const int DBG_DEBUG	= 2;


#ifdef LOGGER_MULTITHREAD
#include <pthread.h>
#endif

#include <string.h>

#include "Timer.h"

#define __LOGGERFILE__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

/**
 * \brief Macro to configure the logger.
 * Example of configuration of the Logger:
 * 	DEBUG_CONF("outputfile", Logger::file_on|Logger::screen_on, DBG_DEBUG, DBG_ERROR);
 */
#define LOGMSG_CONF(outputFile, \
		configuration, \
		fileVerbosityLevel, \
		screenVerbosityLevel) { \
			obvious::Logger::getInstance().configure(outputFile, \
						configuration, \
						fileVerbosityLevel, \
						screenVerbosityLevel); \
		}

/**
 * \brief Macro to print log messages.
 * Example of usage of the Logger:
 *	    DEBUG(DBG_DEBUG, "hello " << "world");
 */
#define LOGMSG(priority, msg) { \
	std::ostringstream __debug_stream__; \
	__debug_stream__ << msg; \
	obvious::Logger::getInstance().print(priority, __LOGGERFILE__, __LINE__, \
			__debug_stream__.str()); \
	}

namespace obvious
{

/**
 * \brief Simple logger to log messages on file and console.
 * This is the implementation of a simple logger in C++. It is implemented
 * as a Singleton, so it can be easily called through two DEBUG macros.
 * It is Pthread-safe.
 * It allows to log on both file and screen, and to specify a verbosity
 * threshold for both of them.
 */
class Logger
{
	/**
	 * \brief Type used for the configuration
	 */
	enum _loggerConf	{L_nofile_	 = 	1 << 0,
						 L_file_	 =	1 << 1,
						 L_noscreen_ =	1 << 2,
						 L_screen_	 =	1 << 3};

#ifdef LOGGER_MULTITHREAD
	/**
	 * \brief Lock for mutual exclusion between different threads
	 */
	static pthread_mutex_t lock_;
#endif

	bool _configured;

	/**
	 * \brief Pointer to the unique Logger (i.e., Singleton)
	 */
	static Logger* _m;

	/**
	 * \brief Initial part of the name of the file used for Logging.
	 * Date and time are automatically appended.
	 */
	std::string _logFile;

	/**
	 * \brief Current configuration of the logger.
	 * Variable to know if logging on file and on screen are enabled.
	 * Note that if the log on file is enabled, it means that the
	 * logger has been already configured, therefore the stream is
	 * already open.
	 */
	_loggerConf _configuration;

	/**
	 * \brief Stream used when logging on a file
	 */
	std::ofstream _out;

	/**
	 * \brief Verbosity threshold for files
	 */
	unsigned int _fileVerbosityLevel;

	/**
	 * \brief Verbosity threshold for screen
	 */
	unsigned int _screenVerbosityLevel;

	Logger();
	~Logger();

	/**
	 * \brief Method to lock in case of multithreading
	 */
	inline static void lock();

	/**
	 * \brief Method to unlock in case of multithreading
	 */
	inline static void unlock();

public:

	typedef _loggerConf loggerConf;
	static const loggerConf file_on= 	L_nofile_;
	static const loggerConf file_off= 	L_file_;
	static const loggerConf screen_on= 	L_noscreen_;
	static const loggerConf screen_off= L_screen_;

	static Logger& getInstance();

	void print(const unsigned int		verbosityLevel,
			const std::string&	sourceFile,
			const int 		codeLine,
			const std::string& 	message);

	void configure (const std::string&	outputFile,
			const loggerConf	configuration,
			const int		fileVerbosityLevel,
			const int		screenVerbosityLevel);
private:
	Timer _t;
};

inline Logger::loggerConf operator|
	(Logger::loggerConf __a, Logger::loggerConf __b)
{
	return Logger::loggerConf(static_cast<int>(__a) |
		static_cast<int>(__b));
}

inline Logger::loggerConf operator&
	(Logger::loggerConf __a, Logger::loggerConf __b)
{
	return Logger::loggerConf(static_cast<int>(__a) &
		static_cast<int>(__b)); }


} // namespace

#endif /* LOGGER_H */
