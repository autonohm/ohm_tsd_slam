/**
 * This file is part of the ONPOSIX library. It has been distributed under under the terms of GPL.
 * See http://onposix.sourceforge.net/ for more information.
 * A few modifications have been introduced in order to adapt it to the present library.
 */

#include <iostream>
#include <new>
#include <cstdlib>

#include "Logger.h"

#define  PRINT_LOG(msg) std::cerr << msg

namespace obvious
{

// Definition (and initialization) of static attributes
Logger* Logger::_m = 0;

#ifdef LOGGER_MULTITHREAD
pthread_mutex_t Logger::lock_ = PTHREAD_MUTEX_INITIALIZER;
inline void Logger::lock()
{
	pthread_mutex_lock(&lock_);
}

inline void Logger::unlock()
{
	pthread_mutex_unlock(&lock_);
}
#else
void Logger::lock(){}
void Logger::unlock(){}
#endif



/**
 * \brief Constructor.
 * It is a private constructor, called only by getInstance() and only the
 * first time. It is called inside a lock, so lock inside this method
 * is not required.
 * It only initializes the initial time. All configuration is done inside the
 * configure() method.
 */
Logger::Logger(): _configured(false)
{
  _t.reset();
}

/**
 * \brief Method to configure the logger. Called by the DEBUG_CONF() macro.
 * To make implementation easier, the old stream is always closed.
 * Then, in case, it is open again in append mode.
 * @param Name of the file used for logging
 * @param Configuration (i.e., log on file and on screen on or off)
 * @param Verbosity threshold for file
 * @param Verbosity threshold for screen
 */
void Logger::configure (const std::string&	outputFile,
			const loggerConf	configuration,
			const int		fileVerbosityLevel,
			const int		screenVerbosityLevel)
{
		Logger::lock();

		_fileVerbosityLevel = fileVerbosityLevel;
		_screenVerbosityLevel = screenVerbosityLevel;

		// Close the old stream, if needed
		if (_configuration&file_on)
			_out.close();

		// Compute a new file name, if needed
		if (outputFile != _logFile){
			std::ostringstream oss;
			time_t currTime;
			time(&currTime);
			struct tm *currTm = localtime(&currTime);
			oss << outputFile << "_" <<
					currTm->tm_mday << "_" <<
					currTm->tm_mon+1 << "_" <<
					(1900 + currTm->tm_year) << "_" <<
					currTm->tm_hour << "-" <<
					currTm->tm_min << "-" <<
					currTm->tm_sec << ".log";
			_logFile = oss.str().c_str();
		}

		// Open a new stream, if needed
		if (configuration&file_on)
			_out.open(_logFile.c_str(), std::ios::app);

		_configuration = configuration;
		_configured = true;

		Logger::unlock();
}

/**
 * \brief Destructor.
 * It only closes the file, if open, and cleans memory.
 */

Logger::~Logger()
{
	Logger::lock();
	if (_configuration&file_on)
		_out.close();
	delete _m;
	Logger::unlock();

}

/**
 * \brief Method to get a reference to the object (i.e., Singleton)
 * It is a static method.
 * @return Reference to the object.
 */
Logger& Logger::getInstance()
{
	Logger::lock();
	if (_m == 0)
	{
		_m = new Logger;
		LOGMSG_CONF("nofile.log", Logger::file_off|Logger::screen_on, DBG_ERROR, DBG_DEBUG);
	}
	Logger::unlock();
	return *_m;
}


/**
 * \brief Method used to print messages.
 * Called by the DEBUG() macro.
 * @param Priority of the message
 * @param Source file where the method has been called (set equal to __FILE__ by the DEBUG macro)
 * @param Source line where the method has been called (set equal to __LINE__ by the macro)
 * @param Message
 */
void Logger::print(const unsigned int verbosityLevel,
					const std::string& file,
					const int line,
					const std::string& message)
{
	std::stringstream logmsg;

	Logger::lock();
	switch(verbosityLevel)
	{
	case DBG_ERROR:
		logmsg << "ERROR ";
		break;
	case DBG_WARN:
		logmsg << "WARNING ";
		break;
	case DBG_DEBUG:
		logmsg << "DEBUG ";
		break;
	}
	logmsg << "[" << file << ":" << line << "] @ " << _t.elapsed() << "s :" << message << std::endl;

	if ((_configuration&file_on) && (verbosityLevel <= _fileVerbosityLevel))
	{
			_out << logmsg.str();
			_out.flush();
	}

	if ((_configuration&screen_on) && (verbosityLevel <= _screenVerbosityLevel))
			PRINT_LOG(logmsg.str().c_str());

	Logger::unlock();
}

} // namespace
