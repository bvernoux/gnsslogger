/**
 * @file GNSSLogger.hpp
 * @brief Declaration of the GNSSLogger class for logging GNSS data from a serial port.
 * 
 * This file contains the declaration of the GNSSLogger class, which is responsible
 * for configuring the serial port, reading NMEA sentences, parsing the data, and
 * logging both raw and parsed GNSS data to files. It ensures thread-safe access to
 * the GNSS data through mutex protection and is designed to compile without warnings
 * on GCC with all warnings treated as errors.
 *
 * @author bvernoux@gmail.com
 */

#ifndef GNSS_LOGGER_HPP
#define GNSS_LOGGER_HPP

#include <memory>
#include <string>
#include <fstream>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
#include "NMEAParser.hpp"
#include "GNSSUtility.hpp"

#ifdef _WIN32
#include <windows.h>
#else
#include <termios.h>
#endif

class ISerialPort {
public:
	virtual ~ISerialPort() = default;
	virtual bool open(const std::string& port, int baudRate) = 0;
	virtual void close() = 0;
	virtual int read(void* buffer, size_t count) = 0;
	virtual bool setBufferSize(size_t size) = 0;
	virtual bool flush() = 0;
	virtual std::string getLastErrorMessage() const = 0;
};

class ActualSerialPort : public ISerialPort {
public:
	bool open(const std::string& port, int baudRate) override;
	void close() override;
	int read(void* buffer, size_t count) override;
	bool setBufferSize(size_t size) override;
	bool flush() override;
	std::string getLastErrorMessage() const override;

private:
	std::string m_lastError;
#ifdef _WIN32
	HANDLE m_hSerial;
#else
	int m_fd;
#endif
};

class GNSSLogger {
public:
	enum class ErrorCode {
		NoError,
		PortOpenError,
		PortConfigError,
		FileOpenError,
		ReadError,
		WriteError,
		InvalidParameter,
		UnknownError
	};

	enum class LogLevel {
		Debug,
		Info,
		Warning,
		Error
	};

	GNSSLogger();
	~GNSSLogger() noexcept;

	ErrorCode start(std::unique_ptr<ISerialPort> serialPort,
					const std::string& port,
					const std::string& rawDataFilename,
					const std::string& parsedDataFilename,
					int baudRate,
					int duration);
	void stop();
	bool isRunning() const;

	ErrorCode getLastError() const;
	std::string getErrorMessage();
	void clearError();

	bool setBufferSize(size_t size);
	bool flush();
	std::string getPortName() const;
	int getBaudRate() const;

	void setLogLevel(LogLevel level);
	void log(LogLevel level, const std::string& message);

	GNSSData getGNSSData() const;
	GNSSData getCurrentGNSSData() const;

private:
	bool configureComPort();
	void loggerThread();
	void processNMEASentence(const std::string& sentence);
	void logParsedData();
	void setError(ErrorCode error, const std::string& message);
	std::string logLevelToString(LogLevel level) const;

	std::string m_port;
	std::string m_rawDataFilename;
	std::string m_parsedDataFilename;
	int m_baudRate;
	int m_duration;
	std::atomic<bool> m_running;
	std::atomic<ErrorCode> m_lastError;
	std::string m_errorMessage;
	std::mutex m_errorMutex;
	mutable std::mutex m_dataMutex;
	std::mutex m_logMutex;
	std::ofstream m_rawDataFile;
	std::ofstream m_parsedDataFile;
	std::thread m_thread;
	NMEAParser m_nmeaParser;
	GNSSData m_GNSSData;
	GNSSData m_currentGNSSData;
	std::string m_old_GNSSData_time;
	std::chrono::steady_clock::time_point m_startTime;
	LogLevel m_logLevel;

	std::unique_ptr<ISerialPort> m_serialPort;
	std::atomic<bool> m_stopping;
	bool m_logRawData;
	bool m_logParsedData;
};

#endif // GNSS_LOGGER_HPP
