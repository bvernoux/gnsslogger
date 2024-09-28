/**
 * @file GNSSLogger.cpp
 * @brief Implementation of the GNSSLogger class for logging GNSS data from a serial port.
 * 
 * This file contains the implementation of the GNSSLogger class, which is responsible
 * for configuring the serial port, reading NMEA sentences, parsing the data, and
 * logging both raw and parsed GNSS data to files. It ensures thread-safe access to
 * the GNSS data through mutex protection and is designed to compile without warnings
 * on GCC with all warnings treated as errors.
 *
 * @author bvernoux@gmail.com
 */

#include "GNSSLogger.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

// ActualSerialPort implementation

bool ActualSerialPort::open(const std::string& port, int baudRate) {
#ifdef _WIN32
	m_hSerial = CreateFileA(port.c_str(), GENERIC_READ, 0, 0, OPEN_EXISTING, 0, 0);
	if (m_hSerial == INVALID_HANDLE_VALUE) {
		m_lastError = "Failed to open port: " + std::to_string(GetLastError());
		return false;
	}

	DCB dcbSerialParams;
	ZeroMemory(&dcbSerialParams, sizeof(DCB));
	dcbSerialParams.DCBlength = sizeof(DCB);
	if (!GetCommState(m_hSerial, &dcbSerialParams)) {
		m_lastError = "Failed to get comm state: " + std::to_string(GetLastError());
		CloseHandle(m_hSerial);
		return false;
	}

	dcbSerialParams.BaudRate = baudRate;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	dcbSerialParams.fParity = FALSE;
	dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;
	dcbSerialParams.fRtsControl = RTS_CONTROL_ENABLE;

	if (!SetCommState(m_hSerial, &dcbSerialParams)) {
		m_lastError = "Failed to set comm state: " + std::to_string(GetLastError());
		CloseHandle(m_hSerial);
		return false;
	}

	COMMTIMEOUTS timeouts;
	ZeroMemory(&timeouts, sizeof(COMMTIMEOUTS));
	timeouts.ReadIntervalTimeout = 50; // The maximum time allowed to elapse before the arrival of the next byte on the communications line, in milliseconds
	timeouts.ReadTotalTimeoutConstant = 50; // A constant used to calculate the total time-out period for read operations, in milliseconds
	timeouts.ReadTotalTimeoutMultiplier = 10; // The multiplier used to calculate the total time-out period for read operations, in milliseconds
	timeouts.WriteTotalTimeoutConstant = 50; // A constant used to calculate the total time-out period for write operations, in milliseconds
	timeouts.WriteTotalTimeoutMultiplier = 10; // The multiplier used to calculate the total time-out period for write operations, in milliseconds

	if (!SetCommTimeouts(m_hSerial, &timeouts)) {
		m_lastError = "Failed to set comm timeouts: " + std::to_string(GetLastError());
		CloseHandle(m_hSerial);
		return false;
	}
#else
	m_fd = ::open(port.c_str(), O_RDONLY | O_NOCTTY);
	if (m_fd < 0) {
		m_lastError = "Failed to open port: " + std::string(strerror(errno));
		return false;
	}

	struct termios tty;
	std::memset(&tty, 0, sizeof(tty));
	if (tcgetattr(m_fd, &tty) != 0) {
		m_lastError = "Failed to get terminal attributes: " + std::string(strerror(errno));
		::close(m_fd);
		return false;
	}

	cfsetispeed(&tty, baudRate);
	cfsetospeed(&tty, baudRate);

	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_cflag |= CREAD | CLOCAL;

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO;
	tty.c_lflag &= ~ECHOE;
	tty.c_lflag &= ~ECHONL;
	tty.c_lflag &= ~ISIG;

	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

	tty.c_oflag &= ~OPOST;
	tty.c_oflag &= ~ONLCR;

	tty.c_cc[VTIME] = 10;
	tty.c_cc[VMIN] = 0;

	if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
		m_lastError = "Failed to set terminal attributes: " + std::string(strerror(errno));
		::close(m_fd);
		return false;
	}
#endif
	return true;
}

void ActualSerialPort::close() {
#ifdef _WIN32
	if (m_hSerial != INVALID_HANDLE_VALUE) {
		CloseHandle(m_hSerial);
		m_hSerial = INVALID_HANDLE_VALUE;
	}
#else
	if (m_fd >= 0) {
		::close(m_fd);
		m_fd = -1;
	}
#endif
}

int ActualSerialPort::read(void* buffer, size_t count) {
#ifdef _WIN32
	DWORD bytesRead;
	if (!ReadFile(m_hSerial, buffer, static_cast<DWORD>(count), &bytesRead, NULL)) {
		DWORD error = GetLastError();
		m_lastError = "ReadFile failed: " + std::to_string(error);
		return (error == ERROR_TIMEOUT) ? 0 : -1;
	}
	return static_cast<int>(bytesRead);
#else
	int result = ::read(m_fd, buffer, count);
	if (result < 0) {
		m_lastError = "read failed: " + std::string(strerror(errno));
		return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
	}
	return result;
#endif
}

bool ActualSerialPort::setBufferSize(size_t size) {
#ifdef _WIN32
	return SetupComm(m_hSerial, static_cast<DWORD>(size), static_cast<DWORD>(size));
#else
	// Linux doesn't have a direct equivalent, so we'll just return true
	(void)size;
	return true;
#endif
}

bool ActualSerialPort::flush() {
#ifdef _WIN32
	return PurgeComm(m_hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
#else
	return tcflush(m_fd, TCIOFLUSH) == 0;
#endif
}

std::string ActualSerialPort::getLastErrorMessage() const {
	return m_lastError;
}

// GNSSLogger implementation

GNSSLogger::GNSSLogger()
	:	m_logLevel(LogLevel::Error), m_stopping(false),
	m_logRawData(false), m_logParsedData(false) {
	m_GNSSData.clear();
	m_currentGNSSData.clear();
	m_lastError = ErrorCode::NoError;
	m_baudRate = 9600;
	m_duration = -1;
	m_running = false;
}

GNSSLogger::~GNSSLogger() noexcept {
	stop();
}

GNSSLogger::ErrorCode GNSSLogger::start(std::unique_ptr<ISerialPort> serialPort,
										const std::string& port,
										const std::string& rawDataFilename,
										const std::string& parsedDataFilename,
										int baudRate,
										int duration) {
	std::lock_guard<std::mutex> lock(m_dataMutex);
	if (m_running) {
		setError(ErrorCode::InvalidParameter, "Logger is already running");
		return ErrorCode::InvalidParameter;
	}

	m_serialPort = std::move(serialPort);
	m_port = port;
	m_rawDataFilename = rawDataFilename;
	m_parsedDataFilename = parsedDataFilename;
	m_baudRate = baudRate;
	m_duration = duration;

	m_logRawData = !rawDataFilename.empty();
	m_logParsedData = !parsedDataFilename.empty();

	if (m_logRawData) {
		m_rawDataFile.open(m_rawDataFilename, std::ios::out | std::ios::binary);
		if (!m_rawDataFile.is_open()) {
			setError(ErrorCode::FileOpenError, "Failed to open raw data file");
			return ErrorCode::FileOpenError;
		}
	}

	if (m_logParsedData) {
		m_parsedDataFile.open(m_parsedDataFilename, std::ios::out);
		if (!m_parsedDataFile.is_open()) {
			if (m_logRawData) {
				m_rawDataFile.close();
			}
			setError(ErrorCode::FileOpenError, "Failed to open parsed data file");
			return ErrorCode::FileOpenError;
		}
	}

	if (!configureComPort()) {
		return ErrorCode::PortConfigError;
	}

	m_running = true;
	m_stopping = false;
	m_startTime = std::chrono::steady_clock::now();
	m_thread = std::thread(&GNSSLogger::loggerThread, this);

	return ErrorCode::NoError;
}

void GNSSLogger::stop() {
	if (!m_stopping.exchange(true)) {
		m_running = false;
		if (m_thread.joinable()) {
			m_thread.join();
		}
		if (m_serialPort) {
			m_serialPort->close();
		}
		if (m_logRawData) {
			m_rawDataFile.close();
			m_logRawData = false;
		}
		if (m_logParsedData) {
			m_parsedDataFile.close();
			m_logParsedData = false;
		}
	}
}

bool GNSSLogger::isRunning() const {
	return m_running;
}

GNSSLogger::ErrorCode GNSSLogger::getLastError() const {
	return m_lastError.load();
}

std::string GNSSLogger::getErrorMessage() {
	std::lock_guard<std::mutex> lock(m_errorMutex);
	return m_errorMessage;
}

void GNSSLogger::clearError() {
	std::lock_guard<std::mutex> lock(m_errorMutex);
	m_lastError = ErrorCode::NoError;
	m_errorMessage.clear();
}

bool GNSSLogger::setBufferSize(size_t size) {
	return m_serialPort->setBufferSize(size);
}

bool GNSSLogger::flush() {
	return m_serialPort->flush();
}

std::string GNSSLogger::getPortName() const {
	return m_port;
}

int GNSSLogger::getBaudRate() const {
	return m_baudRate;
}

void GNSSLogger::setLogLevel(LogLevel level) {
	std::lock_guard<std::mutex> lock(m_logMutex);
	m_logLevel = level;
}

void GNSSLogger::log(LogLevel level, const std::string& message) {
	(void)level;
	(void)message;
	/*
	std::lock_guard<std::mutex> lock(m_logMutex);
	if (level >= m_logLevel) {
		std::cout << "[" << logLevelToString(level) << "] " << message << std::endl;
	}
	*/
}

GNSSData GNSSLogger::getGNSSData() const {
	std::lock_guard<std::mutex> lock(m_dataMutex);
	return m_GNSSData;
}

GNSSData GNSSLogger::getCurrentGNSSData() const {
	std::lock_guard<std::mutex> lock(m_dataMutex);
	return m_currentGNSSData;
}

bool GNSSLogger::configureComPort() {
	if (m_serialPort)
	{
		if (!m_serialPort->open(m_port, m_baudRate)) {
			setError(ErrorCode::PortOpenError, "Failed to open serial port: " + m_serialPort->getLastErrorMessage());
			return false;
		}
	}
	else
	{
		setError(ErrorCode::PortOpenError, "Failed to open serial port: m_serialPort is null");
		return false;
	}
	return true;
}

void GNSSLogger::loggerThread() {
	constexpr size_t bufferSize = 256;
	char buffer[bufferSize];
	std::string sentence;
	int consecutiveErrors = 0;
	const int maxConsecutiveErrors = 10;  // Adjust as needed

	while (m_running) {
		int bytesRead = m_serialPort->read(buffer, bufferSize - 1);

		if (bytesRead > 0) {
			consecutiveErrors = 0;  // Reset error count on successful read
			buffer[bytesRead] = '\0';
			
			if (m_logRawData) {
				m_rawDataFile.write(buffer, bytesRead);
			}

			sentence.append(buffer, bytesRead);

			size_t pos;
			while ((pos = sentence.find('\n')) != std::string::npos) {
				std::string line = sentence.substr(0, pos);
				processNMEASentence(line);
				sentence.erase(0, pos + 1);
			}
		} else if (bytesRead == 0) {
			// No data available or timeout, wait a bit before trying again
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		} else {
			// Error occurred
			consecutiveErrors++;
			
			std::string errorMessage = m_serialPort->getLastErrorMessage();
			log(LogLevel::Warning, "Error reading from serial port: " + errorMessage);

			if (consecutiveErrors >= maxConsecutiveErrors) {
				setError(ErrorCode::ReadError, "Too many consecutive read errors");
				m_running = false;
				break;
			}

			// Wait a bit before retrying
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		if (m_duration > 0) {
			auto currentTime = std::chrono::steady_clock::now();
			auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(currentTime - m_startTime).count();
			if (elapsedSeconds >= m_duration) {
				m_running = false;
			}
		}
	}
}

void GNSSLogger::processNMEASentence(const std::string& sentence) {
	std::lock_guard<std::mutex> lock(m_dataMutex);
	m_currentGNSSData.nb_frames_received = m_currentGNSSData.nb_frames_received + 1;
	if (m_nmeaParser.parseNMEASentence(sentence, m_currentGNSSData) == true)
	{
		if (m_currentGNSSData.time != m_old_GNSSData_time) {
			m_GNSSData = m_currentGNSSData;
			m_old_GNSSData_time = m_currentGNSSData.time;
			if (m_logParsedData) {
				logParsedData();
			}
		}
	}
}

void GNSSLogger::logParsedData() {
	std::stringstream ss;
	ss << std::fixed << std::setprecision(6);
	ss << m_GNSSData.date << "," << m_GNSSData.time << ","
	   << m_GNSSData.latitude << "," << m_GNSSData.longitude << ","
	   << std::setprecision(1) << m_GNSSData.altitude << ","
	   << m_GNSSData.speed << "," << m_GNSSData.speedKmh << ","
	   << m_GNSSData.heading << "," << m_GNSSData.satellitesUsed << ","
	   << static_cast<int>(m_GNSSData.fixQuality) << ","
	   << static_cast<int>(m_GNSSData.fixType) << ","
	   << std::setprecision(2) << m_GNSSData.hdop << ","
	   << m_GNSSData.pdop << "," << m_GNSSData.vdop << ",";

	for (size_t i = 0; i < m_GNSSData.satellitePRNs.size(); ++i) {
		ss << m_GNSSData.satellitePRNs[i];
		if (i < m_GNSSData.satellitePRNs.size() - 1) {
			ss << ";";
		}
	}
	ss << "\n";

	m_parsedDataFile << ss.str();
	m_parsedDataFile.flush();
}

void GNSSLogger::setError(ErrorCode error, const std::string& message) {
	std::lock_guard<std::mutex> lock(m_errorMutex);
	m_lastError = error;
	m_errorMessage = message;
	log(LogLevel::Error, message);
}

std::string GNSSLogger::logLevelToString(LogLevel level) const {
	switch (level) {
		case LogLevel::Debug:   return "DEBUG";
		case LogLevel::Info:    return "INFO";
		case LogLevel::Warning: return "WARNING";
		case LogLevel::Error:   return "ERROR";
		default:                return "UNKNOWN";
	}
}