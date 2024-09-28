#include "GNSSLogger.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <condition_variable>
#include <map>
#include <string>

std::atomic<bool> g_running(true);
std::condition_variable g_cv;
std::mutex g_mutex;

void signal_handler(int signal) {
	(void)signal;
	g_running = false;
	g_cv.notify_all();
}

void clearConsole() {
#ifdef _WIN32
	system("cls");
#else
	std::cout << "\033[2J\033[1;1H";
#endif
}

void displayLiveData(const GNSSData& data) {
	std::cout << "Date Time: " << data.date << " " << data.time << "\n";
	std::cout << std::fixed << std::setprecision(6);
	std::cout << "Latitude:  " << data.latitude << "°\n";
	std::cout << "Longitude: " << data.longitude << "°\n";
	std::cout << std::setprecision(1);
	std::cout << "Altitude:  " << data.altitude << " m\n";
	std::cout << "Speed:     " << data.speed << " knots (" << data.speedKmh << " km/h)\n";
	std::cout << "Heading:   " << data.heading << "°\n";
	std::cout << "Satellites used: " << data.satellitesUsed << "\n";
	std::cout << "Fix Quality: " << std::to_string(data.fixQuality) << " (" << GNSSUtility::fixQualityToString(data.fixQuality) << ")\n";
	std::cout << "Fix Type:    " << std::to_string(data.fixType) << " (" << GNSSUtility::fixTypeToString(data.fixType) << ")\n";
	std::cout << std::setprecision(2);
	std::cout << "HDOP: " << data.hdop << " PDOP: " << data.pdop << " VDOP: " << data.vdop << " Nb PRNs: " << data.satellitePRNs.size() << "\n";

	std::cout << "Status: " << (data.isValid ? "\033[1;32mValid Fix\033[0m" : "\033[1;31mNo Fix\033[0m") << "\n";
	if(!data.isValid)
		std::cout << "nb_frames_received: " << data.nb_frames_received << "\n";

	std::cout << "\nPress Ctrl+C to stop logging.\n";
}

void printUsage(const char* programName) {
	std::cout << "Usage: " << programName << " <port> [options]\n"
			  << "Options:\n"
			  << "  --raw_out_file=<filename>    Specify the raw output file\n"
			  << "  --parsed_out_file=<filename> Specify the parsed output file\n"
			  << "  --duration_seconds=<seconds> Specify the duration in seconds\n"
			  << "  --baudrate=<rate>            Specify the baud rate\n";
}

std::map<std::string, std::string> parseArguments(int argc, char* argv[]) {
	std::map<std::string, std::string> args;
	for (int i = 1; i < argc; ++i) {
		std::string arg = argv[i];
		size_t pos = arg.find('=');
		if (pos != std::string::npos) {
			std::string key = arg.substr(0, pos);
			std::string value = arg.substr(pos + 1);
			args[key] = value;
		} else {
			args[arg] = "";
		}
	}
	return args;
}

int main(int argc, char* argv[]) {
	if (argc < 2) {
		printUsage(argv[0]);
		return 1;
	}

	std::string port = argv[1];
	auto args = parseArguments(argc - 1, argv + 1);

	std::string rawOutputFile = args["--raw_out_file"];
	std::string parsedOutputFile = args["--parsed_out_file"];
	int durationSeconds = -1;
	int baudRate = 9600;

	if (args.count("--duration_seconds")) {
		try {
			durationSeconds = std::stoi(args["--duration_seconds"]);
		} catch (const std::exception& e) {
			(void)e;
			std::cerr << "Invalid duration_seconds value: " << args["--duration_seconds"] << std::endl;
			return 1;
		}
	}

	if (args.count("--baudrate")) {
		try {
			baudRate = std::stoi(args["--baudrate"]);
		} catch (const std::exception& e) {
			(void)e;
			std::cerr << "Invalid baudrate value: " << args["--baudrate"] << std::endl;
			return 1;
		}
	}

	// Set up signal handling
	std::signal(SIGINT, signal_handler);
#ifdef _WIN32
	std::signal(SIGBREAK, signal_handler);
#else
	std::signal(SIGTERM, signal_handler);
#endif

	GNSSLogger logger;
	auto serialPort = std::unique_ptr<ActualSerialPort>(new ActualSerialPort());

	GNSSLogger::ErrorCode startResult = logger.start(std::move(serialPort), 
													 port, 
													 rawOutputFile, 
													 parsedOutputFile, 
													 baudRate, 
													 durationSeconds);
	if (startResult != GNSSLogger::ErrorCode::NoError) {
		std::cerr << "Failed to start GNSS logging: " << logger.getErrorMessage() << std::endl;
		return 1;
	}

	std::cout << "Starting GNSS logging...\n";

	auto start_time = std::chrono::steady_clock::now();
	std::atomic<bool> timeout_reached(false);

	std::thread displayThread([&logger, &timeout_reached, durationSeconds, start_time]() {
		std::string old_time;

		while (g_running && !timeout_reached) {
			if (!logger.isRunning()) break;

			auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
				std::chrono::steady_clock::now() - start_time).count();
			if (durationSeconds > 0 && elapsed >= durationSeconds) {
				timeout_reached = true;
				g_running = false;
				g_cv.notify_all();
				break;
			}

			GNSSData data = logger.getGNSSData();
			if (!data.isValid) {
				clearConsole();
				std::cout << "\033[1;36m=== GNSS Logger Live Data Waiting valid data(Fix)===\033[0m\n";
				displayLiveData(data);
				std::this_thread::sleep_for(std::chrono::milliseconds(900));
			} else {
				if (data.time != old_time) {
					old_time = data.time;
					clearConsole();
					std::cout << "\033[1;36m=== GNSS Logger Live Data ===\033[0m\n";
					displayLiveData(data);
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			}
		}
	});

	// Wait for the logger to finish, for an interrupt, or for the timeout
	{
		std::unique_lock<std::mutex> lock(g_mutex);
		g_cv.wait(lock, [&]() { return !g_running || !logger.isRunning() || timeout_reached.load(); });
	}

	// Stop the logger
	logger.stop();

	// Wait for the display thread to finish
	displayThread.join();

	// Display final data
	GNSSData finalData = logger.getGNSSData();
	clearConsole();
	std::cout << "\033[1;36m=== Final GNSS Logger Data ===\033[0m\n";
	displayLiveData(finalData);

	std::cout << "Logging completed.\n";
	if (timeout_reached) {
		std::cout << "Duration limit reached. Exiting...\n";
	}
	if (!rawOutputFile.empty()) {
		std::cout << "Raw data saved to: " << rawOutputFile << "\n";
	}
	if (!parsedOutputFile.empty()) {
		std::cout << "Parsed data saved to: " << parsedOutputFile << "\n";
	}

	if (logger.getLastError() != GNSSLogger::ErrorCode::NoError) {
		std::cerr << "Error occurred: " << logger.getErrorMessage() << "\n";
		return 1;
	}

	return 0;
}
