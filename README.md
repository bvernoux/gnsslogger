# GNSSLogger

GNSSLogger is a cross-platform C++ project designed to capture, parse, and log NMEA GPS/GNSS data from a serial port (COM port).

It provides real-time GNSS data processing, logging, and monitoring capabilities, and is compatible with Windows and GNU/Linux.

## Features

- Capture raw NMEA data from a specified COM port
- Parse NMEA sentences (GNRMC, GNGGA, GNGSA)
- Real-time GNSS data processing and validation
- Configurable baud rate and logging duration
- Raw data logging to file
- Parsed data logging with human-readable format
- Cross-platform support (Windows and GNU/Linux)
- NMEA coordinate conversion to decimal degrees
- Distance calculation between two GNSS coordinates
- NMEA checksum validation

## Requirements

- C++11 compatible compiler
- CMake 3.10 or higher
- Windows or GNU/Linux

## Building

1. Clone the repository:
   ```
   git clone https://github.com/bvernoux/gnsslogger.git
   cd gnsslogger
   ```

2. Create a build directory and run CMake:
For Windows (Visual Studio 2022):
   ```
   rm -rf build_VS2022
   mkdir build_VS2022 && cd build_VS2022
   cmake -G "Visual Studio 17 2022" -A x64 -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release ..
   ```
For Windows MinGW64:
   ```
   rm -rf build_MinGW64
   mkdir build_MinGW64 && cd build_MinGW64
   cmake -G "MinGW Makefiles" -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release ..
   ```
For GNU/Linux:
   ```
   rm -rf build
   mkdir build && cd build
   cmake -DBUILD_TESTS=ON ..
   cmake -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release ..
   ```

3. Build the project:
For Windows (Visual Studio 2022):
Open the sln

For MinGW64 and GNU/Linux:
   ```
   cmake --build .
   ```

   For a specific build type (e.g., Debug):
   ```
   cmake --build . --config Debug
   ```

## Usage

Run the gnsslogger executable with the following arguments:

```
gnsslogger <port> [options]
Options:
  --raw_out_file=<filename>    Specify the raw output file
  --parsed_out_file=<filename> Specify the parsed output file
  --duration_seconds=<seconds> Specify the duration in seconds
  --baudrate=<rate>            Specify the baud rate
```

### Parameters

- **`<port>`**: The name of the serial port (e.g., "COM3" on Windows, "/dev/ttyACM0" on Linux).

### Options

- **`--raw_out_file=<filename>`**: Specify the raw output file where NMEA data will be saved.
  
- **`--parsed_out_file=<filename>`**: Specify the parsed output file where processed GNSS data will be saved (csv format).

- **`--duration_seconds=<seconds>`**: Specify the duration of logging in seconds. Use `-1` for continuous logging (default: `-1`).

- **`--baudrate=<rate>`**: Specify the baud rate for the serial port (default: `9600`).

## Examples

Windows:
```
gnsslogger.exe COM3 --raw_out_file=raw_output.txt --parsed_out_file=parsed_output.csv --duration_seconds=300 --baudrate=115200
```

GNU/Linux Ubuntu:
```
./gnsslogger /dev/ttyACM0 --raw_out_file=raw_output.txt --parsed_out_file=parsed_output.csv --duration_seconds=300 --baudrate=115200
```

## Output

- options: 
  - Raw NMEA sentences are saved to the specified raw output file.
  - Parsed GNSS data is saved to the specified parsed output file in a csv format.
- Real-time status updates are displayed in the console, including GPS/GNSS date/time, current position, fix quality ...

Example Real-time console display (refreshed each second)
```
Date Time: 280924 194612
Latitude:  xx.xxxxxx°
Longitude: xx.xxxxxx°
Altitude:  420.0 m
Speed:     0.0 knots (0.0 km/h)
Heading:   0.0°
Satellites used: 12
Fix Quality: 1 (Standard)
Fix Type:    3 (3D)
HDOP: 0.66 PDOP: 1.26 VDOP: 1.08 Nb PRNs: 17
Status: Valid Fix
```

## Additional Features GNSS Utility Class (GNSSUtility.cpp / GNSSUtility.hpp)

- **`nmeaToDecimalDegrees()`**: Converts NMEA coordinates (in `DDMM.MMMM` or `DDDMM.MMMM` format) to decimal degrees. It supports directions for both latitude and longitude.
- **`calculateDistance()`**: Calculates the distance between two geographic points (latitude and longitude) using the Haversine formula.
- **`calculateBearing()`**: Calculates the initial bearing (forward azimuth) between two points (latitude and longitude) on Earth.
- **`validateNMEAChecksum()`**: Validates the checksum of an NMEA sentence to ensure data integrity.
- **`fixTypeToString()`**: Converts an integer representing the GNSS fix type to a corresponding string (e.g., "NA", "2D", "3D").
- **`fixQualityToString()`**: Converts an integer representing the GNSS fix quality to a corresponding string (e.g., "Invalid", "DGPS", "PPS fix").

## Contributing

Contributions to GNSSLogger are welcome! 

Please feel free to submit pull requests, create issues, or suggest improvements.

## Testing

To build and run tests (if implemented):

```
cmake -DBUILD_TESTS=OFF ..
cmake --build .
ctest
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

