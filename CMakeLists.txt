cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(gnsslogger VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add an option for building tests
option(BUILD_TESTS "Build the tests" OFF)

# Set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Output directory for binaries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Add include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add source files
set(SOURCES
    src/GNSSLogger.cpp
    src/GNSSUtility.cpp
    src/NMEAParser.cpp
)

# Add header files
set(HEADERS
    include/GNSSLogger.hpp
    include/GNSSUtility.hpp
    include/NMEAParser.hpp
)

set(GNSSLOGGER_LIB gnssloggerlib)

# Create a static library
add_library(${GNSSLOGGER_LIB} STATIC ${SOURCES} ${HEADERS})

# Explicitly set the linker language to CXX
set_target_properties(${GNSSLOGGER_LIB} PROPERTIES LINKER_LANGUAGE CXX)

# Add the main executable
add_executable(${PROJECT_NAME} src/main.cpp)

# Link the main executable with the library
target_link_libraries(${PROJECT_NAME} PRIVATE ${GNSSLOGGER_LIB})

# Platform-specific settings
if(WIN32)
    # Windows-specific settings
    target_compile_definitions(${GNSSLOGGER_LIB} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
    target_compile_definitions(${PROJECT_NAME} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
    
    # Set architecture to x64 for MSVC
    if(MSVC)
        set(CMAKE_GENERATOR_PLATFORM x64)
    endif()
    
    # MinGW specific flags
    if(MINGW)
        target_compile_options(${GNSSLOGGER_LIB} PRIVATE -Wall -Wextra -pedantic)
        target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)
    endif()
else()
    # Unix-like specific settings
    target_compile_options(${GNSSLOGGER_LIB} PRIVATE -Wall -Wextra -pedantic)
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)
    
    # Link against real-time library on Linux
    if(UNIX AND NOT APPLE)
        target_link_libraries(${PROJECT_NAME} PRIVATE rt)
    endif()
endif()

# MSVC-specific settings
if(MSVC)
    # Use static runtime for both release and debug configurations
    foreach(flag_var
            CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
            CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE)
        if(${flag_var} MATCHES "/MD")
            string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
        endif()
        if(${flag_var} MATCHES "/MDd")
            string(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
        endif()
    endforeach()

    # Disable specific warnings for MSVC
    target_compile_options(${GNSSLOGGER_LIB} PRIVATE /wd4996)
    target_compile_options(${PROJECT_NAME} PRIVATE /wd4996)
else()
    # Enable static linking for non-MSVC compilers
    target_link_options(${PROJECT_NAME} PRIVATE -static)
endif()

# Set optimization flags for Release build
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(MSVC)
        # Optimization for Release build
        target_compile_options(${GNSSLOGGER_LIB} PRIVATE "$<$<CONFIG:Release>:/O2>")
        target_compile_options(${PROJECT_NAME} PRIVATE "$<$<CONFIG:Release>:/O2>")
    else()
        # Optimization for Release build
        target_compile_options(${GNSSLOGGER_LIB} PRIVATE "$<$<CONFIG:Release>:-O3>")
        target_compile_options(${PROJECT_NAME} PRIVATE "$<$<CONFIG:Release>:-O3>")
        # Strip for non-MSVC compilers in Release mode
        target_link_options(${PROJECT_NAME} PRIVATE "$<$<CONFIG:Release>:-s>")
    endif()
    
    # Strip for non-MSVC compilers in Release mode
    if(NOT MSVC)
        target_link_options(${PROJECT_NAME} PRIVATE -s)
    endif()
endif()

# Enable warnings
if(MSVC)
    target_compile_options(${GNSSLOGGER_LIB} PRIVATE /W4 /WX)
    target_compile_options(${PROJECT_NAME} PRIVATE /W4 /WX)
else()
    target_compile_options(${GNSSLOGGER_LIB} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

# Add tests if BUILD_TESTS is ON
if(BUILD_TESTS)
    # Check if the tests directory exists and contains a CMakeLists.txt file
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
        enable_testing()
        add_subdirectory(tests)
    else()
        message(WARNING "Tests are enabled but the tests directory is missing or doesn't contain a CMakeLists.txt file. Skipping tests.")
    endif()
endif()

# Install the targets
install(TARGETS ${PROJECT_NAME} ${GNSSLOGGER_LIB}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# Install header files
install(FILES ${HEADERS} DESTINATION include)

# Generate and install a version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION lib/cmake/${PROJECT_NAME})

# Print build information
message(STATUS "CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_CXX_COMPILER_ID: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMAKE_CXX_COMPILER_VERSION: ${CMAKE_CXX_COMPILER_VERSION}")