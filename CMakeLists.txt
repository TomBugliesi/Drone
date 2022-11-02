cmake_minimum_required(VERSION 3.10.2)

include(pico_sdk_import.cmake) #Pull in SDK (must be before project)

project(rooster) #Name of the output executable file automatically stored as PROJECT_NAME
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init() #Initialize the SDK

add_executable(${PROJECT_NAME} main.cpp) #the executable is the c or cpp file that contains the main loop
pico_add_extra_outputs(${PROJECT_NAME}) # create map/bin/hex file etc.

add_subdirectory(lib)#Name of the library folder where the .h and .cpp files are saved

target_include_directories(${PROJECT_NAME} PUBLIC lib) #Add LibraryFolder as a public directory for the project path 

target_link_directories(${PROJECT_NAME} PRIVATE lib) #The folder will be also included in the final build folder

target_link_libraries(${PROJECT_NAME} pico_mpu9250) #The Library is the CMake library name that CMake will look for in the compilation process

pico_enable_stdio_usb(${PROJECT_NAME} 1) #Set Serial communication to active 
pico_enable_stdio_uart(${PROJECT_NAME} 0)