#MPU9250 
add_library(pico_mpu9250 mpu9250.cpp mpu9250.h)

target_link_libraries(pico_mpu9250 pico_stdlib hardware_spi) #Specify which library is linked to the new library 

#target_include_directories(pico_mpu9250 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

#MahonyAHRS 
add_library(MahonyAHRS MahonyAHRS.c MahonyAHRS.h)

#init
add_library(init init.cpp init.h)

target_link_libraries(init pico_stdlib hardware_spi) #Specify which library is linked to the new library 