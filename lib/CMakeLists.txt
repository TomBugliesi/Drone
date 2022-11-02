#MPU9250 
add_library(pico_mpu9250 mpu9250.cpp mpu9250.h)

target_link_libraries(pico_mpu9250 pico_stdlib hardware_spi) #Specify which library is linked to the new library 

