cmake_minimum_required(VERSION 3.22)
project(sri)

set(CMAKE_CXX_STANDARD 20)

set(BUILD_TYPE_RELEASE TRUE)
set(CMAKE_CXX_FLAGS "-O3 -no-pie -ggdb")

add_subdirectory(third-party)
add_subdirectory(common)
add_subdirectory(user)

# copy config file
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config/ethercat_config.yaml DESTINATION
        ${CMAKE_CURRENT_BINARY_DIR}/user/sri_one)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/user/sri_one/urdf/sri.urdf DESTINATION
        ${CMAKE_CURRENT_BINARY_DIR}/user/sri_one)
