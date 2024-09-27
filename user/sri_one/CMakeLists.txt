cmake_minimum_required(VERSION 3.16)
project(hje-one)

set(CMAKE_CXX_STANDARD 20)

include_directories("../../common/include")
include_directories("../../third-party/SOEM-1.4.0/osal")
include_directories("../../third-party/SOEM-1.4.0/osal/linux")
include_directories("../../third-party/SOEM-1.4.0/oshw")
include_directories("../../third-party/SOEM-1.4.0/oshw/linux")
include_directories("../../third-party/SOEM-1.4.0/soem")
include_directories("../../third-party/qpOASES/include")
include_directories("../../third-party/ParamHandler")
include_directories("../../third-party/inih")

find_package(drake CONFIG REQUIRED PATHS /opt/drake)

find_package(Threads)
file(GLOB_RECURSE sources
        "./main.cpp"
        "FSM_States/*.cpp"
        "cmpc/*.cpp"
        "utils/*.cpp"
        "utils/*.c"
        "control/*.cpp"
)

add_executable(sri ${sources})

target_link_libraries(sri PUBLIC yaml-cpp::yaml-cpp Threads::Threads soem core drake::drake inih dynacore_param_handler qpOASES)
