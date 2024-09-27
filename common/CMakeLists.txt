add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

include_directories(".")
include_directories("include/" )
include_directories("../third-party/ParamHandler")
include_directories("../third-party/inih")

file(GLOB_RECURSE sources "src/*.cpp")

find_package(Threads)

add_library(core SHARED ${sources})
target_link_libraries(core inih dynacore_param_handler Threads::Threads)
