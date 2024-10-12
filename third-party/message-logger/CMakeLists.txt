cmake_minimum_required (VERSION 3.16)

project(message_logger)

include_directories(
        include
)

add_library(message_logger SHARED
        src/time/Time.cpp
        src/time/TimeStd.cpp
)

target_include_directories(message_logger PUBLIC
        include)