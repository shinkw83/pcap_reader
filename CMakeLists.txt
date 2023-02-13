cmake_minimum_required(VERSION 3.24)
project(pcap_reader)

set(CMAKE_CXX_STANDARD 14)

include_directories(/usr/local/Cellar/pcapplusplus/22.11/include/pcapplusplus)
link_directories(/usr/local/Cellar/pcapplusplus/22.11/lib)

add_executable(${PROJECT_NAME}
        main.cpp)

target_link_libraries(${PROJECT_NAME} Pcap++ Packet++ Common++ pcap)