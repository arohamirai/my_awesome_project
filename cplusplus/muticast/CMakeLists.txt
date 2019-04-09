cmake_minimum_required(VERSION 2.8.3)
project(muticast)

include_directories(
# include
)

## Declare a C++ library
add_executable(broadcast_server
   src/broadcast_server.c
 )
target_link_libraries(broadcast_server
)
 
add_executable(broadcast_client
   src/broadcast_client.c
 )
target_link_libraries(broadcast_client
)
