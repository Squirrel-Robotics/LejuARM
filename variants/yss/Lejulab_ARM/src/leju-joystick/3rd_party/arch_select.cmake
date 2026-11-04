# Select architecture-specific 3rd_party directory for SDL
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(SDL_3RD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/aarch64")
else()
    set(SDL_3RD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/x86_64")
endif()

if(NOT EXISTS "${SDL_3RD_PARTY_DIR}")
    message(FATAL_ERROR "3rd_party directory for ${CMAKE_SYSTEM_PROCESSOR} not found: ${SDL_3RD_PARTY_DIR}")
endif()

message(STATUS "Using SDL 3rd_party for: ${CMAKE_SYSTEM_PROCESSOR} -> ${SDL_3RD_PARTY_DIR}")
