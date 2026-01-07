cmake_minimum_required(VERSION 3.15)
project(GTFO LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
set(SOURCES
    gtfo.cpp
    lib/serialib/lib/serialib.cpp
    StateMachine/SerialStateMachine.cpp
    StateMachine/Nodes/AbstractStateNode.cpp
    StateMachine/Nodes/IncrementingNode.cpp
    StateMachine/Nodes/NopNode.cpp
    StateMachine/Nodes/SendStringNode.cpp
    StateMachine/Nodes/StreamNode.cpp
    StateMachine/Nodes/StringListNode.cpp
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/serialib/lib
    ${CMAKE_SOURCE_DIR}/lib/cxxopts/include
    ${CMAKE_SOURCE_DIR}/StateMachine
    ${CMAKE_SOURCE_DIR}/StateMachine/Nodes
)

# Platform-specific settings
if(WIN32)
    target_compile_definitions(${PROJECT_NAME} PRIVATE _WIN32)
    
    # Link Windows-specific libraries
    target_link_libraries(${PROJECT_NAME} PRIVATE
        setupapi
    )
    
    # For MSVC compiler
    if(MSVC)
        # Suppress warnings if needed (similar to -w in gcc)
        target_compile_options(${PROJECT_NAME} PRIVATE /W0)
        
        # Use console subsystem
        set_target_properties(${PROJECT_NAME} PROPERTIES
            LINK_FLAGS "/SUBSYSTEM:CONSOLE"
        )
    endif()
else()
    # Unix/Linux settings
    target_compile_options(${PROJECT_NAME} PRIVATE -w -g)
endif()

# Set output directory
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin
)
