cmake_minimum_required(VERSION 3.5)
project(libdecklink_wrapper)

option(BUILD_DOC      "Build documentation"         OFF)

#
# Compiler configuration
#

set(CMAKE_CXX_STANDARD 14)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_definitions("-Wdocumentation")
endif()

# Add a bunch of extra flags. We assume that the only compilers ever used will be GCC or Clang.
add_definitions("-Wextra -Wdouble-promotion -Wshadow -Wconversion")

# Optimise code for this processor. We won't be shipping binaries
add_definitions("-march=native")


#
# Code Dependencies
#

find_package(Boost REQUIRED COMPONENTS exception program_options)
find_package(Threads REQUIRED)

#
# Binaries & Libraries
#

# Locate BlackMagic DeckLink library headers
SET(DECKLINK_SDK_DIR "${PROJECT_SOURCE_DIR}/include/decklink_sdk")
SET(DECKLINK_API_DISPATCH_SRC ${DECKLINK_SDK_DIR}/DeckLinkAPIDispatch.cpp)

## Create libdecklink library
#
# The library wraps the SDK in a higher level SDK and hides the dlsym and
# pthreads dependencies.

SET(DECKLINK_WRAPPER_SRCS
        src/libdecklink/types.cpp
        src/libdecklink/device.cpp
        src/libdecklink/display_mode.cpp
        src/libdecklink/device_input_interface.cpp
        src/libdecklink/device_output_interface.cpp

        src/libdecklink/video_frame.cpp
        src/libdecklink/video_input_frame.cpp
        src/libdecklink/video_output_frame.cpp

        src/libdecklink/capture_callback.cpp
        src/libdecklink/notification_callback.cpp

        ${LIBDECKLINK_GENERATED_SRC}
        ${DECKLINK_API_DISPATCH_SRC}
        )

add_library(decklink ${DECKLINK_WRAPPER_SRCS})
target_link_libraries(decklink
        Threads::Threads
        ${CMAKE_DL_LIBS}
        ${Boost_LIBRARIES}
        )
target_include_directories(decklink
        PUBLIC
            include/
            ${DECKLINK_SDK_DIR}/include
        PRIVATE
            ${Boost_INCLUDE_DIRS}
        )

#
# Documentation
#

# check if Doxygen is installed
find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    # note the option ALL which allows to build the docs together with the application
    add_custom_target(docs ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM
        )

else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

# Dummy target for IDEs
FILE (GLOB_RECURSE all_headers_for_ides
        ${CMAKE_SOURCE_DIR}/**/*.hpp
        ${CMAKE_SOURCE_DIR}/**/*.h)
ADD_CUSTOM_TARGET(ide_includes SOURCES ${all_headers_for_ides})
