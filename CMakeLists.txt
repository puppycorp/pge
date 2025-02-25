cmake_minimum_required(VERSION 3.10)
project(pge C)
set(CMAKE_C_STANDARD 11)

option(BUILD_TESTS "Build tests" OFF)

if(BUILD_TESTS)
    message(STATUS "Building tests")
    list(APPEND SOURCES src/graphics_test.c)
    list(APPEND SOURCES src/physics_tests.c)
    list(APPEND SOURCES src/test_main.c)
    add_executable(pge_test ${SOURCES})
    target_link_libraries(pge_test m)
endif()

if(APPLE)
	project(pge C OBJC)
    message(STATUS "Detected macOS - building mac target")
    set(MAC_SOURCES src/mac_main.m)
    add_executable(pge ${MAC_SOURCES})
    set_source_files_properties(src/mac_main.m PROPERTIES LANGUAGE OBJC)

    set(SHADERS ${CMAKE_SOURCE_DIR}/src/shaders.metal)
    set(SHADERS_LIB ${CMAKE_BINARY_DIR}/shaders.metallib)
    set(SHADERS_HEADER ${CMAKE_BINARY_DIR}/shaders.h)

    add_custom_command(
        OUTPUT ${SHADERS_LIB}
        COMMAND xcrun -sdk macosx metal -c ${SHADERS} -o ${CMAKE_BINARY_DIR}/shaders.air
        COMMAND xcrun -sdk macosx metallib ${CMAKE_BINARY_DIR}/shaders.air -o ${SHADERS_LIB}
        DEPENDS ${SHADERS}
        COMMENT "Compiling Metal shader to metallib"
    )

    add_custom_command(
        OUTPUT ${SHADERS_HEADER}
		COMMAND xxd -i -n shaders_metallib shaders.metallib > shaders.h
        DEPENDS ${SHADERS_LIB}
        COMMENT "Generating shader header from metallib"
    )

    add_custom_target(ShadersEmbedded ALL DEPENDS ${SHADERS_HEADER})
    add_dependencies(pge ShadersEmbedded)
    target_include_directories(pge PRIVATE ${CMAKE_BINARY_DIR})

    target_link_libraries(pge
        "-framework Cocoa"
        "-framework Metal"
        "-framework MetalKit"
    )
endif()