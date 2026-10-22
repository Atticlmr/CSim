cmake_minimum_required(VERSION 3.20)

project(CSim VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CTest)
include(CMakePackageConfigHelpers)

option(CSIM_BUILD_TESTS "Build the C++ test programs" ON)
option(CSIM_BUILD_PYTHON "Build the pybind11 Python extension" OFF)
option(CSIM_BUILD_VIEWER "Build the GLFW/OpenGL viewer" OFF)
option(CSIM_BUILD_PYTHON_TESTS "Register the Python unit tests with CTest" ON)
option(CSIM_BUILD_BENCHMARKS "Add the Python batch benchmark target" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# A plain `cmake -S . -B <dir>` leaves CMAKE_BUILD_TYPE empty, which compiles
# with no optimization flag at all. Default that single-config case to -O2 so
# the ordinary build is usable; an explicit -DCMAKE_BUILD_TYPE, or a
# multi-config generator that selects the configuration at build time, wins.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "CMAKE_BUILD_TYPE is empty; compiling with -O2")
    add_compile_options(-O2)
endif()

find_package(Threads REQUIRED)

add_library(csim_numerics INTERFACE)
target_include_directories(csim_numerics INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

add_library(csim_model INTERFACE)
target_link_libraries(csim_model INTERFACE csim_numerics)

add_library(csim_dynamics INTERFACE)
target_link_libraries(csim_dynamics INTERFACE csim_numerics)

find_package(tinyxml2 CONFIG QUIET)
if(TARGET tinyxml2::tinyxml2)
    set(CSIM_TINYXML2_TARGET tinyxml2::tinyxml2)
elseif(TARGET tinyxml2)
    set(CSIM_TINYXML2_TARGET tinyxml2)
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(TINYXML2 REQUIRED IMPORTED_TARGET tinyxml2)
    set(CSIM_TINYXML2_TARGET PkgConfig::TINYXML2)
endif()

add_library(csim_io STATIC
    src/io/mesh.cpp
    src/io/model_import.cpp
    src/model/rigid_body.cpp)
set_target_properties(csim_io PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(csim_io PUBLIC csim_model csim_dynamics ${CSIM_TINYXML2_TARGET})

add_library(csim_simulation INTERFACE)
target_link_libraries(csim_simulation INTERFACE csim_model csim_dynamics Threads::Threads)

add_executable(simulator apps/simulator/main.cpp)
target_link_libraries(simulator PRIVATE csim_simulation)

function(csim_add_test target source)
    add_executable(${target} ${source})
    target_link_libraries(${target} PRIVATE ${ARGN})
    add_test(NAME ${target} COMMAND ${target})
endfunction()

if(CSIM_BUILD_TESTS)
    enable_testing()

    csim_add_test(vector_test tests/math/vector_test.cpp csim_numerics)
    csim_add_test(matrix_test tests/math/matrix_test.cpp csim_numerics)
    csim_add_test(lu_test tests/math/lu_test.cpp csim_numerics)
    csim_add_test(decompositions_test tests/math/decompositions_test.cpp csim_numerics)
    csim_add_test(rotation_test tests/math/rotation_test.cpp csim_numerics)
    csim_add_test(rk4_test tests/numerics/rk4_test.cpp csim_numerics)
    csim_add_test(dormand_prince_test tests/numerics/dormand_prince_test.cpp csim_numerics)
    csim_add_test(explicit_methods_test tests/numerics/explicit_methods_test.cpp csim_numerics)
    csim_add_test(advanced_methods_test tests/numerics/advanced_methods_test.cpp csim_simulation)

    csim_add_test(pendulum_test tests/dynamics/pendulum_test.cpp csim_dynamics csim_model)
    csim_add_test(drone_test tests/dynamics/drone_test.cpp csim_dynamics csim_model)
    csim_add_test(suspended_payload_test tests/dynamics/suspended_payload_test.cpp csim_dynamics csim_model)

    csim_add_test(pendulum_simulation_test tests/simulation/pendulum_test.cpp csim_simulation)
    csim_add_test(drone_simulation_test tests/simulation/drone_test.cpp csim_simulation)
    csim_add_test(suspended_payload_simulation_test tests/simulation/suspended_payload_test.cpp csim_simulation)
    csim_add_test(cable_events_test tests/simulation/cable_events_test.cpp csim_simulation)
    csim_add_test(drone_batch_test tests/simulation/drone_batch_test.cpp csim_simulation)
    csim_add_test(rigid_payload_test tests/simulation/rigid_payload_test.cpp csim_simulation)
    csim_add_test(rigid_payload_batch_test tests/simulation/rigid_payload_batch_test.cpp csim_simulation)

    add_executable(viewer_test
        tests/viewer/viewer_test.cpp
        apps/viewer/camera.cpp
        apps/viewer/scene.cpp
        apps/viewer/trajectory.cpp)
    target_link_libraries(viewer_test PRIVATE csim_simulation)
    target_include_directories(viewer_test PRIVATE apps/viewer)
    add_test(NAME viewer_test COMMAND viewer_test)
endif()

if(CSIM_BUILD_PYTHON)
    find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
    find_package(pybind11 2.13 CONFIG REQUIRED)

    pybind11_add_module(csim MODULE
        bindings/python/module.cpp
        bindings/python/model_loading.cpp
        bindings/python/link_binding.cpp
        bindings/python/environment_binding.cpp
        bindings/python/batch_binding.cpp
        bindings/python/rigid_payload_binding.cpp
        bindings/python/rigid_payload_batch_binding.cpp)
    target_link_libraries(csim PRIVATE csim_simulation csim_io)
    target_compile_definitions(csim PRIVATE CSIM_PYTHON_BUILD)

    foreach(python_package csim_control csim_experiments csim_bridge)
        add_custom_command(TARGET csim POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${PROJECT_SOURCE_DIR}/bindings/python/${python_package}
                $<TARGET_FILE_DIR:csim>/${python_package})
    endforeach()

    if(CSIM_BUILD_PYTHON_TESTS)
        add_test(NAME python_tests
            COMMAND ${CMAKE_COMMAND} -E env
                "PYTHONPATH=$<TARGET_FILE_DIR:csim>"
                ${Python3_EXECUTABLE} -m unittest discover
                -s ${PROJECT_SOURCE_DIR}/tests/python -v)
        set_tests_properties(python_tests PROPERTIES
            DEPENDS csim
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
    endif()

    if(CSIM_BUILD_BENCHMARKS)
        add_custom_target(csim_benchmark
            COMMAND ${CMAKE_COMMAND} -E env
                "PYTHONPATH=$<TARGET_FILE_DIR:csim>"
                ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/examples/python/batch_benchmark.py
                --kind both --num-envs 1024 --threads 4
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            DEPENDS csim
            USES_TERMINAL)
    endif()
endif()

if(CSIM_BUILD_VIEWER)
    find_package(glfw3 CONFIG QUIET)
    if(TARGET glfw)
        set(CSIM_GLFW_TARGET glfw)
    elseif(TARGET glfw3)
        set(CSIM_GLFW_TARGET glfw3)
    else()
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(GLFW REQUIRED IMPORTED_TARGET glfw3)
        set(CSIM_GLFW_TARGET PkgConfig::GLFW)
    endif()
    find_package(OpenGL REQUIRED)

    add_executable(viewer
        apps/viewer/main.cpp
        apps/viewer/camera.cpp
        apps/viewer/renderer.cpp
        apps/viewer/scene.cpp
        apps/viewer/trajectory.cpp)
    target_include_directories(viewer PRIVATE apps/viewer)
    target_link_libraries(viewer PRIVATE csim_simulation ${CSIM_GLFW_TARGET} OpenGL::GL)
    add_custom_command(TARGET viewer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${PROJECT_SOURCE_DIR}/bindings/python/csim_viewer/shaders
            $<TARGET_FILE_DIR:viewer>/shaders)
endif()

if(CSIM_BUILD_PYTHON AND CSIM_BUILD_VIEWER)
    pybind11_add_module(csim_python_viewer MODULE
        bindings/python/viewer_module.cpp
        apps/viewer/window.cpp
        apps/viewer/renderer.cpp
        apps/viewer/camera.cpp)
    set_target_properties(csim_python_viewer PROPERTIES OUTPUT_NAME _csim_viewer)
    target_include_directories(csim_python_viewer PRIVATE apps/viewer)
    target_link_libraries(csim_python_viewer PRIVATE csim_simulation ${CSIM_GLFW_TARGET} OpenGL::GL)
    add_custom_command(TARGET csim_python_viewer POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${PROJECT_SOURCE_DIR}/bindings/python/csim_viewer
            $<TARGET_FILE_DIR:csim_python_viewer>/csim_viewer)
endif()

set(CSIM_INSTALL_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/CSim)
configure_package_config_file(
    cmake/CSimConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/CSimConfig.cmake
    INSTALL_DESTINATION ${CSIM_INSTALL_CMAKE_DIR})
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/CSimConfigVersion.cmake
    COMPATIBILITY SameMajorVersion)

install(TARGETS csim_numerics csim_model csim_dynamics csim_simulation csim_io
    EXPORT CSimTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS simulator
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(EXPORT CSimTargets
    FILE CSimTargets.cmake
    NAMESPACE CSim::
    DESTINATION ${CSIM_INSTALL_CMAKE_DIR})
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/CSimConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/CSimConfigVersion.cmake
    DESTINATION ${CSIM_INSTALL_CMAKE_DIR})
