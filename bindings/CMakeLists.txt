find_package(Python 3.9 REQUIRED COMPONENTS Interpreter Development.Module)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 2.13.6 CONFIG QUIET)
if(NOT pybind11_FOUND)
  FetchContent_Declare(pybind11 URL https://codeload.github.com/pybind/pybind11/tar.gz/refs/tags/v2.13.6)
  FetchContent_MakeAvailable(pybind11)
endif()
pybind11_add_module(pydrivebench pydrivebench.cpp)
target_link_libraries(pydrivebench PRIVATE drivebench)

if(BUILD_TESTING)
  add_test(NAME python_pipeline COMMAND "${CMAKE_COMMAND}" -E env
    "PYTHONPATH=$<TARGET_FILE_DIR:pydrivebench>" "${Python_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pipeline.py")
endif()