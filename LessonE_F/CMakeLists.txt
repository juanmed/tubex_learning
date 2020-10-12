cmake_minimum_required(VERSION 3.0.2)
project(my_project LANGUAGES CXX)

# Adding IBEX

# In case you installed IBEX in a local directory, you need
# to specify its path with the CMAKE_PREFIX_PATH option.
# set(CMAKE_PREFIX_PATH "~/ibex-lib/build_install")

  find_package(IBEX REQUIRED)
  ibex_init_common() # IBEX should have installed this function
message(STATUS "Found IBEX version ${IBEX_VERSION}")

# Adding Tubex

# In case you installed Tubex in a local directory, you need
# to specify its path with the CMAKE_PREFIX_PATH option.
# set(CMAKE_PREFIX_PATH "~/tubex-lib/build_install")

find_package(TUBEX REQUIRED)
message(STATUS "Found Tubex version ${TUBEX_VERSION}")

# Compilation

add_executable(${PROJECT_NAME}_E LessonE.cpp)
target_compile_options(${PROJECT_NAME}_E PUBLIC ${TUBEX_CXX_FLAGS})
target_include_directories(${PROJECT_NAME}_E SYSTEM PUBLIC ${TUBEX_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_E PUBLIC ${TUBEX_LIBRARIES} Ibex::ibex ${TUBEX_LIBRARIES} Ibex::ibex)

add_executable(${PROJECT_NAME}_F LessonF.cpp)
target_compile_options(${PROJECT_NAME}_F PUBLIC ${TUBEX_CXX_FLAGS})
target_include_directories(${PROJECT_NAME}_F SYSTEM PUBLIC ${TUBEX_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_F PUBLIC ${TUBEX_LIBRARIES} Ibex::ibex ${TUBEX_LIBRARIES} Ibex::ibex)

add_executable(${PROJECT_NAME}_F2 LessonF2.cpp)
target_compile_options(${PROJECT_NAME}_F2 PUBLIC ${TUBEX_CXX_FLAGS})
target_include_directories(${PROJECT_NAME}_F2 SYSTEM PUBLIC ${TUBEX_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_F2 PUBLIC ${TUBEX_LIBRARIES} Ibex::ibex ${TUBEX_LIBRARIES} Ibex::ibex)