# enable_language(C)
enable_language(CXX)

# set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)
# set_source_files_properties(single_gomotor.c PROPERTIES LANGUAGE CXX )
set(SOURCES single_gomotor.cpp)
add_executable(single_gomotor ${SOURCES})
target_link_libraries(single_gomotor soem pthread)
target_include_directories(single_gomotor
                            PRIVATE
                            $ENV{HOME}/demo-expo-cart-pole/cppmtr/config
                            $ENV{HOME}/demo-expo-cart-pole/cppmtr/shm/shm)
install(TARGETS single_gomotor DESTINATION bin)
