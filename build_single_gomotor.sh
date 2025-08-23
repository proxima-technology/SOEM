rm -rf build
mkdir build && cd build
cmake .. -DBUILD_SINGLE_GOMOTOR=ON
make