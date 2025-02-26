@echo off
if not exist build mkdir build
cmake -Bbuild -H. -DTEST=ON
cmake --build build
build\Debug\pge_test %*
