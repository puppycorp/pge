#!/bin/bash
mkdir -p build
cmake -Bbuild -H. -DMODE="test"
cmake --build build
./build/pge_test "$@"