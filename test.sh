#!/bin/bash
set -e
mkdir -p build
cmake -Bbuild -H. -DTEST=ON
cmake --build build
./build/pge_test "$@"