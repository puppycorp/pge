#!/bin/bash
set -e
mkdir -p build
cmake -Bbuild -H.
cmake --build build
./build/pge