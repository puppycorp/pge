#!/bin/bash
mkdir -p build
cmake -Bbuild -H.
cmake --build build
./build/pge