.DEFAULT_GOAL := build

init:
	cmake -DCMAKE_C_COMPILER=gcc-11 -DCMAKE_CXX_COMPILER=g++-11 -S . -B build

build: init
	cmake --build build -j 2

install: build
	cmake --install build
