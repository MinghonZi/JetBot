.DEFAULT_GOAL := build

init:
	@cmake -D CMAKE_BUILD_TYPE=Debug -S . -B debug -G Ninja

build: init
	@NINJA_STATUS="[%p/%f/%t %e] " cmake --build debug -j 2

install: build
	@cmake --install debug
