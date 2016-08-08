# Target specific macros
TARGET = medina

TARGET_CPP_SOURCES =  movement.cpp communication.cpp slam.cpp medina.cpp

TOPPERS_OSEK_OIL_SOURCE = ./medina.oil

BUILD_MODE = ROM_ONLY

O_PATH ?= build

# makefile for C++(.cpp) build
include /cygdrive/c/cygwin/nxtOSEK/ecrobot/ecrobot++.mak
