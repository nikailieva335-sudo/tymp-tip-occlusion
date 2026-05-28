cmake_minimum_required(VERSION 3.30.0) #version might be wrong
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tip-occlusion)
target_sources(app PRIVATE src/main.c)
