# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/emre/Desktop/Inverse_Kinematics

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/emre/Desktop/Inverse_Kinematics

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target install
install: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install

# Special rule for the target install
install/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# Special rule for the target test
test:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running tests..."
	/usr/bin/ctest --force-new-ctest-process $(ARGS)
.PHONY : test

# Special rule for the target test
test/fast: test

.PHONY : test/fast

# Special rule for the target install/local
install/local: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local

# Special rule for the target install/local
install/local/fast: install/local

.PHONY : install/local/fast

# Special rule for the target list_install_components
list_install_components:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Available install components are: \"Unspecified\""
.PHONY : list_install_components

# Special rule for the target list_install_components
list_install_components/fast: list_install_components

.PHONY : list_install_components/fast

# Special rule for the target install/strip
install/strip: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip

# Special rule for the target install/strip
install/strip/fast: install/strip

.PHONY : install/strip/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/emre/Desktop/Inverse_Kinematics/CMakeFiles /home/emre/Desktop/Inverse_Kinematics/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/emre/Desktop/Inverse_Kinematics/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named download_extra_data

# Build rule for target.
download_extra_data: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 download_extra_data
.PHONY : download_extra_data

# fast build rule for target.
download_extra_data/fast:
	$(MAKE) -f CMakeFiles/download_extra_data.dir/build.make CMakeFiles/download_extra_data.dir/build
.PHONY : download_extra_data/fast

#=============================================================================
# Target rules for targets named std_msgs_generate_messages_eus

# Build rule for target.
std_msgs_generate_messages_eus: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 std_msgs_generate_messages_eus
.PHONY : std_msgs_generate_messages_eus

# fast build rule for target.
std_msgs_generate_messages_eus/fast:
	$(MAKE) -f CMakeFiles/std_msgs_generate_messages_eus.dir/build.make CMakeFiles/std_msgs_generate_messages_eus.dir/build
.PHONY : std_msgs_generate_messages_eus/fast

#=============================================================================
# Target rules for targets named std_msgs_generate_messages_cpp

# Build rule for target.
std_msgs_generate_messages_cpp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 std_msgs_generate_messages_cpp
.PHONY : std_msgs_generate_messages_cpp

# fast build rule for target.
std_msgs_generate_messages_cpp/fast:
	$(MAKE) -f CMakeFiles/std_msgs_generate_messages_cpp.dir/build.make CMakeFiles/std_msgs_generate_messages_cpp.dir/build
.PHONY : std_msgs_generate_messages_cpp/fast

#=============================================================================
# Target rules for targets named geometry_msgs_generate_messages_py

# Build rule for target.
geometry_msgs_generate_messages_py: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 geometry_msgs_generate_messages_py
.PHONY : geometry_msgs_generate_messages_py

# fast build rule for target.
geometry_msgs_generate_messages_py/fast:
	$(MAKE) -f CMakeFiles/geometry_msgs_generate_messages_py.dir/build.make CMakeFiles/geometry_msgs_generate_messages_py.dir/build
.PHONY : geometry_msgs_generate_messages_py/fast

#=============================================================================
# Target rules for targets named std_msgs_generate_messages_nodejs

# Build rule for target.
std_msgs_generate_messages_nodejs: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 std_msgs_generate_messages_nodejs
.PHONY : std_msgs_generate_messages_nodejs

# fast build rule for target.
std_msgs_generate_messages_nodejs/fast:
	$(MAKE) -f CMakeFiles/std_msgs_generate_messages_nodejs.dir/build.make CMakeFiles/std_msgs_generate_messages_nodejs.dir/build
.PHONY : std_msgs_generate_messages_nodejs/fast

#=============================================================================
# Target rules for targets named tests

# Build rule for target.
tests: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 tests
.PHONY : tests

# fast build rule for target.
tests/fast:
	$(MAKE) -f CMakeFiles/tests.dir/build.make CMakeFiles/tests.dir/build
.PHONY : tests/fast

#=============================================================================
# Target rules for targets named app

# Build rule for target.
app: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 app
.PHONY : app

# fast build rule for target.
app/fast:
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/build
.PHONY : app/fast

#=============================================================================
# Target rules for targets named doxygen

# Build rule for target.
doxygen: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 doxygen
.PHONY : doxygen

# fast build rule for target.
doxygen/fast:
	$(MAKE) -f CMakeFiles/doxygen.dir/build.make CMakeFiles/doxygen.dir/build
.PHONY : doxygen/fast

#=============================================================================
# Target rules for targets named run_tests

# Build rule for target.
run_tests: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 run_tests
.PHONY : run_tests

# fast build rule for target.
run_tests/fast:
	$(MAKE) -f CMakeFiles/run_tests.dir/build.make CMakeFiles/run_tests.dir/build
.PHONY : run_tests/fast

#=============================================================================
# Target rules for targets named sensor_msgs_generate_messages_py

# Build rule for target.
sensor_msgs_generate_messages_py: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 sensor_msgs_generate_messages_py
.PHONY : sensor_msgs_generate_messages_py

# fast build rule for target.
sensor_msgs_generate_messages_py/fast:
	$(MAKE) -f CMakeFiles/sensor_msgs_generate_messages_py.dir/build.make CMakeFiles/sensor_msgs_generate_messages_py.dir/build
.PHONY : sensor_msgs_generate_messages_py/fast

#=============================================================================
# Target rules for targets named std_msgs_generate_messages_py

# Build rule for target.
std_msgs_generate_messages_py: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 std_msgs_generate_messages_py
.PHONY : std_msgs_generate_messages_py

# fast build rule for target.
std_msgs_generate_messages_py/fast:
	$(MAKE) -f CMakeFiles/std_msgs_generate_messages_py.dir/build.make CMakeFiles/std_msgs_generate_messages_py.dir/build
.PHONY : std_msgs_generate_messages_py/fast

#=============================================================================
# Target rules for targets named geometry_msgs_generate_messages_eus

# Build rule for target.
geometry_msgs_generate_messages_eus: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 geometry_msgs_generate_messages_eus
.PHONY : geometry_msgs_generate_messages_eus

# fast build rule for target.
geometry_msgs_generate_messages_eus/fast:
	$(MAKE) -f CMakeFiles/geometry_msgs_generate_messages_eus.dir/build.make CMakeFiles/geometry_msgs_generate_messages_eus.dir/build
.PHONY : geometry_msgs_generate_messages_eus/fast

#=============================================================================
# Target rules for targets named sensor_msgs_generate_messages_cpp

# Build rule for target.
sensor_msgs_generate_messages_cpp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 sensor_msgs_generate_messages_cpp
.PHONY : sensor_msgs_generate_messages_cpp

# fast build rule for target.
sensor_msgs_generate_messages_cpp/fast:
	$(MAKE) -f CMakeFiles/sensor_msgs_generate_messages_cpp.dir/build.make CMakeFiles/sensor_msgs_generate_messages_cpp.dir/build
.PHONY : sensor_msgs_generate_messages_cpp/fast

#=============================================================================
# Target rules for targets named sensor_msgs_generate_messages_eus

# Build rule for target.
sensor_msgs_generate_messages_eus: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 sensor_msgs_generate_messages_eus
.PHONY : sensor_msgs_generate_messages_eus

# fast build rule for target.
sensor_msgs_generate_messages_eus/fast:
	$(MAKE) -f CMakeFiles/sensor_msgs_generate_messages_eus.dir/build.make CMakeFiles/sensor_msgs_generate_messages_eus.dir/build
.PHONY : sensor_msgs_generate_messages_eus/fast

#=============================================================================
# Target rules for targets named sensor_msgs_generate_messages_lisp

# Build rule for target.
sensor_msgs_generate_messages_lisp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 sensor_msgs_generate_messages_lisp
.PHONY : sensor_msgs_generate_messages_lisp

# fast build rule for target.
sensor_msgs_generate_messages_lisp/fast:
	$(MAKE) -f CMakeFiles/sensor_msgs_generate_messages_lisp.dir/build.make CMakeFiles/sensor_msgs_generate_messages_lisp.dir/build
.PHONY : sensor_msgs_generate_messages_lisp/fast

#=============================================================================
# Target rules for targets named geometry_msgs_generate_messages_cpp

# Build rule for target.
geometry_msgs_generate_messages_cpp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 geometry_msgs_generate_messages_cpp
.PHONY : geometry_msgs_generate_messages_cpp

# fast build rule for target.
geometry_msgs_generate_messages_cpp/fast:
	$(MAKE) -f CMakeFiles/geometry_msgs_generate_messages_cpp.dir/build.make CMakeFiles/geometry_msgs_generate_messages_cpp.dir/build
.PHONY : geometry_msgs_generate_messages_cpp/fast

#=============================================================================
# Target rules for targets named sensor_msgs_generate_messages_nodejs

# Build rule for target.
sensor_msgs_generate_messages_nodejs: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 sensor_msgs_generate_messages_nodejs
.PHONY : sensor_msgs_generate_messages_nodejs

# fast build rule for target.
sensor_msgs_generate_messages_nodejs/fast:
	$(MAKE) -f CMakeFiles/sensor_msgs_generate_messages_nodejs.dir/build.make CMakeFiles/sensor_msgs_generate_messages_nodejs.dir/build
.PHONY : sensor_msgs_generate_messages_nodejs/fast

#=============================================================================
# Target rules for targets named std_msgs_generate_messages_lisp

# Build rule for target.
std_msgs_generate_messages_lisp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 std_msgs_generate_messages_lisp
.PHONY : std_msgs_generate_messages_lisp

# fast build rule for target.
std_msgs_generate_messages_lisp/fast:
	$(MAKE) -f CMakeFiles/std_msgs_generate_messages_lisp.dir/build.make CMakeFiles/std_msgs_generate_messages_lisp.dir/build
.PHONY : std_msgs_generate_messages_lisp/fast

#=============================================================================
# Target rules for targets named geometry_msgs_generate_messages_lisp

# Build rule for target.
geometry_msgs_generate_messages_lisp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 geometry_msgs_generate_messages_lisp
.PHONY : geometry_msgs_generate_messages_lisp

# fast build rule for target.
geometry_msgs_generate_messages_lisp/fast:
	$(MAKE) -f CMakeFiles/geometry_msgs_generate_messages_lisp.dir/build.make CMakeFiles/geometry_msgs_generate_messages_lisp.dir/build
.PHONY : geometry_msgs_generate_messages_lisp/fast

#=============================================================================
# Target rules for targets named clean_test_results

# Build rule for target.
clean_test_results: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 clean_test_results
.PHONY : clean_test_results

# fast build rule for target.
clean_test_results/fast:
	$(MAKE) -f CMakeFiles/clean_test_results.dir/build.make CMakeFiles/clean_test_results.dir/build
.PHONY : clean_test_results/fast

#=============================================================================
# Target rules for targets named geometry_msgs_generate_messages_nodejs

# Build rule for target.
geometry_msgs_generate_messages_nodejs: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 geometry_msgs_generate_messages_nodejs
.PHONY : geometry_msgs_generate_messages_nodejs

# fast build rule for target.
geometry_msgs_generate_messages_nodejs/fast:
	$(MAKE) -f CMakeFiles/geometry_msgs_generate_messages_nodejs.dir/build.make CMakeFiles/geometry_msgs_generate_messages_nodejs.dir/build
.PHONY : geometry_msgs_generate_messages_nodejs/fast

#=============================================================================
# Target rules for targets named gmock

# Build rule for target.
gmock: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 gmock
.PHONY : gmock

# fast build rule for target.
gmock/fast:
	$(MAKE) -f gtest/CMakeFiles/gmock.dir/build.make gtest/CMakeFiles/gmock.dir/build
.PHONY : gmock/fast

#=============================================================================
# Target rules for targets named gmock_main

# Build rule for target.
gmock_main: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 gmock_main
.PHONY : gmock_main

# fast build rule for target.
gmock_main/fast:
	$(MAKE) -f gtest/CMakeFiles/gmock_main.dir/build.make gtest/CMakeFiles/gmock_main.dir/build
.PHONY : gmock_main/fast

#=============================================================================
# Target rules for targets named gtest

# Build rule for target.
gtest: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 gtest
.PHONY : gtest

# fast build rule for target.
gtest/fast:
	$(MAKE) -f gtest/gtest/CMakeFiles/gtest.dir/build.make gtest/gtest/CMakeFiles/gtest.dir/build
.PHONY : gtest/fast

#=============================================================================
# Target rules for targets named gtest_main

# Build rule for target.
gtest_main: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 gtest_main
.PHONY : gtest_main

# fast build rule for target.
gtest_main/fast:
	$(MAKE) -f gtest/gtest/CMakeFiles/gtest_main.dir/build.make gtest/gtest/CMakeFiles/gtest_main.dir/build
.PHONY : gtest_main/fast

src/Inverse_Kinematics.o: src/Inverse_Kinematics.cpp.o

.PHONY : src/Inverse_Kinematics.o

# target to build an object file
src/Inverse_Kinematics.cpp.o:
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/src/Inverse_Kinematics.cpp.o
.PHONY : src/Inverse_Kinematics.cpp.o

src/Inverse_Kinematics.i: src/Inverse_Kinematics.cpp.i

.PHONY : src/Inverse_Kinematics.i

# target to preprocess a source file
src/Inverse_Kinematics.cpp.i:
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/src/Inverse_Kinematics.cpp.i
.PHONY : src/Inverse_Kinematics.cpp.i

src/Inverse_Kinematics.s: src/Inverse_Kinematics.cpp.s

.PHONY : src/Inverse_Kinematics.s

# target to generate assembly for a file
src/Inverse_Kinematics.cpp.s:
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/src/Inverse_Kinematics.cpp.s
.PHONY : src/Inverse_Kinematics.cpp.s

src/main.o: src/main.cpp.o

.PHONY : src/main.o

# target to build an object file
src/main.cpp.o:
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/src/main.cpp.o
.PHONY : src/main.cpp.o

src/main.i: src/main.cpp.i

.PHONY : src/main.i

# target to preprocess a source file
src/main.cpp.i:
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/src/main.cpp.i
.PHONY : src/main.cpp.i

src/main.s: src/main.cpp.s

.PHONY : src/main.s

# target to generate assembly for a file
src/main.cpp.s:
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/src/main.cpp.s
.PHONY : src/main.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... install"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... test"
	@echo "... install/local"
	@echo "... download_extra_data"
	@echo "... std_msgs_generate_messages_eus"
	@echo "... std_msgs_generate_messages_cpp"
	@echo "... geometry_msgs_generate_messages_py"
	@echo "... std_msgs_generate_messages_nodejs"
	@echo "... tests"
	@echo "... app"
	@echo "... doxygen"
	@echo "... run_tests"
	@echo "... sensor_msgs_generate_messages_py"
	@echo "... std_msgs_generate_messages_py"
	@echo "... geometry_msgs_generate_messages_eus"
	@echo "... sensor_msgs_generate_messages_cpp"
	@echo "... list_install_components"
	@echo "... sensor_msgs_generate_messages_eus"
	@echo "... sensor_msgs_generate_messages_lisp"
	@echo "... geometry_msgs_generate_messages_cpp"
	@echo "... sensor_msgs_generate_messages_nodejs"
	@echo "... std_msgs_generate_messages_lisp"
	@echo "... geometry_msgs_generate_messages_lisp"
	@echo "... clean_test_results"
	@echo "... geometry_msgs_generate_messages_nodejs"
	@echo "... install/strip"
	@echo "... gmock"
	@echo "... gmock_main"
	@echo "... gtest"
	@echo "... gtest_main"
	@echo "... src/Inverse_Kinematics.o"
	@echo "... src/Inverse_Kinematics.i"
	@echo "... src/Inverse_Kinematics.s"
	@echo "... src/main.o"
	@echo "... src/main.i"
	@echo "... src/main.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

