# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/takashi/test/endoscope_holding_robot/robot/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/takashi/test/endoscope_holding_robot/robot/build

# Include any dependencies generated for this target.
include robot/CMakeFiles/pi_real_client.dir/depend.make

# Include the progress variables for this target.
include robot/CMakeFiles/pi_real_client.dir/progress.make

# Include the compile flags for this target's objects.
include robot/CMakeFiles/pi_real_client.dir/flags.make

robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o: robot/CMakeFiles/pi_real_client.dir/flags.make
robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o: /home/takashi/test/endoscope_holding_robot/robot/src/robot/src/pi_real_client.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/takashi/test/endoscope_holding_robot/robot/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o"
	cd /home/takashi/test/endoscope_holding_robot/robot/build/robot && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o -c /home/takashi/test/endoscope_holding_robot/robot/src/robot/src/pi_real_client.cpp

robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.i"
	cd /home/takashi/test/endoscope_holding_robot/robot/build/robot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/takashi/test/endoscope_holding_robot/robot/src/robot/src/pi_real_client.cpp > CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.i

robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.s"
	cd /home/takashi/test/endoscope_holding_robot/robot/build/robot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/takashi/test/endoscope_holding_robot/robot/src/robot/src/pi_real_client.cpp -o CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.s

robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o.requires:
.PHONY : robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o.requires

robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o.provides: robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o.requires
	$(MAKE) -f robot/CMakeFiles/pi_real_client.dir/build.make robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o.provides.build
.PHONY : robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o.provides

robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o.provides.build: robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o

# Object files for target pi_real_client
pi_real_client_OBJECTS = \
"CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o"

# External object files for target pi_real_client
pi_real_client_EXTERNAL_OBJECTS =

/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: robot/CMakeFiles/pi_real_client.dir/build.make
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /opt/ros/indigo/lib/libtf.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /opt/ros/indigo/lib/libtf2_ros.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /opt/ros/indigo/lib/libactionlib.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /opt/ros/indigo/lib/libmessage_filters.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /opt/ros/indigo/lib/libroscpp.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /opt/ros/indigo/lib/libtf2.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /opt/ros/indigo/lib/librosconsole.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /usr/lib/liblog4cxx.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /opt/ros/indigo/lib/librostime.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /opt/ros/indigo/lib/libcpp_common.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client: robot/CMakeFiles/pi_real_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client"
	cd /home/takashi/test/endoscope_holding_robot/robot/build/robot && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pi_real_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
robot/CMakeFiles/pi_real_client.dir/build: /home/takashi/test/endoscope_holding_robot/robot/devel/lib/robot/pi_real_client
.PHONY : robot/CMakeFiles/pi_real_client.dir/build

robot/CMakeFiles/pi_real_client.dir/requires: robot/CMakeFiles/pi_real_client.dir/src/pi_real_client.cpp.o.requires
.PHONY : robot/CMakeFiles/pi_real_client.dir/requires

robot/CMakeFiles/pi_real_client.dir/clean:
	cd /home/takashi/test/endoscope_holding_robot/robot/build/robot && $(CMAKE_COMMAND) -P CMakeFiles/pi_real_client.dir/cmake_clean.cmake
.PHONY : robot/CMakeFiles/pi_real_client.dir/clean

robot/CMakeFiles/pi_real_client.dir/depend:
	cd /home/takashi/test/endoscope_holding_robot/robot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/takashi/test/endoscope_holding_robot/robot/src /home/takashi/test/endoscope_holding_robot/robot/src/robot /home/takashi/test/endoscope_holding_robot/robot/build /home/takashi/test/endoscope_holding_robot/robot/build/robot /home/takashi/test/endoscope_holding_robot/robot/build/robot/CMakeFiles/pi_real_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : robot/CMakeFiles/pi_real_client.dir/depend

