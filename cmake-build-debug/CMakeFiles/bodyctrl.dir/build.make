# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
CMAKE_COMMAND = /home/bie/clion-2020.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/bie/clion-2020.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bie/spider/bodyctrl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bie/spider/bodyctrl/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/bodyctrl.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/bodyctrl.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/bodyctrl.dir/flags.make

CMakeFiles/bodyctrl.dir/main.cpp.o: CMakeFiles/bodyctrl.dir/flags.make
CMakeFiles/bodyctrl.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bie/spider/bodyctrl/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/bodyctrl.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bodyctrl.dir/main.cpp.o -c /home/bie/spider/bodyctrl/main.cpp

CMakeFiles/bodyctrl.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bodyctrl.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bie/spider/bodyctrl/main.cpp > CMakeFiles/bodyctrl.dir/main.cpp.i

CMakeFiles/bodyctrl.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bodyctrl.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bie/spider/bodyctrl/main.cpp -o CMakeFiles/bodyctrl.dir/main.cpp.s

CMakeFiles/bodyctrl.dir/matrix.cpp.o: CMakeFiles/bodyctrl.dir/flags.make
CMakeFiles/bodyctrl.dir/matrix.cpp.o: ../matrix.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bie/spider/bodyctrl/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/bodyctrl.dir/matrix.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bodyctrl.dir/matrix.cpp.o -c /home/bie/spider/bodyctrl/matrix.cpp

CMakeFiles/bodyctrl.dir/matrix.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bodyctrl.dir/matrix.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bie/spider/bodyctrl/matrix.cpp > CMakeFiles/bodyctrl.dir/matrix.cpp.i

CMakeFiles/bodyctrl.dir/matrix.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bodyctrl.dir/matrix.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bie/spider/bodyctrl/matrix.cpp -o CMakeFiles/bodyctrl.dir/matrix.cpp.s

CMakeFiles/bodyctrl.dir/coordinates_ctrl.cpp.o: CMakeFiles/bodyctrl.dir/flags.make
CMakeFiles/bodyctrl.dir/coordinates_ctrl.cpp.o: ../coordinates_ctrl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bie/spider/bodyctrl/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/bodyctrl.dir/coordinates_ctrl.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bodyctrl.dir/coordinates_ctrl.cpp.o -c /home/bie/spider/bodyctrl/coordinates_ctrl.cpp

CMakeFiles/bodyctrl.dir/coordinates_ctrl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bodyctrl.dir/coordinates_ctrl.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bie/spider/bodyctrl/coordinates_ctrl.cpp > CMakeFiles/bodyctrl.dir/coordinates_ctrl.cpp.i

CMakeFiles/bodyctrl.dir/coordinates_ctrl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bodyctrl.dir/coordinates_ctrl.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bie/spider/bodyctrl/coordinates_ctrl.cpp -o CMakeFiles/bodyctrl.dir/coordinates_ctrl.cpp.s

CMakeFiles/bodyctrl.dir/reachable.cpp.o: CMakeFiles/bodyctrl.dir/flags.make
CMakeFiles/bodyctrl.dir/reachable.cpp.o: ../reachable.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bie/spider/bodyctrl/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/bodyctrl.dir/reachable.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bodyctrl.dir/reachable.cpp.o -c /home/bie/spider/bodyctrl/reachable.cpp

CMakeFiles/bodyctrl.dir/reachable.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bodyctrl.dir/reachable.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bie/spider/bodyctrl/reachable.cpp > CMakeFiles/bodyctrl.dir/reachable.cpp.i

CMakeFiles/bodyctrl.dir/reachable.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bodyctrl.dir/reachable.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bie/spider/bodyctrl/reachable.cpp -o CMakeFiles/bodyctrl.dir/reachable.cpp.s

CMakeFiles/bodyctrl.dir/communication.cpp.o: CMakeFiles/bodyctrl.dir/flags.make
CMakeFiles/bodyctrl.dir/communication.cpp.o: ../communication.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bie/spider/bodyctrl/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/bodyctrl.dir/communication.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bodyctrl.dir/communication.cpp.o -c /home/bie/spider/bodyctrl/communication.cpp

CMakeFiles/bodyctrl.dir/communication.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bodyctrl.dir/communication.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bie/spider/bodyctrl/communication.cpp > CMakeFiles/bodyctrl.dir/communication.cpp.i

CMakeFiles/bodyctrl.dir/communication.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bodyctrl.dir/communication.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bie/spider/bodyctrl/communication.cpp -o CMakeFiles/bodyctrl.dir/communication.cpp.s

CMakeFiles/bodyctrl.dir/angle_ctrl.cpp.o: CMakeFiles/bodyctrl.dir/flags.make
CMakeFiles/bodyctrl.dir/angle_ctrl.cpp.o: ../angle_ctrl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bie/spider/bodyctrl/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/bodyctrl.dir/angle_ctrl.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bodyctrl.dir/angle_ctrl.cpp.o -c /home/bie/spider/bodyctrl/angle_ctrl.cpp

CMakeFiles/bodyctrl.dir/angle_ctrl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bodyctrl.dir/angle_ctrl.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bie/spider/bodyctrl/angle_ctrl.cpp > CMakeFiles/bodyctrl.dir/angle_ctrl.cpp.i

CMakeFiles/bodyctrl.dir/angle_ctrl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bodyctrl.dir/angle_ctrl.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bie/spider/bodyctrl/angle_ctrl.cpp -o CMakeFiles/bodyctrl.dir/angle_ctrl.cpp.s

# Object files for target bodyctrl
bodyctrl_OBJECTS = \
"CMakeFiles/bodyctrl.dir/main.cpp.o" \
"CMakeFiles/bodyctrl.dir/matrix.cpp.o" \
"CMakeFiles/bodyctrl.dir/coordinates_ctrl.cpp.o" \
"CMakeFiles/bodyctrl.dir/reachable.cpp.o" \
"CMakeFiles/bodyctrl.dir/communication.cpp.o" \
"CMakeFiles/bodyctrl.dir/angle_ctrl.cpp.o"

# External object files for target bodyctrl
bodyctrl_EXTERNAL_OBJECTS =

bodyctrl: CMakeFiles/bodyctrl.dir/main.cpp.o
bodyctrl: CMakeFiles/bodyctrl.dir/matrix.cpp.o
bodyctrl: CMakeFiles/bodyctrl.dir/coordinates_ctrl.cpp.o
bodyctrl: CMakeFiles/bodyctrl.dir/reachable.cpp.o
bodyctrl: CMakeFiles/bodyctrl.dir/communication.cpp.o
bodyctrl: CMakeFiles/bodyctrl.dir/angle_ctrl.cpp.o
bodyctrl: CMakeFiles/bodyctrl.dir/build.make
bodyctrl: CMakeFiles/bodyctrl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bie/spider/bodyctrl/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable bodyctrl"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bodyctrl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/bodyctrl.dir/build: bodyctrl

.PHONY : CMakeFiles/bodyctrl.dir/build

CMakeFiles/bodyctrl.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/bodyctrl.dir/cmake_clean.cmake
.PHONY : CMakeFiles/bodyctrl.dir/clean

CMakeFiles/bodyctrl.dir/depend:
	cd /home/bie/spider/bodyctrl/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bie/spider/bodyctrl /home/bie/spider/bodyctrl /home/bie/spider/bodyctrl/cmake-build-debug /home/bie/spider/bodyctrl/cmake-build-debug /home/bie/spider/bodyctrl/cmake-build-debug/CMakeFiles/bodyctrl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/bodyctrl.dir/depend

