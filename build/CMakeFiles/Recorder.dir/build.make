# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/max/Documents/zedTing

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/max/Documents/zedTing/build

# Include any dependencies generated for this target.
include CMakeFiles/Recorder.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Recorder.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Recorder.dir/flags.make

CMakeFiles/Recorder.dir/src/main.cpp.o: CMakeFiles/Recorder.dir/flags.make
CMakeFiles/Recorder.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/max/Documents/zedTing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Recorder.dir/src/main.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Recorder.dir/src/main.cpp.o -c /home/max/Documents/zedTing/src/main.cpp

CMakeFiles/Recorder.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Recorder.dir/src/main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/max/Documents/zedTing/src/main.cpp > CMakeFiles/Recorder.dir/src/main.cpp.i

CMakeFiles/Recorder.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Recorder.dir/src/main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/max/Documents/zedTing/src/main.cpp -o CMakeFiles/Recorder.dir/src/main.cpp.s

# Object files for target Recorder
Recorder_OBJECTS = \
"CMakeFiles/Recorder.dir/src/main.cpp.o"

# External object files for target Recorder
Recorder_EXTERNAL_OBJECTS =

Recorder: CMakeFiles/Recorder.dir/src/main.cpp.o
Recorder: CMakeFiles/Recorder.dir/build.make
Recorder: CMakeFiles/Recorder.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/max/Documents/zedTing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Recorder"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Recorder.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Recorder.dir/build: Recorder

.PHONY : CMakeFiles/Recorder.dir/build

CMakeFiles/Recorder.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Recorder.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Recorder.dir/clean

CMakeFiles/Recorder.dir/depend:
	cd /home/max/Documents/zedTing/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/max/Documents/zedTing /home/max/Documents/zedTing /home/max/Documents/zedTing/build /home/max/Documents/zedTing/build /home/max/Documents/zedTing/build/CMakeFiles/Recorder.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Recorder.dir/depend

