# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

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

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/peter/Desktop/tests/datadriven

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/peter/Desktop/tests/datadriven/build

# Include any dependencies generated for this target.
include locusBase/CMakeFiles/LocusMathFunctions.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include locusBase/CMakeFiles/LocusMathFunctions.dir/compiler_depend.make

# Include the progress variables for this target.
include locusBase/CMakeFiles/LocusMathFunctions.dir/progress.make

# Include the compile flags for this target's objects.
include locusBase/CMakeFiles/LocusMathFunctions.dir/flags.make

locusBase/CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.o: locusBase/CMakeFiles/LocusMathFunctions.dir/flags.make
locusBase/CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.o: /home/peter/Desktop/tests/datadriven/locusBase/LocusMathFunctions.cpp
locusBase/CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.o: locusBase/CMakeFiles/LocusMathFunctions.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/peter/Desktop/tests/datadriven/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object locusBase/CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.o"
	cd /home/peter/Desktop/tests/datadriven/build/locusBase && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT locusBase/CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.o -MF CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.o.d -o CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.o -c /home/peter/Desktop/tests/datadriven/locusBase/LocusMathFunctions.cpp

locusBase/CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.i"
	cd /home/peter/Desktop/tests/datadriven/build/locusBase && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/peter/Desktop/tests/datadriven/locusBase/LocusMathFunctions.cpp > CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.i

locusBase/CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.s"
	cd /home/peter/Desktop/tests/datadriven/build/locusBase && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/peter/Desktop/tests/datadriven/locusBase/LocusMathFunctions.cpp -o CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.s

# Object files for target LocusMathFunctions
LocusMathFunctions_OBJECTS = \
"CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.o"

# External object files for target LocusMathFunctions
LocusMathFunctions_EXTERNAL_OBJECTS =

locusBase/libLocusMathFunctions.a: locusBase/CMakeFiles/LocusMathFunctions.dir/LocusMathFunctions.cpp.o
locusBase/libLocusMathFunctions.a: locusBase/CMakeFiles/LocusMathFunctions.dir/build.make
locusBase/libLocusMathFunctions.a: locusBase/CMakeFiles/LocusMathFunctions.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/peter/Desktop/tests/datadriven/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libLocusMathFunctions.a"
	cd /home/peter/Desktop/tests/datadriven/build/locusBase && $(CMAKE_COMMAND) -P CMakeFiles/LocusMathFunctions.dir/cmake_clean_target.cmake
	cd /home/peter/Desktop/tests/datadriven/build/locusBase && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LocusMathFunctions.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
locusBase/CMakeFiles/LocusMathFunctions.dir/build: locusBase/libLocusMathFunctions.a
.PHONY : locusBase/CMakeFiles/LocusMathFunctions.dir/build

locusBase/CMakeFiles/LocusMathFunctions.dir/clean:
	cd /home/peter/Desktop/tests/datadriven/build/locusBase && $(CMAKE_COMMAND) -P CMakeFiles/LocusMathFunctions.dir/cmake_clean.cmake
.PHONY : locusBase/CMakeFiles/LocusMathFunctions.dir/clean

locusBase/CMakeFiles/LocusMathFunctions.dir/depend:
	cd /home/peter/Desktop/tests/datadriven/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/peter/Desktop/tests/datadriven /home/peter/Desktop/tests/datadriven/locusBase /home/peter/Desktop/tests/datadriven/build /home/peter/Desktop/tests/datadriven/build/locusBase /home/peter/Desktop/tests/datadriven/build/locusBase/CMakeFiles/LocusMathFunctions.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : locusBase/CMakeFiles/LocusMathFunctions.dir/depend

