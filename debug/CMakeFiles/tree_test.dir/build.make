# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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
CMAKE_SOURCE_DIR = /home/jaybrow/rob320/projects/rob-320-project-4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jaybrow/rob320/projects/rob-320-project-4/debug

# Include any dependencies generated for this target.
include CMakeFiles/tree_test.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/tree_test.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/tree_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tree_test.dir/flags.make

CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.o: CMakeFiles/tree_test.dir/flags.make
CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.o: /home/jaybrow/rob320/projects/rob-320-project-4/test/unit_tests/tree_test.cpp
CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.o: CMakeFiles/tree_test.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jaybrow/rob320/projects/rob-320-project-4/debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.o"
	/usr/um/gcc-11.3.0/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.o -MF CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.o.d -o CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.o -c /home/jaybrow/rob320/projects/rob-320-project-4/test/unit_tests/tree_test.cpp

CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.i"
	/usr/um/gcc-11.3.0/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jaybrow/rob320/projects/rob-320-project-4/test/unit_tests/tree_test.cpp > CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.i

CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.s"
	/usr/um/gcc-11.3.0/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jaybrow/rob320/projects/rob-320-project-4/test/unit_tests/tree_test.cpp -o CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.s

# Object files for target tree_test
tree_test_OBJECTS = \
"CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.o"

# External object files for target tree_test
tree_test_EXTERNAL_OBJECTS =

tree_test: CMakeFiles/tree_test.dir/test/unit_tests/tree_test.cpp.o
tree_test: CMakeFiles/tree_test.dir/build.make
tree_test: librixrdf.a
tree_test: lib/libgtest_main.a
tree_test: /home/jaybrow/rob320/projects/rob-320-project-4/lib/librixcore.a
tree_test: /home/jaybrow/rob320/projects/rob-320-project-4/lib/librixutil.a
tree_test: /home/jaybrow/rob320/projects/rob-320-project-4/lib/librixipc.a
tree_test: lib/libgtest.a
tree_test: CMakeFiles/tree_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jaybrow/rob320/projects/rob-320-project-4/debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable tree_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tree_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tree_test.dir/build: tree_test
.PHONY : CMakeFiles/tree_test.dir/build

CMakeFiles/tree_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tree_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tree_test.dir/clean

CMakeFiles/tree_test.dir/depend:
	cd /home/jaybrow/rob320/projects/rob-320-project-4/debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jaybrow/rob320/projects/rob-320-project-4 /home/jaybrow/rob320/projects/rob-320-project-4 /home/jaybrow/rob320/projects/rob-320-project-4/debug /home/jaybrow/rob320/projects/rob-320-project-4/debug /home/jaybrow/rob320/projects/rob-320-project-4/debug/CMakeFiles/tree_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tree_test.dir/depend

