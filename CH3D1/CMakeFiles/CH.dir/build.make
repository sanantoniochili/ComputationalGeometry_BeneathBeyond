# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = "/home/tnt/Desktop/LabsnProjects/COMPUTATIONAL GEOMETRY/Programming2/Παραδοτέα/CH3D1"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/tnt/Desktop/LabsnProjects/COMPUTATIONAL GEOMETRY/Programming2/Παραδοτέα/CH3D1"

# Include any dependencies generated for this target.
include CMakeFiles/CH.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/CH.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/CH.dir/flags.make

CMakeFiles/CH.dir/CH3D_1.cpp.o: CMakeFiles/CH.dir/flags.make
CMakeFiles/CH.dir/CH3D_1.cpp.o: CH3D_1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/tnt/Desktop/LabsnProjects/COMPUTATIONAL GEOMETRY/Programming2/Παραδοτέα/CH3D1/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/CH.dir/CH3D_1.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CH.dir/CH3D_1.cpp.o -c "/home/tnt/Desktop/LabsnProjects/COMPUTATIONAL GEOMETRY/Programming2/Παραδοτέα/CH3D1/CH3D_1.cpp"

CMakeFiles/CH.dir/CH3D_1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CH.dir/CH3D_1.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/tnt/Desktop/LabsnProjects/COMPUTATIONAL GEOMETRY/Programming2/Παραδοτέα/CH3D1/CH3D_1.cpp" > CMakeFiles/CH.dir/CH3D_1.cpp.i

CMakeFiles/CH.dir/CH3D_1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CH.dir/CH3D_1.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/tnt/Desktop/LabsnProjects/COMPUTATIONAL GEOMETRY/Programming2/Παραδοτέα/CH3D1/CH3D_1.cpp" -o CMakeFiles/CH.dir/CH3D_1.cpp.s

CMakeFiles/CH.dir/CH3D_1.cpp.o.requires:

.PHONY : CMakeFiles/CH.dir/CH3D_1.cpp.o.requires

CMakeFiles/CH.dir/CH3D_1.cpp.o.provides: CMakeFiles/CH.dir/CH3D_1.cpp.o.requires
	$(MAKE) -f CMakeFiles/CH.dir/build.make CMakeFiles/CH.dir/CH3D_1.cpp.o.provides.build
.PHONY : CMakeFiles/CH.dir/CH3D_1.cpp.o.provides

CMakeFiles/CH.dir/CH3D_1.cpp.o.provides.build: CMakeFiles/CH.dir/CH3D_1.cpp.o


# Object files for target CH
CH_OBJECTS = \
"CMakeFiles/CH.dir/CH3D_1.cpp.o"

# External object files for target CH
CH_EXTERNAL_OBJECTS =

CH: CMakeFiles/CH.dir/CH3D_1.cpp.o
CH: CMakeFiles/CH.dir/build.make
CH: /usr/lib/x86_64-linux-gnu/libmpfr.so
CH: /usr/lib/x86_64-linux-gnu/libgmp.so
CH: /usr/lib/x86_64-linux-gnu/libCGAL.so.11.0.1
CH: /usr/lib/x86_64-linux-gnu/libboost_thread.so
CH: /usr/lib/x86_64-linux-gnu/libboost_system.so
CH: /usr/lib/x86_64-linux-gnu/libpthread.so
CH: /usr/lib/x86_64-linux-gnu/libCGAL.so.11.0.1
CH: /usr/lib/x86_64-linux-gnu/libboost_thread.so
CH: /usr/lib/x86_64-linux-gnu/libboost_system.so
CH: /usr/lib/x86_64-linux-gnu/libpthread.so
CH: CMakeFiles/CH.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/tnt/Desktop/LabsnProjects/COMPUTATIONAL GEOMETRY/Programming2/Παραδοτέα/CH3D1/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable CH"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CH.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/CH.dir/build: CH

.PHONY : CMakeFiles/CH.dir/build

CMakeFiles/CH.dir/requires: CMakeFiles/CH.dir/CH3D_1.cpp.o.requires

.PHONY : CMakeFiles/CH.dir/requires

CMakeFiles/CH.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/CH.dir/cmake_clean.cmake
.PHONY : CMakeFiles/CH.dir/clean

CMakeFiles/CH.dir/depend:
	cd "/home/tnt/Desktop/LabsnProjects/COMPUTATIONAL GEOMETRY/Programming2/Παραδοτέα/CH3D1" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/tnt/Desktop/LabsnProjects/COMPUTATIONAL GEOMETRY/Programming2/Παραδοτέα/CH3D1" "/home/tnt/Desktop/LabsnProjects/COMPUTATIONAL GEOMETRY/Programming2/Παραδοτέα/CH3D1" "/home/tnt/Desktop/LabsnProjects/COMPUTATIONAL GEOMETRY/Programming2/Παραδοτέα/CH3D1" "/home/tnt/Desktop/LabsnProjects/COMPUTATIONAL GEOMETRY/Programming2/Παραδοτέα/CH3D1" "/home/tnt/Desktop/LabsnProjects/COMPUTATIONAL GEOMETRY/Programming2/Παραδοτέα/CH3D1/CMakeFiles/CH.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/CH.dir/depend

