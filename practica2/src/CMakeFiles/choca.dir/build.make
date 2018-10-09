# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/brio/robotica2018/practica2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/brio/robotica2018/practica2

# Include any dependencies generated for this target.
include src/CMakeFiles/choca.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/choca.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/choca.dir/flags.make

src/CommonBehavior.cpp: /home/brio/robocomp/interfaces/CommonBehavior.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating CommonBehavior.cpp and CommonBehavior.h from CommonBehavior.ice"
	cd /home/brio/robotica2018/practica2/src && slice2cpp -I/home/brio/robocomp//interfaces/ -I/home/brio/robocomp/interfaces -I/opt/robocomp/interfaces -I. /home/brio/robocomp/interfaces/CommonBehavior.ice --output-dir .

src/CommonBehavior.h: src/CommonBehavior.cpp
	@$(CMAKE_COMMAND) -E touch_nocreate src/CommonBehavior.h

src/DifferentialRobot.cpp: /home/brio/robocomp/interfaces/DifferentialRobot.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating DifferentialRobot.cpp and DifferentialRobot.h from DifferentialRobot.ice"
	cd /home/brio/robotica2018/practica2/src && slice2cpp -I/home/brio/robocomp//interfaces/ -I/home/brio/robocomp/interfaces -I/opt/robocomp/interfaces -I. /home/brio/robocomp/interfaces/DifferentialRobot.ice --output-dir .

src/DifferentialRobot.h: src/DifferentialRobot.cpp
	@$(CMAKE_COMMAND) -E touch_nocreate src/DifferentialRobot.h

src/GenericBase.cpp: /home/brio/robocomp/interfaces/GenericBase.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating GenericBase.cpp and GenericBase.h from GenericBase.ice"
	cd /home/brio/robotica2018/practica2/src && slice2cpp -I/home/brio/robocomp//interfaces/ -I/home/brio/robocomp/interfaces -I/opt/robocomp/interfaces -I. /home/brio/robocomp/interfaces/GenericBase.ice --output-dir .

src/GenericBase.h: src/GenericBase.cpp
	@$(CMAKE_COMMAND) -E touch_nocreate src/GenericBase.h

src/Laser.cpp: /home/brio/robocomp/interfaces/Laser.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating Laser.cpp and Laser.h from Laser.ice"
	cd /home/brio/robotica2018/practica2/src && slice2cpp -I/home/brio/robocomp//interfaces/ -I/home/brio/robocomp/interfaces -I/opt/robocomp/interfaces -I. /home/brio/robocomp/interfaces/Laser.ice --output-dir .

src/Laser.h: src/Laser.cpp
	@$(CMAKE_COMMAND) -E touch_nocreate src/Laser.h

src/ui_mainUI.h: src/mainUI.ui
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating ui_mainUI.h"
	cd /home/brio/robotica2018/practica2/src && /usr/lib/x86_64-linux-gnu/qt4/bin/uic -o /home/brio/robotica2018/practica2/src/ui_mainUI.h /home/brio/robotica2018/practica2/src/mainUI.ui

src/CMakeFiles/choca.dir/specificworker.cpp.o: src/CMakeFiles/choca.dir/flags.make
src/CMakeFiles/choca.dir/specificworker.cpp.o: src/specificworker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/CMakeFiles/choca.dir/specificworker.cpp.o"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/choca.dir/specificworker.cpp.o -c /home/brio/robotica2018/practica2/src/specificworker.cpp

src/CMakeFiles/choca.dir/specificworker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/choca.dir/specificworker.cpp.i"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brio/robotica2018/practica2/src/specificworker.cpp > CMakeFiles/choca.dir/specificworker.cpp.i

src/CMakeFiles/choca.dir/specificworker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/choca.dir/specificworker.cpp.s"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brio/robotica2018/practica2/src/specificworker.cpp -o CMakeFiles/choca.dir/specificworker.cpp.s

src/CMakeFiles/choca.dir/specificworker.cpp.o.requires:

.PHONY : src/CMakeFiles/choca.dir/specificworker.cpp.o.requires

src/CMakeFiles/choca.dir/specificworker.cpp.o.provides: src/CMakeFiles/choca.dir/specificworker.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/choca.dir/build.make src/CMakeFiles/choca.dir/specificworker.cpp.o.provides.build
.PHONY : src/CMakeFiles/choca.dir/specificworker.cpp.o.provides

src/CMakeFiles/choca.dir/specificworker.cpp.o.provides.build: src/CMakeFiles/choca.dir/specificworker.cpp.o


src/CMakeFiles/choca.dir/specificmonitor.cpp.o: src/CMakeFiles/choca.dir/flags.make
src/CMakeFiles/choca.dir/specificmonitor.cpp.o: src/specificmonitor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/CMakeFiles/choca.dir/specificmonitor.cpp.o"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/choca.dir/specificmonitor.cpp.o -c /home/brio/robotica2018/practica2/src/specificmonitor.cpp

src/CMakeFiles/choca.dir/specificmonitor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/choca.dir/specificmonitor.cpp.i"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brio/robotica2018/practica2/src/specificmonitor.cpp > CMakeFiles/choca.dir/specificmonitor.cpp.i

src/CMakeFiles/choca.dir/specificmonitor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/choca.dir/specificmonitor.cpp.s"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brio/robotica2018/practica2/src/specificmonitor.cpp -o CMakeFiles/choca.dir/specificmonitor.cpp.s

src/CMakeFiles/choca.dir/specificmonitor.cpp.o.requires:

.PHONY : src/CMakeFiles/choca.dir/specificmonitor.cpp.o.requires

src/CMakeFiles/choca.dir/specificmonitor.cpp.o.provides: src/CMakeFiles/choca.dir/specificmonitor.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/choca.dir/build.make src/CMakeFiles/choca.dir/specificmonitor.cpp.o.provides.build
.PHONY : src/CMakeFiles/choca.dir/specificmonitor.cpp.o.provides

src/CMakeFiles/choca.dir/specificmonitor.cpp.o.provides.build: src/CMakeFiles/choca.dir/specificmonitor.cpp.o


src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o: src/CMakeFiles/choca.dir/flags.make
src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o: /home/brio/robocomp/classes/rapplication/rapplication.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o -c /home/brio/robocomp/classes/rapplication/rapplication.cpp

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.i"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brio/robocomp/classes/rapplication/rapplication.cpp > CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.i

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.s"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brio/robocomp/classes/rapplication/rapplication.cpp -o CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.s

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o.requires:

.PHONY : src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o.requires

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o.provides: src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/choca.dir/build.make src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o.provides.build
.PHONY : src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o.provides

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o.provides.build: src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o


src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o: src/CMakeFiles/choca.dir/flags.make
src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o: /home/brio/robocomp/classes/sigwatch/sigwatch.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o -c /home/brio/robocomp/classes/sigwatch/sigwatch.cpp

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.i"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brio/robocomp/classes/sigwatch/sigwatch.cpp > CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.i

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.s"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brio/robocomp/classes/sigwatch/sigwatch.cpp -o CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.s

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o.requires:

.PHONY : src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o.requires

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o.provides: src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/choca.dir/build.make src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o.provides.build
.PHONY : src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o.provides

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o.provides.build: src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o


src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o: src/CMakeFiles/choca.dir/flags.make
src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o: /home/brio/robocomp/classes/qlog/qlog.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o -c /home/brio/robocomp/classes/qlog/qlog.cpp

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.i"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brio/robocomp/classes/qlog/qlog.cpp > CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.i

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.s"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brio/robocomp/classes/qlog/qlog.cpp -o CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.s

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o.requires:

.PHONY : src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o.requires

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o.provides: src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/choca.dir/build.make src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o.provides.build
.PHONY : src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o.provides

src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o.provides.build: src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o


src/CMakeFiles/choca.dir/main.cpp.o: src/CMakeFiles/choca.dir/flags.make
src/CMakeFiles/choca.dir/main.cpp.o: src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object src/CMakeFiles/choca.dir/main.cpp.o"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/choca.dir/main.cpp.o -c /home/brio/robotica2018/practica2/src/main.cpp

src/CMakeFiles/choca.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/choca.dir/main.cpp.i"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brio/robotica2018/practica2/src/main.cpp > CMakeFiles/choca.dir/main.cpp.i

src/CMakeFiles/choca.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/choca.dir/main.cpp.s"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brio/robotica2018/practica2/src/main.cpp -o CMakeFiles/choca.dir/main.cpp.s

src/CMakeFiles/choca.dir/main.cpp.o.requires:

.PHONY : src/CMakeFiles/choca.dir/main.cpp.o.requires

src/CMakeFiles/choca.dir/main.cpp.o.provides: src/CMakeFiles/choca.dir/main.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/choca.dir/build.make src/CMakeFiles/choca.dir/main.cpp.o.provides.build
.PHONY : src/CMakeFiles/choca.dir/main.cpp.o.provides

src/CMakeFiles/choca.dir/main.cpp.o.provides.build: src/CMakeFiles/choca.dir/main.cpp.o


src/CMakeFiles/choca.dir/genericmonitor.cpp.o: src/CMakeFiles/choca.dir/flags.make
src/CMakeFiles/choca.dir/genericmonitor.cpp.o: src/genericmonitor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object src/CMakeFiles/choca.dir/genericmonitor.cpp.o"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/choca.dir/genericmonitor.cpp.o -c /home/brio/robotica2018/practica2/src/genericmonitor.cpp

src/CMakeFiles/choca.dir/genericmonitor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/choca.dir/genericmonitor.cpp.i"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brio/robotica2018/practica2/src/genericmonitor.cpp > CMakeFiles/choca.dir/genericmonitor.cpp.i

src/CMakeFiles/choca.dir/genericmonitor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/choca.dir/genericmonitor.cpp.s"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brio/robotica2018/practica2/src/genericmonitor.cpp -o CMakeFiles/choca.dir/genericmonitor.cpp.s

src/CMakeFiles/choca.dir/genericmonitor.cpp.o.requires:

.PHONY : src/CMakeFiles/choca.dir/genericmonitor.cpp.o.requires

src/CMakeFiles/choca.dir/genericmonitor.cpp.o.provides: src/CMakeFiles/choca.dir/genericmonitor.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/choca.dir/build.make src/CMakeFiles/choca.dir/genericmonitor.cpp.o.provides.build
.PHONY : src/CMakeFiles/choca.dir/genericmonitor.cpp.o.provides

src/CMakeFiles/choca.dir/genericmonitor.cpp.o.provides.build: src/CMakeFiles/choca.dir/genericmonitor.cpp.o


src/CMakeFiles/choca.dir/commonbehaviorI.cpp.o: src/CMakeFiles/choca.dir/flags.make
src/CMakeFiles/choca.dir/commonbehaviorI.cpp.o: src/commonbehaviorI.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object src/CMakeFiles/choca.dir/commonbehaviorI.cpp.o"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/choca.dir/commonbehaviorI.cpp.o -c /home/brio/robotica2018/practica2/src/commonbehaviorI.cpp

src/CMakeFiles/choca.dir/commonbehaviorI.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/choca.dir/commonbehaviorI.cpp.i"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brio/robotica2018/practica2/src/commonbehaviorI.cpp > CMakeFiles/choca.dir/commonbehaviorI.cpp.i

src/CMakeFiles/choca.dir/commonbehaviorI.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/choca.dir/commonbehaviorI.cpp.s"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brio/robotica2018/practica2/src/commonbehaviorI.cpp -o CMakeFiles/choca.dir/commonbehaviorI.cpp.s

src/CMakeFiles/choca.dir/commonbehaviorI.cpp.o.requires:

.PHONY : src/CMakeFiles/choca.dir/commonbehaviorI.cpp.o.requires

src/CMakeFiles/choca.dir/commonbehaviorI.cpp.o.provides: src/CMakeFiles/choca.dir/commonbehaviorI.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/choca.dir/build.make src/CMakeFiles/choca.dir/commonbehaviorI.cpp.o.provides.build
.PHONY : src/CMakeFiles/choca.dir/commonbehaviorI.cpp.o.provides

src/CMakeFiles/choca.dir/commonbehaviorI.cpp.o.provides.build: src/CMakeFiles/choca.dir/commonbehaviorI.cpp.o


src/CMakeFiles/choca.dir/genericworker.cpp.o: src/CMakeFiles/choca.dir/flags.make
src/CMakeFiles/choca.dir/genericworker.cpp.o: src/genericworker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object src/CMakeFiles/choca.dir/genericworker.cpp.o"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/choca.dir/genericworker.cpp.o -c /home/brio/robotica2018/practica2/src/genericworker.cpp

src/CMakeFiles/choca.dir/genericworker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/choca.dir/genericworker.cpp.i"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brio/robotica2018/practica2/src/genericworker.cpp > CMakeFiles/choca.dir/genericworker.cpp.i

src/CMakeFiles/choca.dir/genericworker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/choca.dir/genericworker.cpp.s"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brio/robotica2018/practica2/src/genericworker.cpp -o CMakeFiles/choca.dir/genericworker.cpp.s

src/CMakeFiles/choca.dir/genericworker.cpp.o.requires:

.PHONY : src/CMakeFiles/choca.dir/genericworker.cpp.o.requires

src/CMakeFiles/choca.dir/genericworker.cpp.o.provides: src/CMakeFiles/choca.dir/genericworker.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/choca.dir/build.make src/CMakeFiles/choca.dir/genericworker.cpp.o.provides.build
.PHONY : src/CMakeFiles/choca.dir/genericworker.cpp.o.provides

src/CMakeFiles/choca.dir/genericworker.cpp.o.provides.build: src/CMakeFiles/choca.dir/genericworker.cpp.o


src/CMakeFiles/choca.dir/CommonBehavior.cpp.o: src/CMakeFiles/choca.dir/flags.make
src/CMakeFiles/choca.dir/CommonBehavior.cpp.o: src/CommonBehavior.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object src/CMakeFiles/choca.dir/CommonBehavior.cpp.o"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/choca.dir/CommonBehavior.cpp.o -c /home/brio/robotica2018/practica2/src/CommonBehavior.cpp

src/CMakeFiles/choca.dir/CommonBehavior.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/choca.dir/CommonBehavior.cpp.i"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brio/robotica2018/practica2/src/CommonBehavior.cpp > CMakeFiles/choca.dir/CommonBehavior.cpp.i

src/CMakeFiles/choca.dir/CommonBehavior.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/choca.dir/CommonBehavior.cpp.s"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brio/robotica2018/practica2/src/CommonBehavior.cpp -o CMakeFiles/choca.dir/CommonBehavior.cpp.s

src/CMakeFiles/choca.dir/CommonBehavior.cpp.o.requires:

.PHONY : src/CMakeFiles/choca.dir/CommonBehavior.cpp.o.requires

src/CMakeFiles/choca.dir/CommonBehavior.cpp.o.provides: src/CMakeFiles/choca.dir/CommonBehavior.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/choca.dir/build.make src/CMakeFiles/choca.dir/CommonBehavior.cpp.o.provides.build
.PHONY : src/CMakeFiles/choca.dir/CommonBehavior.cpp.o.provides

src/CMakeFiles/choca.dir/CommonBehavior.cpp.o.provides.build: src/CMakeFiles/choca.dir/CommonBehavior.cpp.o


src/CMakeFiles/choca.dir/DifferentialRobot.cpp.o: src/CMakeFiles/choca.dir/flags.make
src/CMakeFiles/choca.dir/DifferentialRobot.cpp.o: src/DifferentialRobot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object src/CMakeFiles/choca.dir/DifferentialRobot.cpp.o"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/choca.dir/DifferentialRobot.cpp.o -c /home/brio/robotica2018/practica2/src/DifferentialRobot.cpp

src/CMakeFiles/choca.dir/DifferentialRobot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/choca.dir/DifferentialRobot.cpp.i"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brio/robotica2018/practica2/src/DifferentialRobot.cpp > CMakeFiles/choca.dir/DifferentialRobot.cpp.i

src/CMakeFiles/choca.dir/DifferentialRobot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/choca.dir/DifferentialRobot.cpp.s"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brio/robotica2018/practica2/src/DifferentialRobot.cpp -o CMakeFiles/choca.dir/DifferentialRobot.cpp.s

src/CMakeFiles/choca.dir/DifferentialRobot.cpp.o.requires:

.PHONY : src/CMakeFiles/choca.dir/DifferentialRobot.cpp.o.requires

src/CMakeFiles/choca.dir/DifferentialRobot.cpp.o.provides: src/CMakeFiles/choca.dir/DifferentialRobot.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/choca.dir/build.make src/CMakeFiles/choca.dir/DifferentialRobot.cpp.o.provides.build
.PHONY : src/CMakeFiles/choca.dir/DifferentialRobot.cpp.o.provides

src/CMakeFiles/choca.dir/DifferentialRobot.cpp.o.provides.build: src/CMakeFiles/choca.dir/DifferentialRobot.cpp.o


src/CMakeFiles/choca.dir/GenericBase.cpp.o: src/CMakeFiles/choca.dir/flags.make
src/CMakeFiles/choca.dir/GenericBase.cpp.o: src/GenericBase.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building CXX object src/CMakeFiles/choca.dir/GenericBase.cpp.o"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/choca.dir/GenericBase.cpp.o -c /home/brio/robotica2018/practica2/src/GenericBase.cpp

src/CMakeFiles/choca.dir/GenericBase.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/choca.dir/GenericBase.cpp.i"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brio/robotica2018/practica2/src/GenericBase.cpp > CMakeFiles/choca.dir/GenericBase.cpp.i

src/CMakeFiles/choca.dir/GenericBase.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/choca.dir/GenericBase.cpp.s"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brio/robotica2018/practica2/src/GenericBase.cpp -o CMakeFiles/choca.dir/GenericBase.cpp.s

src/CMakeFiles/choca.dir/GenericBase.cpp.o.requires:

.PHONY : src/CMakeFiles/choca.dir/GenericBase.cpp.o.requires

src/CMakeFiles/choca.dir/GenericBase.cpp.o.provides: src/CMakeFiles/choca.dir/GenericBase.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/choca.dir/build.make src/CMakeFiles/choca.dir/GenericBase.cpp.o.provides.build
.PHONY : src/CMakeFiles/choca.dir/GenericBase.cpp.o.provides

src/CMakeFiles/choca.dir/GenericBase.cpp.o.provides.build: src/CMakeFiles/choca.dir/GenericBase.cpp.o


src/CMakeFiles/choca.dir/Laser.cpp.o: src/CMakeFiles/choca.dir/flags.make
src/CMakeFiles/choca.dir/Laser.cpp.o: src/Laser.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building CXX object src/CMakeFiles/choca.dir/Laser.cpp.o"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/choca.dir/Laser.cpp.o -c /home/brio/robotica2018/practica2/src/Laser.cpp

src/CMakeFiles/choca.dir/Laser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/choca.dir/Laser.cpp.i"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brio/robotica2018/practica2/src/Laser.cpp > CMakeFiles/choca.dir/Laser.cpp.i

src/CMakeFiles/choca.dir/Laser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/choca.dir/Laser.cpp.s"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brio/robotica2018/practica2/src/Laser.cpp -o CMakeFiles/choca.dir/Laser.cpp.s

src/CMakeFiles/choca.dir/Laser.cpp.o.requires:

.PHONY : src/CMakeFiles/choca.dir/Laser.cpp.o.requires

src/CMakeFiles/choca.dir/Laser.cpp.o.provides: src/CMakeFiles/choca.dir/Laser.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/choca.dir/build.make src/CMakeFiles/choca.dir/Laser.cpp.o.provides.build
.PHONY : src/CMakeFiles/choca.dir/Laser.cpp.o.provides

src/CMakeFiles/choca.dir/Laser.cpp.o.provides.build: src/CMakeFiles/choca.dir/Laser.cpp.o


src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o: src/CMakeFiles/choca.dir/flags.make
src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o: src/choca_autogen/mocs_compilation.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Building CXX object src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o -c /home/brio/robotica2018/practica2/src/choca_autogen/mocs_compilation.cpp

src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.i"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/brio/robotica2018/practica2/src/choca_autogen/mocs_compilation.cpp > CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.i

src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.s"
	cd /home/brio/robotica2018/practica2/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/brio/robotica2018/practica2/src/choca_autogen/mocs_compilation.cpp -o CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.s

src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o.requires:

.PHONY : src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o.requires

src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o.provides: src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/choca.dir/build.make src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o.provides.build
.PHONY : src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o.provides

src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o.provides.build: src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o


# Object files for target choca
choca_OBJECTS = \
"CMakeFiles/choca.dir/specificworker.cpp.o" \
"CMakeFiles/choca.dir/specificmonitor.cpp.o" \
"CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o" \
"CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o" \
"CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o" \
"CMakeFiles/choca.dir/main.cpp.o" \
"CMakeFiles/choca.dir/genericmonitor.cpp.o" \
"CMakeFiles/choca.dir/commonbehaviorI.cpp.o" \
"CMakeFiles/choca.dir/genericworker.cpp.o" \
"CMakeFiles/choca.dir/CommonBehavior.cpp.o" \
"CMakeFiles/choca.dir/DifferentialRobot.cpp.o" \
"CMakeFiles/choca.dir/GenericBase.cpp.o" \
"CMakeFiles/choca.dir/Laser.cpp.o" \
"CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o"

# External object files for target choca
choca_EXTERNAL_OBJECTS =

bin/choca: src/CMakeFiles/choca.dir/specificworker.cpp.o
bin/choca: src/CMakeFiles/choca.dir/specificmonitor.cpp.o
bin/choca: src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o
bin/choca: src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o
bin/choca: src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o
bin/choca: src/CMakeFiles/choca.dir/main.cpp.o
bin/choca: src/CMakeFiles/choca.dir/genericmonitor.cpp.o
bin/choca: src/CMakeFiles/choca.dir/commonbehaviorI.cpp.o
bin/choca: src/CMakeFiles/choca.dir/genericworker.cpp.o
bin/choca: src/CMakeFiles/choca.dir/CommonBehavior.cpp.o
bin/choca: src/CMakeFiles/choca.dir/DifferentialRobot.cpp.o
bin/choca: src/CMakeFiles/choca.dir/GenericBase.cpp.o
bin/choca: src/CMakeFiles/choca.dir/Laser.cpp.o
bin/choca: src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o
bin/choca: src/CMakeFiles/choca.dir/build.make
bin/choca: /usr/lib/x86_64-linux-gnu/libQtOpenGL.so
bin/choca: /usr/lib/x86_64-linux-gnu/libQtGui.so
bin/choca: /usr/lib/x86_64-linux-gnu/libQtXml.so
bin/choca: /usr/lib/x86_64-linux-gnu/libQtCore.so
bin/choca: /usr/lib/x86_64-linux-gnu/libIce.so
bin/choca: /usr/lib/x86_64-linux-gnu/libIceStorm.so
bin/choca: src/CMakeFiles/choca.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/brio/robotica2018/practica2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_20) "Linking CXX executable ../bin/choca"
	cd /home/brio/robotica2018/practica2/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/choca.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/choca.dir/build: bin/choca

.PHONY : src/CMakeFiles/choca.dir/build

src/CMakeFiles/choca.dir/requires: src/CMakeFiles/choca.dir/specificworker.cpp.o.requires
src/CMakeFiles/choca.dir/requires: src/CMakeFiles/choca.dir/specificmonitor.cpp.o.requires
src/CMakeFiles/choca.dir/requires: src/CMakeFiles/choca.dir/home/brio/robocomp/classes/rapplication/rapplication.cpp.o.requires
src/CMakeFiles/choca.dir/requires: src/CMakeFiles/choca.dir/home/brio/robocomp/classes/sigwatch/sigwatch.cpp.o.requires
src/CMakeFiles/choca.dir/requires: src/CMakeFiles/choca.dir/home/brio/robocomp/classes/qlog/qlog.cpp.o.requires
src/CMakeFiles/choca.dir/requires: src/CMakeFiles/choca.dir/main.cpp.o.requires
src/CMakeFiles/choca.dir/requires: src/CMakeFiles/choca.dir/genericmonitor.cpp.o.requires
src/CMakeFiles/choca.dir/requires: src/CMakeFiles/choca.dir/commonbehaviorI.cpp.o.requires
src/CMakeFiles/choca.dir/requires: src/CMakeFiles/choca.dir/genericworker.cpp.o.requires
src/CMakeFiles/choca.dir/requires: src/CMakeFiles/choca.dir/CommonBehavior.cpp.o.requires
src/CMakeFiles/choca.dir/requires: src/CMakeFiles/choca.dir/DifferentialRobot.cpp.o.requires
src/CMakeFiles/choca.dir/requires: src/CMakeFiles/choca.dir/GenericBase.cpp.o.requires
src/CMakeFiles/choca.dir/requires: src/CMakeFiles/choca.dir/Laser.cpp.o.requires
src/CMakeFiles/choca.dir/requires: src/CMakeFiles/choca.dir/choca_autogen/mocs_compilation.cpp.o.requires

.PHONY : src/CMakeFiles/choca.dir/requires

src/CMakeFiles/choca.dir/clean:
	cd /home/brio/robotica2018/practica2/src && $(CMAKE_COMMAND) -P CMakeFiles/choca.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/choca.dir/clean

src/CMakeFiles/choca.dir/depend: src/CommonBehavior.cpp
src/CMakeFiles/choca.dir/depend: src/CommonBehavior.h
src/CMakeFiles/choca.dir/depend: src/DifferentialRobot.cpp
src/CMakeFiles/choca.dir/depend: src/DifferentialRobot.h
src/CMakeFiles/choca.dir/depend: src/GenericBase.cpp
src/CMakeFiles/choca.dir/depend: src/GenericBase.h
src/CMakeFiles/choca.dir/depend: src/Laser.cpp
src/CMakeFiles/choca.dir/depend: src/Laser.h
src/CMakeFiles/choca.dir/depend: src/ui_mainUI.h
	cd /home/brio/robotica2018/practica2 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/brio/robotica2018/practica2 /home/brio/robotica2018/practica2/src /home/brio/robotica2018/practica2 /home/brio/robotica2018/practica2/src /home/brio/robotica2018/practica2/src/CMakeFiles/choca.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/choca.dir/depend

