merge wip

Karl Zeilhofer
authored10/20/2018 @ 9:32 AM
parent:78fd5b
1 modified
Name

Path

Tree

View all files
motion.cpp
File History: motion.cpp
merge wip
a minute ago by Karl Zeilhofer
d37364
format all source code, prepare for merge
an hour ago by Karl Zeilhofer
8a6b48
add format.sh with artistic style and reformatted source code
2 hours ago by Karl Zeilhofer
f80002
switch between MK3 and MK2.5 current settings
2 days ago by PavelSindler
7d1891
add speed parameter for move_xxx() functions, enable production loop again
3 days ago by Karl Zeilhofer
83ce07
eject/pull back filament for tip inspection, using move_pulley()
3 days ago by Karl Zeilhofer
b8c405
documentation, comments and simplifications
4 days ago by Karl Zeilhofer
f89d81
some comments
4 days ago by Karl Zeilhofer
b0f381
simlified load_filament_intoExtruder()
4 days ago by Karl Zeilhofer
80422a
mk25 currents initial
4 days ago by PavelSindler
c4ae20
removed engaging filament before moving selector, replaced some 5-constants
4 days ago by Karl Zeilhofer
4775ce
bugfixes in homing
4 days ago by Karl Zeilhofer
abd451
add smooth homing and smooth move for all axis, add enum for Motion Return
4 days ago by Karl Zeilhofer
70ea87
introduce isFilamentInFinda()
4 days ago by Karl Zeilhofer
0a5f94
perfect homing, ultra fast, 6 positions (all selector)
5 days ago by Karl Zeilhofer
d25812
proper homing of selector
5 days ago by Karl Zeilhofer
ccea9a
add testing: 5k steps/s on selector with 20k steps/s^2 acceleration
5 days ago by Karl Zeilhofer
c2f113
sequential reference move to initial position after homing
5 days ago by Karl Zeilhofer
2cebdc
minor changes, compiles now again (with platformio)
5 days ago by Karl Zeilhofer
8644b9
add separate move functions for each axis (for better readability)
5 days ago by Karl Zeilhofer
080ed8
removed all state checks of isIdlerParked for callers of engage_pully()
6 days ago by Karl Zeilhofer
534195
major "bug" fix: if engage_filament_pully(true) is called twice, ...
6 days ago by Karl Zeilhofer
d44a9e
fixed comment
6 days ago by Karl Zeilhofer
cbf9c9
cleanup of static/extern (private/public) in all modules
6 days ago by Karl Zeilhofer
6ef78d
rename park_idler to engage_filament_pully for better clarity
6 days ago by Karl Zeilhofer
e0afa8
use constants for pin-toggling and asix selection, use bool instead of uint8
6 days ago by Karl Zeilhofer
463a0b
reformatted files with clang format
6 days ago by Karl Zeilhofer
2ebb0d
Fix compiler warnings.
18 days ago by Marek Bel
2a2029
Store separate bowden length for each filament.
18 days ago by Marek Bel
4d7d51
Give buttons symbolic names, use smaller underlying type.
23 days ago by Marek Bel
bb170e
Fix compile warnings.
25 days ago by Marek Bel
f291dd
go to park position fixed
8/29/2018 by PavelSindler
fbff44
eject filament commands, unload filament unpark
8/29/2018 by PavelSindler
0d0060
buildnr changed, line ending in motion.cpp changed to crlf
8/25/2018 by PavelSindler
736446
C0 command, buildnr update
8/25/2018 by PavelSindler
5993a0
Less sensitivity on stallguard for selector, service position for cleaning FINDA, decreased holding current for selector, slower loading speed and longer almost zero current load
8/21/2018 by akukan
1b36d7
Filament types (F0-4), reporting version (S1) and build number (S2)
8/8/2018 by Robert Pelnar
713294
Sync with master because of broken fork
8/3/2018 by Robert Pelnar
5bef70
Filament error state handling
8/2/2018 by akukan
4e794a
Start check if filament loaded, all error messages in red LED
8/1/2018 by akukan
2ec8f9
Switched pulley / idler order, setup, corrections
7/18/2018 by akukan
de23e0
Settings menu, experimental flex
7/5/2018 by akukan
bf2cec
Test for flex
6/26/2018 by akukan
19b43a
Unload of filament routine changed
6/18/2018 by akukan
869ee9
Loading / unloading with FINDA
6/13/2018 by akukan
ed5d14
v01
5/17/2018 by akukan
93f337
basic movements and communication with printer
3/22/2018 by akukan
ae18c7
communication
3/20/2018 by Robert Pelnar
2faa6e
motion
3/19/2018 by akukan
7a585f
ADDED motion.cpp
End of History
Diff View
File View

Show blame details
#include "motion.h"
#include "shr16.h"
#include "tmc2130.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <Arduino.h>
#include "main.h"
#include "mmctl.h"
#include "Buttons.h"
#include "permanent_storage.h"

const int selector_steps_after_homing = -3700;
const int idler_steps_after_homing = -130;

const int selector_steps = 2790 / 4;
const int idler_steps = 1420 / 4;    // 2 msteps = 180 / 4
const int idler_parking_steps = (idler_steps / 2) + 40;  // 40

const int bowden_length = 1000;
// endstop to tube  - 30 mm, 550 steps

int selector_steps_for_eject = 0;
int idler_steps_for_eject = 0;

int8_t filament_type[EXTRUDERS] = {-1, -1, -1, -1, -1};

int set_idler_direction(int _steps);
int set_selector_direction(int _steps);
int set_pulley_direction(int _steps);

void cut_filament();

void park_idler(bool _unpark);

void load_filament_inPrinter();
void load_filament_withSensor();

void do_pulley_step();
void do_idler_step();

void set_positions(int _current_extruder, int _next_extruder);

bool checkOk();

void cut_filament()
{
}

void set_positions(int _current_extruder, int _next_extruder)
{
    // steps to move to new position of idler and selector
    int _selector_steps = ((_current_extruder - _next_extruder) * selector_steps) * -1;
    int _idler_steps = (_current_extruder - _next_extruder) * idler_steps;

    // move both to new position
    move_proportional(_idler_steps, _selector_steps);
}

