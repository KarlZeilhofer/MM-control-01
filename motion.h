// motion.h

#ifndef _MOTION_h
#define _MOTION_h

#include "config.h"
#include <inttypes.h>
#include <stdbool.h>

extern int8_t filament_type[EXTRUDERS];



void home();
bool home_idler();
bool home_selector();
void engage_filament_pully(bool engage);

void load_filament_withSensor();
void load_filament_intoExtruder();

void unload_filament_withSensor();
void set_positions(int _current_extruder, int _next_extruder);
void init_Pulley();
void do_pulley_step();

void set_pulley_dir_pull();
void set_pulley_dir_push();

void move(int _idler, int _selector, int _pulley);
void move_idler(int steps);
void move_selector(int steps);
void move_pulley(int steps);

void move_proportional(int _idler, int _selector);
void eject_filament(int extruder);
void recover_after_eject();

#ifdef TESTING
void moveTest(uint8_t axis, int steps, int speed, bool rehomeOnFail = true);
#endif

#endif
