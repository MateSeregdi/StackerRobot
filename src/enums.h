#ifndef ENUMS_H
#define ENUMS_H
enum MotorMode { SPEED_MODE, POSITION_MODE, IDLE };
enum LimitSwitchSource {NONE, FORK_FORWARD_END, FORK_FORWARD_START, LEAD_SCREW};
static const char* limitSwitchSourceStrings[] = {"Error, you shouldnt see this", "fork forward at the end", "fork forward at the start", "lead_screw"};
#endif
