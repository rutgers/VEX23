#ifndef MAIN_H
#define MAIN_H
#include "main.h"
#endif
#ifndef AUTON_CPP
#define AUTON_CPP
#include "auton_util.cpp"
#endif

/*

    selector::auton == 1 : Red Front
    selector::auton == 2 : Red Back
    selector::auton == 3 : Do Nothing
    selector::auton == -1 : Blue Front
    selector::auton == -2 : Blue Back
    selector::auton == -3 : Do Nothing
    selector::auton == 0 : Skills
*/


#include "autoSelect/selection.h"

#define SMALL_BOT 2
#define BIG_BOT 3
#define SKAR_3 4

// Build Target
#define BUILD_TARGET BIG_BOT //SKAR_1 or SKAR_2

// Initial speed for auton
#define AUTON_INIT 165
#define COLOR_SIDE goal_color::RED //goal_color::RED or goal_color::BLUE


// Build Target
#if BUILD_TARGET == SMALL_BOT
    #include "SMALL_BOT.cpp"
#endif
#if BUILD_TARGET == BIG_BOT
    #include "BIG_BOT.cpp"
#endif
#if BUILD_TARGET == SKAR_3
    #include "SKAR_3.cpp"
#endif
