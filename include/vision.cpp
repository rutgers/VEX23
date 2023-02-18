#ifndef MAIN_H
#define MAIN_H
#include "main.h"
#endif
#ifndef OKAPI_H
#define OKAPI_H
#include "okapi/api.hpp"
#endif

enum goal_color
{
    RED,
    YELLOW,
    BLUE    
};

class GoalCamera {
    public:
    std::shared_ptr<pros::Vision> vision_sensor;

    GoalCamera(int port) {
        vision_sensor.reset(new pros::Vision(port, pros::E_VISION_ZERO_CENTER));

        pros::vision_signature_s_t RED_SIG =
            pros::Vision::signature_from_utility(RED, 10091, 11357, 10724, -2207, -1527, -1867, 3.000, 0);
            //pros::Vision::signature_from_utility(RED, 8717, 11443, 10080, -2329, -1959, -2144, 3.000, 0);

        pros::vision_signature_s_t YELLOW_SIG =
            pros::Vision::signature_from_utility(YELLOW, 3295, 5517, 4406, -4699, -4415, -4557, 3.000, 0);

        pros::vision_signature_s_t BLUE_SIG =
            pros::Vision::signature_from_utility(BLUE, -1921, -1195, -1558, 5113, 7113, 6113, 3.000, 0);

        vision_sensor->set_signature(RED, &RED_SIG);
        vision_sensor->set_signature(YELLOW, &YELLOW_SIG);
        vision_sensor->set_signature(BLUE, &BLUE_SIG);
    }

    //Recieving object by doing 
    //int x, y
    //tie(x, y) = sensor.get_by_sig(RED)
    std::tuple<int, int> get_by_sig(goal_color c) {
        pros::vision_object_s_t object = vision_sensor->get_by_sig(0, c);

        double x = object.x_middle_coord;
        double y = object.y_middle_coord;
        return std::make_tuple(x, y);
    }
};
