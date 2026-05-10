#pragma once
#include "rcj_soccer_robot.hpp"

class MyRobot3 : public RCJSoccerRobot {
public:
    MyRobot3(Robot* r) : RCJSoccerRobot(r) {}

    void run() override {
        while (robot->step(timeStep) != -1) {
            if (is_new_data()) {
                map<string, string> data = get_new_data();

                while (is_new_team_data()) {
                    map<string, string> team_data = get_new_team_data();
                }

                if (!is_new_ball_data()) {
                    left_motor->setVelocity(0.0);
                    right_motor->setVelocity(0.0);
                    continue;
                }

                const double* ball_direction = ball_receiver->getEmitterDirection();
                ball_receiver->nextPacket();

                // double heading = get_compass_heading();
                // const double* robot_pos = gps->getValues();

                int direction = get_direction(ball_direction);
                double left_speed = 0.0;
                double right_speed = 0.0;

                if (direction == 0) {
                    left_speed = 7.0;
                    right_speed = 7.0;
                } else {
                    left_speed = direction * 4.0;
                    right_speed = direction * -4.0;
                }

                // left_motor->setVelocity(left_speed);
                // right_motor->setVelocity(right_speed);

                send_data_to_team(player_id);
            }
        }
    }
};