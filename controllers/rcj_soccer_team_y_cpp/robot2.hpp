#pragma once
#include "rcj_soccer_robot.hpp"

class MyRobot2 : public RCJSoccerRobot {
public:
    MyRobot2(Robot* r) : RCJSoccerRobot(r) {}

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

                // 註解掉馬達以符合原本 Python 設定
                // left_motor->setVelocity(left_speed);
                // right_motor->setVelocity(right_speed);

                send_data_to_team(player_id);
            }
        }
    }
};