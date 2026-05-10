#pragma once
#include "rcj_soccer_robot.hpp"

class MyRobot1 : public RCJSoccerRobot {
public:
    MyRobot1(Robot* r) : RCJSoccerRobot(r) {}

    void run() override {
        int count = 0;
        Motor* dribbler = robot->getMotor("dribbler motor");
        if (dribbler) {
            dribbler->setPosition(INFINITY);
            dribbler->setVelocity(0.0);
        }        

        int wander_timer = 0;
        int foward_timer = 0;
        while (robot->step(timeStep) != -1) {
            if (is_new_data()) {
                // Global information
                map<string, string> data = get_new_data();
                double heading = get_compass_heading();
                const double* robot_pos = gps->getValues();
                // cout << "Position 1: " << robot_pos[0] << ", 2: " << robot_pos[1] << endl;
                cout << "Sonar info: " << sonar_front->getValue() << endl;

                // x = v * t + 1/2 a * t ^2

                while (is_new_team_data()) {
                    map<string, string> team_data = get_new_team_data();
                }

                if (!is_new_ball_data()) {
                    // Stop any motion
                    // left_motor->setVelocity(0.0);
                    // right_motor->setVelocity(0.0);
                    // cout << "No ball data received!" << endl;

                    // How about wandering around to find the ball
                    if ( wander_timer <= 50 )
                    {
                        // try to turn around and look for it
                        left_motor->setVelocity(10.0);
                        right_motor->setVelocity(-10.0);
                        wander_timer++;
                    }
                    else if ( sonar_front->getValue() > 300 && sonar_left->getValue() > 300 && sonar_right->getValue() > 300 )
                    {
                        // try to go to other place to look for it
                        left_motor->setVelocity(10.0);
                        right_motor->setVelocity(10.0);                        
                    }
                    else
                    {
                        wander_timer = 0;
                    }
                    continue;
                }

                wander_timer = 0;

                const double* ball_direction = ball_receiver->getEmitterDirection();
                // cout << "Ball direction: " << ball_direction[1] << endl;
                int direction = get_direction(ball_direction);
                double strength = ball_receiver->getSignalStrength();
                ball_receiver->nextPacket();

                // Goal range x: (-0.3~0.3), y: (0.5~0.7)

                double dribbler_speed = 10.0;
                double left_speed = 0.0;
                double right_speed = 0.0;

                // Main logic part
                if (strength >= 250) {
                    if (count < 20) {
                        count++;
                    } else {
                        dribbler_speed = -100.0;
                        cout << "Fire the ball!" << endl;
                    }
                } else {
                    count = 0;
                    if (direction == 0) {
                        left_speed = 10.0;
                        right_speed = 10.0;
                    } else {
                        left_speed = direction * 10.0;
                        right_speed = direction * -10.0;
                    }
                }

                // Send message to the motors or other teammates
                if (dribbler) dribbler->setVelocity(dribbler_speed);
                left_motor->setVelocity(left_speed);
                right_motor->setVelocity(right_speed);
                send_data_to_team(player_id);
            }
        }
    }
};