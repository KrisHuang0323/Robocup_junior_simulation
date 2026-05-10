#pragma once
#define _USE_MATH_DEFINES
#include "rcj_soccer_robot.hpp"
#include <map>
#include <string>
#include <array>
#include <math.h>

class MyRobot1 : public RCJSoccerRobot {
public:
    MyRobot1(Robot* r) : RCJSoccerRobot(r) {
        cmd["move backward"] = {1, 1, -1, -1};
        cmd["move forward"] = {-1, -1, 1, 1};
        cmd["move left"] = {1, -1, -1, 1};
        cmd["move right"] = {-1, 1, 1, -1};
        cmd["move left back"] = {1, 0, -1, 0};
        cmd["move right back"] = {0, 1, 0, -1};
        cmd["move right forward"] = {-1, 0, 1, 0};
        cmd["move left forward"] = {0, -1, 0, 1};
        cmd["turn right"] = {1, 1, 1, 1};
        cmd["turn left"] = {-1, -1, -1, -1};
        cmd["stop"] = {0, 0, 0, 0};
    }

    std::map<std::string, std::array<double, 4>> cmd;

    double shortest_angle_diff(double target, double source) {
        double diff = target - source;

        // Regular the difference to be within the range of [0, 180]
        while (diff <= -180) diff += 360;
        while (diff > 180) diff -= 360;
        return diff;
    }

    // Get next new relative information of the goal
    void get_new_goal_info_relative(const double* robot_pos, double heading, double* new_info)
    {
        double goal_pos[] = {0.0, -0.65};
        // convert the goal position to the robot's local coordinate
        double local_goal_x = (goal_pos[0] - robot_pos[0]) * cos(heading - 90 * M_PI / 180) + (goal_pos[1] - robot_pos[1]) * sin(heading - 90 * M_PI / 180);
        double local_goal_y = -(goal_pos[0] - robot_pos[0]) * sin(heading - 90 * M_PI / 180) + (goal_pos[1] - robot_pos[1]) * cos(heading - 90 * M_PI / 180);
        new_info[0] = 1.0 / (pow(local_goal_x, 2) + pow(local_goal_y, 2));
        new_info[1] = atan2(local_goal_y, local_goal_x);
        // cout << "Robot position: (" << robot_pos[0] << ", " << robot_pos[1] << "), Heading: " << heading / M_PI * 180.0 << " degree, Goal relative info: (" << new_info[0] << ", " << new_info[1] / M_PI * 180.0 << " degree)" << endl;
    }
    
    // Return the new information relative to the robot after executing the action
    void get_new_relative_info(double* robot_action, double* info_relative, double* result)
    {
        if (robot_action[0] != 0.0)
        {
            result[0] = info_relative[0];
            result[1] = info_relative[1] - robot_action[0];
            return;
        }

        double goal_strength = sqrt(1.0 / info_relative[0]);
        double goal_direction = info_relative[1];
        double goal_vector[] = {goal_strength * cos(goal_direction), goal_strength * sin(goal_direction)};
        double final_vector[] = {goal_vector[0] - robot_action[1], goal_vector[1] - robot_action[2]};
        cout << "Goal vector: (" << goal_vector[0] << ", " << goal_vector[1] << "), Final vector: (" << final_vector[0] << ", " << final_vector[1] << ")" << endl;
        result[0] = 1.0 / (pow(final_vector[0], 2) + pow(final_vector[1], 2));
        result[1] = atan2(final_vector[1], final_vector[0]);
    }

    double GradingLogic(double* ball_info, double* goal_info, double* pre_ball_info, double* pre_goal_info)
    {
        double grade = 0.0;
        if (pre_ball_info[0] < 80.0)
        {
            grade += ball_info[0] * ball_info[0];
        }
        else if (fabs(shortest_angle_diff(pre_ball_info[1], 0.0)) > 15.0 / 180.0 * M_PI)
        {
            // cout << "Ball direction is too large: " << pre_ball_info[1] / M_PI * 180.0 << " degree" << endl;
            grade -= fabs(shortest_angle_diff(ball_info[1] / M_PI * 180.0, 0.0));
        }
        else if (pre_ball_info[0] < 120.0) {
            // cout << "Ball strength is too weak: " << pre_ball_info[0] << endl;
            grade += ball_info[0] * ball_info[0];
            grade -= fabs(shortest_angle_diff(ball_info[1] / M_PI * 180.0, 0.0));
        } 
        else if (fabs(shortest_angle_diff(pre_goal_info[1], 0.0)) > 15.0 / 180.0 * M_PI) {
            cout << "Goal direction is too large: " << pre_goal_info[1] / M_PI * 180.0 << " degree" << endl;
            grade -= fabs(shortest_angle_diff(goal_info[1] / M_PI * 180.0, 0.0));
        }
        else {
            // cout << "Ball is in good position! goal strength: " << goal_info[0] << ", Direction: " << goal_info[1] / M_PI * 180.0 << " degree" << endl;
            grade += goal_info[0] * goal_info[0];
            grade -= fabs(shortest_angle_diff(goal_info[1] / M_PI * 180.0, 0.0));
        }
        return grade;
    }

    double CalculateGrade(string cmd_key, double* pre_goal_info_relative, double* pre_ball_info_relative)
    {
        // Possible next heading after executing the command
        double robot_action[] = {0.0, 0.0, 0.0};
        double disp = 0.04;
        if (cmd_key == "turn right") {
            robot_action[0] = -0.1;
        } else if (cmd_key == "turn left") {
            robot_action[0] = 0.1;
        }
        else
        {
            if (cmd_key == "move forward") {
                robot_action[1] = disp;
                robot_action[2] = 0.0;
            } else if (cmd_key == "move backward") {
                robot_action[1] = -disp;
                robot_action[2] = 0.0;
            }
            else if (cmd_key == "move left") {
                robot_action[1] = 0.0;
                robot_action[2] = disp;
            } else if (cmd_key == "move right") {
                robot_action[1] = 0.0;
                robot_action[2] = -disp;
            } else if (cmd_key == "move left back") {
                robot_action[1] = - disp * cos(M_PI / 4);
                robot_action[2] = disp * sin(M_PI / 4);
            } else if (cmd_key == "move right back") {
                robot_action[1] = - disp * cos(M_PI / 4);
                robot_action[2] = - disp * sin(M_PI / 4);
            } else if (cmd_key == "move right forward") {
                robot_action[1] = - disp * cos(M_PI / 4);
                robot_action[2] = disp * sin(M_PI / 4);
            } else if (cmd_key == "move left forward") {
                robot_action[1] = disp * cos(M_PI / 4);
                robot_action[2] = disp * sin(M_PI / 4);
            }
        }

        // Calculate the new relative info after executing the command
        double new_goal_info_relative[2];
        get_new_relative_info(robot_action, pre_goal_info_relative, new_goal_info_relative);
        double new_ball_info_relative[2];
        get_new_relative_info(robot_action, pre_ball_info_relative, new_ball_info_relative);

        // Calculate new grade accordiny to the new relative info
        cout << "Evaluating command: " << cmd_key << ", new goal relative info: (" << new_goal_info_relative[0] << ", " << new_goal_info_relative[1] / M_PI * 180.0 << " degree), new ball relative info: (" << new_ball_info_relative[0] << ", " << new_ball_info_relative[1] / M_PI * 180.0 << " degree)" << endl;
        return GradingLogic(new_ball_info_relative, new_goal_info_relative, pre_ball_info_relative, pre_goal_info_relative);
    }

    void run() override {
        int count = 0;
        Motor* dribbler = robot->getMotor("dribbler motor");
        if (dribbler) {
            dribbler->setPosition(INFINITY);
            dribbler->setVelocity(0.0);
        }

        int find_count = 0;
        int straight_count = 0;
        while (robot->step(timeStep) != -1) {
            if (is_new_data()) {
                map<string, string> data = get_new_data();

                while (is_new_team_data()) {
                    map<string, string> team_data = get_new_team_data();
                }

                std::string command = "stop";
                double vel_const = 8.0;
                double dribbler_speed = 50.0;

                const double* ball_direction;
                double strength = 0.0001;
                if (is_new_ball_data()) {
                    ball_direction = ball_receiver->getEmitterDirection();
                    strength = ball_receiver->getSignalStrength();
                    cout << "Ball direction: (" << ball_direction[0] << ", " << ball_direction[1] << ", " << ball_direction[2] << "), Strength: " << strength << endl;
                    ball_receiver->nextPacket();
                } else {
                    ball_direction = new double[2]{0.0, 0.0};
                }
                double heading = get_compass_heading();
                const double* robot_pos = gps->getValues();

                // Clear the console
                cout << "\033[2J\033[1;1H";

                // Calculate the relative information of the goal                
                double goal_info_relative[2];
                get_new_goal_info_relative(robot_pos, heading, goal_info_relative);

                // Controller logic
                if (false) { // The reactive controller
                    if (strength == 0.0001) {
                        if ( find_count < 50 ) {
                            command = "turn right";
                            find_count++;
                        }
                        else {
                            if ( straight_count < 50 ) {
                                if ( sonar_front->getValue() > 300.0 ) {
                                    command = "move forward";
                                    straight_count++;
                                }
                                else {
                                    command = "move backward";
                                    straight_count++;
                                }
                            }
                            else {
                                find_count = 0;
                                straight_count = 0;
                            }
                        }
                        
                        wheel1_motor->setVelocity(cmd[command][0] * vel_const);
                        wheel2_motor->setVelocity(cmd[command][1] * vel_const);
                        wheel3_motor->setVelocity(cmd[command][2] * vel_const);
                        wheel4_motor->setVelocity(cmd[command][3] * vel_const);
                        continue;
                    }

                    find_count = 0;      
                    straight_count = 0;              
                    int direction = get_direction(ball_direction);
                    if (strength < 120) {
                        std::string command;
                        if (direction == 0) {
                            command = "move forward";
                        } 
                        else if (direction < 0) {
                            command = "turn right";
                        }
                        else {
                            command = "turn left";
                        }
                        wheel1_motor->setVelocity(cmd[command][0] * vel_const);
                        wheel2_motor->setVelocity(cmd[command][1] * vel_const);
                        wheel3_motor->setVelocity(cmd[command][2] * vel_const);
                        wheel4_motor->setVelocity(cmd[command][3] * vel_const);
                    }
                }              
                else if (true) { // The grading controller
                    double ball_info_relative[2];
                    ball_info_relative[0] = strength;
                    ball_info_relative[1] = atan2(ball_direction[1], ball_direction[0]);

                    double max_grade = -100000.0;
                    std::string best_command = "stop";
                    for (auto const& pair : cmd) {
                        double grade = CalculateGrade(pair.first, goal_info_relative, ball_info_relative);
                        if (grade > max_grade) {
                            max_grade = grade;
                            best_command = pair.first;
                        }
                    }
                    wheel1_motor->setVelocity(cmd[best_command][0] * vel_const);
                    wheel2_motor->setVelocity(cmd[best_command][1] * vel_const);
                    wheel3_motor->setVelocity(cmd[best_command][2] * vel_const);
                    wheel4_motor->setVelocity(cmd[best_command][3] * vel_const);
                    // cout << "Best command: " << best_command << ", Grade: " << max_grade << endl;
                }
                else if (false) {  // The PID controller
                    double target_heading = atan2(ball_direction[1], ball_direction[0]);
                    double heading_error = shortest_angle_diff(target_heading, heading);
                    double Kp = 5.0;
                    double command_value = Kp * heading_error;
                    if (command_value > 1.0) command_value = 1.0;
                    if (command_value < -1.0) command_value = -1.0;

                    wheel1_motor->setVelocity(command_value * vel_const);
                    wheel2_motor->setVelocity(command_value * vel_const);
                    wheel3_motor->setVelocity(command_value * vel_const);
                    wheel4_motor->setVelocity(command_value * vel_const);            
                }
                else if (false) { // The PID controller witn distance
                    double target_heading = atan2(ball_direction[1], ball_direction[0]);
                    double heading_error = shortest_angle_diff(target_heading, heading);
                    double distance = sqrt(pow(ball_direction[0], 2) + pow(ball_direction[1], 2));
                    double Kp_heading = 5.0;
                    double Kp_distance = 0.5;
                    double command_value_heading = Kp_heading * heading_error;
                    double command_value_distance = Kp_distance * (1.0 / distance - 1.0 / 80.0);
                    if (command_value_heading > 1.0) command_value_heading = 1.0;
                    if (command_value_heading < -1.0) command_value_heading = -1.0;
                    if (command_value_distance > 1.0) command_value_distance = 1.0;
                    if (command_value_distance < -1.0) command_value_distance = -1.0;

                    wheel1_motor->setVelocity((command_value_distance + command_value_heading) * vel_const);
                    wheel2_motor->setVelocity((command_value_distance + command_value_heading) * vel_const);
                    wheel3_motor->setVelocity((command_value_distance + command_value_heading) * vel_const);
                    wheel4_motor->setVelocity((command_value_distance + command_value_heading) * vel_const);            
                }

                // If it is close to the goal, try to make a shot
                if (goal_info_relative[0] > 100.0 && fabs(goal_info_relative[1]) < 15.0 / 180.0 * M_PI) {
                    cout << "Close to the goal, try to make a shot!" << endl;
                    dribbler_speed = -100.0;
                }

                if (dribbler) dribbler->setVelocity(dribbler_speed);
                send_data_to_team(player_id);
            }
        }
    }
};