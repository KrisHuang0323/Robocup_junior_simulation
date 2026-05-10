#pragma once
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Emitter.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <string>
#include <map>
#include <cmath>
#include <sstream>
#include <algorithm>

using namespace std;
using namespace webots;

// ==========================================
// 工具函式：方向計算與輕量級 JSON 解析
// ==========================================
int get_direction(const double* ball_vector) {
    if (ball_vector[1] >= -0.13 && ball_vector[1] <= 0.13) return 0;
    return (ball_vector[1] < 0) ? -1 : 1;
}

// 完美模擬 Python json.loads 的字串解析器
map<string, string> parse_json_to_dict(string json_str) {
    map<string, string> data;
    // 移除括號、引號與空白
    json_str.erase(remove(json_str.begin(), json_str.end(), '{'), json_str.end());
    json_str.erase(remove(json_str.begin(), json_str.end(), '}'), json_str.end());
    json_str.erase(remove(json_str.begin(), json_str.end(), '"'), json_str.end());
    json_str.erase(remove(json_str.begin(), json_str.end(), ' '), json_str.end());
    
    stringstream ss(json_str);
    string item;
    while (getline(ss, item, ',')) {
        size_t colon_pos = item.find(':');
        if (colon_pos != string::npos) {
            string key = item.substr(0, colon_pos);
            string value = item.substr(colon_pos + 1);
            data[key] = value;
        }
    }
    return data;
}

// ==========================================
// 基礎機器人類別 (RCJSoccerRobot)
// ==========================================
class RCJSoccerRobot {
protected:
    Robot* robot;
    int timeStep;
    string name;
    char team;
    int player_id;

    Receiver* supervisor_receiver;
    Emitter* team_emitter;
    Receiver* team_receiver;
    Receiver* ball_receiver;
    GPS* gps;
    Compass* compass;
    DistanceSensor* sonar_left;
    DistanceSensor* sonar_right;
    DistanceSensor* sonar_front;
    DistanceSensor* sonar_back;

    Motor* left_motor;
    Motor* right_motor;

public:
    RCJSoccerRobot(Robot* r) {
        robot = r;
        timeStep = (int)robot->getBasicTimeStep();
        name = robot->getName();
        team = name[0];
        player_id = name[1] - '0';

        supervisor_receiver = robot->getReceiver("supervisor receiver");
        supervisor_receiver->enable(timeStep);

        team_emitter = robot->getEmitter("team emitter");
        team_receiver = robot->getReceiver("team receiver");
        team_receiver->enable(timeStep);

        ball_receiver = robot->getReceiver("ball receiver");
        ball_receiver->enable(timeStep);

        gps = robot->getGPS("gps");
        gps->enable(timeStep);

        compass = robot->getCompass("compass");
        compass->enable(timeStep);

        sonar_left = robot->getDistanceSensor("distancesensor left");
        sonar_left->enable(timeStep);
        sonar_right = robot->getDistanceSensor("distancesensor right");
        sonar_right->enable(timeStep);
        sonar_front = robot->getDistanceSensor("distancesensor front");
        sonar_front->enable(timeStep);
        sonar_back = robot->getDistanceSensor("distancesensor back");
        sonar_back->enable(timeStep);

        left_motor = robot->getMotor("left wheel motor");
        right_motor = robot->getMotor("right wheel motor");

        left_motor->setPosition(INFINITY);
        right_motor->setPosition(INFINITY);
        left_motor->setVelocity(0.0);
        right_motor->setVelocity(0.0);
    }

    bool is_new_data() { return supervisor_receiver->getQueueLength() > 0; }
    map<string, string> get_new_data() {
        // 先取得原始位元組，轉成字元指標，再丟給 string
        const char* raw_bytes = (const char*)supervisor_receiver->getData();
        string raw_data(raw_bytes);
        supervisor_receiver->nextPacket();
        return parse_json_to_dict(raw_data);
    }

    bool is_new_team_data() { return team_receiver->getQueueLength() > 0; }
    map<string, string> get_new_team_data() {
        const char* raw_bytes = (const char*)team_receiver->getData();
        string raw_data(raw_bytes);
        team_receiver->nextPacket();
        return parse_json_to_dict(raw_data);
    }

    void send_data_to_team(int id) {
        string data = "{\"robot_id\": " + to_string(id) + "}";
        team_emitter->send(data.c_str(), data.length() + 1);
    }

    bool is_new_ball_data() { return ball_receiver->getQueueLength() > 0; }
    
    double get_compass_heading() {
        const double* compass_values = compass->getValues();
        double rad = atan2(compass_values[0], compass_values[1]) - (M_PI / 2.0);
        if (rad < -M_PI) rad = rad + (2.0 * M_PI);
        return rad;
    }

    virtual void run() = 0; 
};