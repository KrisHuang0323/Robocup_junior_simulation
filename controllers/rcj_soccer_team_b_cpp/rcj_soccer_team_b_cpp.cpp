#include <webots/Robot.hpp>
#include <string>

// 匯入藍隊球員邏輯
#include "robot1.hpp"
#include "robot2.hpp"
#include "robot3.hpp"

using namespace std;
using namespace webots;

int main(int argc, char **argv) {
    // 建立與 Webots 模擬器的連線
    Robot* robot = new Robot();
    
    // 判斷自己是藍隊幾號機器人 (B1, B2, B3)
    string name = robot->getName(); 
    int robot_number = name[1] - '0'; 

    RCJSoccerRobot* robot_controller = nullptr;

    // 依據機器人編號掛載大腦
    if (robot_number == 1) {
        robot_controller = new MyRobot1(robot);
    } else if (robot_number == 2) {
        robot_controller = new MyRobot2(robot);
    } else {
        robot_controller = new MyRobot3(robot);
    }

    // 啟動主迴圈
    robot_controller->run();

    // 釋放記憶體
    delete robot_controller;
    delete robot;
    
    return 0;
}