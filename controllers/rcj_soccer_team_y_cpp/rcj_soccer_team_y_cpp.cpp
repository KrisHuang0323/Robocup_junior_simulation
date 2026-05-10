#include <webots/Robot.hpp>
#include <string>

// 匯入寫在其他檔案的機器人邏輯
#include "robot1.hpp"
#include "robot2.hpp"
#include "robot3.hpp"

using namespace std;
using namespace webots;

int main(int argc, char **argv) {
    // 建立與 Webots 模擬器的連線 (指標)
    Robot* robot = new Robot();
    
    // 判斷自己是幾號機器人
    string name = robot->getName(); 
    int robot_number = name[1] - '0'; 

    RCJSoccerRobot* robot_controller = nullptr;

    // 依據機器人編號，掛載對應的 AI 大腦
    if (robot_number == 1) {
        robot_controller = new MyRobot1(robot);
    } else if (robot_number == 2) {
        robot_controller = new MyRobot2(robot);
    } else {
        robot_controller = new MyRobot3(robot);
    }

    // 啟動主迴圈
    robot_controller->run();

    // 程式結束時清理記憶體
    delete robot_controller;
    delete robot;
    
    return 0;
}