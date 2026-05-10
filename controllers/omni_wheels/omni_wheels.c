#include <stdio.h>
#include <webots/motor.h>
#include <webots/robot.h>

// ✨ 陣列大小從 3 改成 4
static WbDeviceTag wheels[4];

static double cmd[11][4] = {
  {1, 1, -1, -1}, // move left
  {-1, -1, 1, 1}, // move right
  {1, -1, -1, 1}, // move up
  {-1, 1, 1, -1}, // move down
  {1, 0, -1, 0}, // move left upper
  {0, 1, 0, -1}, // move left lower
  {-1, 0, 1, 0}, // move right lower
  {0, -1, 0, 1}, // move right upper
  {1, 1, 1, 1}, // turn right
  {-1, -1, -1, -1}, // turn left
  {0, 0, 0, 0}
};


static double SPEED_FACTOR = 10.0;

int main() {
  int i, j, k;

  wb_robot_init();

  for (i = 0; i < 4; i++) {
    char name[64];
    sprintf(name, "wheel%d", i + 1);
    wheels[i] = wb_robot_get_device(name);
    wb_motor_set_position(wheels[i], INFINITY);
  }

  while (1) {
    for (i = 0; i < 11; i++) {
      for (j = 0; j < 4; j++)
        wb_motor_set_velocity(wheels[j], cmd[i][j] * SPEED_FACTOR);

      for (k = 0; k < 100; k++)
        wb_robot_step(8);
    }
  }

  return 0;
}