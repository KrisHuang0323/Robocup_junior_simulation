from controller import Robot

robot = Robot()
y_g_r_emitter = robot.getDevice("yellow_goal_emitter_right")
y_g_l_emitter = robot.getDevice("yellow_goal_emitter_left")
b_g_r_emitter = robot.getDevice("blue_goal_emitter_right")
b_g_l_emitter = robot.getDevice("blue_goal_emitter_left")

data = "x"  # Packet cannot be empty

while robot.step(32) != -1:
    y_g_r_emitter.send(data)
    y_g_l_emitter.send(data)
    b_g_r_emitter.send(data)
    b_g_l_emitter.send(data)
