from fr3_gripper import Gripper, GripperState


def main():
    gripper = Gripper("10.42.0.4")

    # # Homes the gripper (closes it and opens up again)
    gripper.homing()

    # # reads the gripper state
    gripper_state = gripper.readOnce()
    print(gripper_state)

    # moves the gripper to a specific width 
    # with a specified speed
    gripper_width = 0.05
    speed = 0.05
    gripper.move(gripper_width, speed)

    # Grasps an object and returns True if it succeeds
    gripper_width = 0.02  # m
    speed = 0.05  # m / s
    force = 15  # N
    epsilon_inner = 0.01  # m
    epsilon_outer = 0.01  # m
    gripper.grasp(gripper_width, speed, force, epsilon_inner, epsilon_outer)


if __name__ == "__main__":
    main()