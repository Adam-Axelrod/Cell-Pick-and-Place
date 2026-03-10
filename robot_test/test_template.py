import mecademicpy.robot as mdr

def main():
    # Create a robot instance
    robot = mdr.Robot()

    # Connect to the robot
    robot.Connect("192.168.0.100")

    robot.ActivateAndHome()
    robot.WaitHomed()

    robot.SetJointVel(25)

    present_pos_list = robot.GetRtTargetJointPos()
    print("Target Joint Positions:", present_pos_list)
     # Get current joint positions
    present_pos = {f"joint_{i+1}": p for i, p in enumerate(present_pos_list)}
    print("Current Joint Positions:", present_pos)

     # Move to a new joint position

    robot.DeactivateRobot()
    robot.Disconnect()
    robot.WaitDisconnected()
    print("Robot disconnected successfully.")


if __name__ == "__main__":
    main()