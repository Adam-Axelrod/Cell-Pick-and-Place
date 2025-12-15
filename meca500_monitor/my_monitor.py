import mecademicpy.robot as mdr
import socket

class main():
    def __init__(self):
        self.robot = mdr.Robot()

    def connect(self):
        try:
            #self.monitor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.robot.Connect(address="192.168.0.100", monitor_mode=True)
            #self.monitor_socket.settimeout(0.01)
            print("Monitor Started")
        except mdr.MecademicException as e:
            print(f"Error connecting to robot monitor: {e}")
            return
    
    def get_observation(self):
        while True:
            #joints = self.robot.GetJoints()
            joints = self.robot.GetJoints()
            
            status = self.robot.GetStatusRobot()

            obs_dict = {
            "joint_1.pos": joints[0],
            "joint_2.pos": joints[1],
            "joint_3.pos": joints[2],
            "joint_4.pos": joints[3],
            "joint_5.pos": joints[4],
            "joint_6.pos": joints[5],
            }

            print(obs_dict)
    
    def disconnect(self):
        try:
            self.robot.Disconnect()
            self.robot.WaitDisconnected()
            #self.monitor_socket.close()
            print("Monitor Stopped")
        except mdr.MecademicException as e:
            print(f"Error disconnecting from robot monitor: {e}")
            return

if __name__ == "__main__":
    main_ = main()
    main_.connect()
    main_.get_observation()
    main_.disconnect()
    
