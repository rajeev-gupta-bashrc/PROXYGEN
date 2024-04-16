import serial
import time, math

class MARS_ARM():
    def __init__(self):
        self.port = '/dev/ttyACM1'
        self.num_stepper = 4
        self.command_stepper = [0 for i in range(self.num_stepper)]
        self.channel = serial.Serial(port = self.port, timeout=0)
        time.sleep(2)
        self.motor_deg_to_steps = [2800/40, -130/13, -3200/90, 6500/90]
        
    def steps_input(self):
        print("enter %d space separated values to send run all the steppers", self.num_stepper)
        command_val = [0 for i in range(self.num_stepper)]
        try:
            command_val = [float(val) for val in input().split()]
        except ValueError:
            print("Bad values entered :(")
            self.keyboard_op()
            return
        comma_separated_string = ', '.join([str(val) for val in command_val])
        comma_separated_string+=','
        print(comma_separated_string)
        # convert to binary and then send
        self.channel.write(str.encode(comma_separated_string))
        print("Data sent on channel !!")
        
    def angle_to_steps(self, motor_steps):
        if(len(motor_steps)==self.num_stepper):
            for i in range(self.num_stepper):
                motor_steps[i]=round(self.motor_deg_to_steps[i]*motor_steps[i], 2)
            return motor_steps
        else:
            return [0 for i in range(self.num_stepper)]
      
    def angle_input(self):
        print("enter %d space separated values to send run all the steppers", self.num_stepper+1)
        command_val = [0 for i in range(self.num_stepper)]
        try:
            command_val = [float(val) for val in input().split()]
            command_val2 = self.angle_to_steps(command_val[:-1])
        except ValueError:
            print("Bad values entered :(")
            self.angle_input()
            return
        comma_separated_string = ','.join([str(val) for val in command_val2])
        comma_separated_string+=','
        comma_separated_string+=str(command_val[-1])
        comma_separated_string+=','
        print(comma_separated_string)
        # convert to binary and then send
        self.channel.write(str.encode(comma_separated_string))
        print("Data sent on channel !!")
        
    def go_to_default_angle(self, suc):
        command_val = [0 for i in range(self.num_stepper)]
        command_val[2] = -15
        command_val = self.angle_to_steps(command_val)
        comma_separated_string = ','.join([str(val) for val in command_val])
        comma_separated_string+=','
        comma_separated_string+=str(suc)
        comma_separated_string+=',0'
        comma_separated_string+=','
        print("angle in steps", comma_separated_string)
        # convert to binary and then send
        self.channel.write(str.encode(comma_separated_string))
        print("Data sent on channel !!")
        
    def go_to_default_angle0(self, suc):
        command_val = [0 for i in range(self.num_stepper)]
        comma_separated_string = ','.join([str(val) for val in command_val])
        comma_separated_string+=','
        comma_separated_string+=str(suc)
        comma_separated_string+=',0,'
        print("angle in steps", comma_separated_string)
        # convert to binary and then send
        self.channel.write(str.encode(comma_separated_string))
        print("Data sent on channel !!")
        
    def go_to_desired_angle(self, posn_angles, suc):
        print("angles in degrees to send ", posn_angles, suc)
        command_val = self.angle_to_steps(posn_angles)
        comma_separated_string = ','.join([str(val) for val in command_val])
        comma_separated_string+=','
        comma_separated_string+=str(suc)
        comma_separated_string+=',1,'
        print("angle in steps", comma_separated_string)
        # convert to binary and then send
        self.channel.write(str.encode(comma_separated_string))
        print("Data sent on channel !!")
        
        
    # def pick_place(obj_angle, goal_angle,self):
    #     self.go_to_default_angle()
    #     input("Press Enter to continue")
    #     self.go_to_desired_angle(obj_angle)
    #     input("Press Enter to continue")
    #     self.go_to_default_angle()
    #     input("Press Enter to continue")
    #     self.go_to_desired_angle(goal_angle)
    #     input("Press Enter to continue")
    #     self.go_to_default_angle()
        
        
if __name__ == '__main__':
    myarm = MARS_ARM()
    # myarm.steps_input()
    # myarm.angle_input()
    # obj_angle = [30, 60, 45]
    # goal_angle = [60 , 60 ,45]
    # myarm.pick_place(obj_angle, goal_angle)
    # myarm.go_to_desired_angle([-90, 0, 0, 0])
    
    
    arm = MARS_ARM()
    # obj_posn = [0, 0, 10, 25]
    # goal_posn = [0, 0, 50, 15]
    # to move robotic arm to object posn
    # set_position_client(obj_posn[0], obj_posn[1], obj_posn[2], 2)
    # input("Press Enter to send value to serial")
    # arm.go_to_desired_angle(obj_posn, 255)
    # input("Press Enter if arm has reached")
    # # go_to_default_posn()
    # input("Press Enter to send value toserial serial")
    # arm.go_to_default_angle(255)
    # input("Press Enter if arm has reached")
    
    # # to move robotic arm to goal posn
    # # set_position_client(goal_posn[0], goal_posn[1], goal_posn[2], 2)
    # input("Press Enter to send value to serial")
    # arm.go_to_desired_angle(goal_posn, 255)
    # input("Press Enter if arm has reached")
    # # go_to_default_posn()
    # arm.go_to_default_angle(0)
    # input("Press Enter if arm has reached")
    
    
    while(True):
        des_angle = [float(theta) for theta in input("enter 6 space separated values: ").split()]
        arm.go_to_desired_angle(des_angle[:-2], des_angle[-2])
    
    
