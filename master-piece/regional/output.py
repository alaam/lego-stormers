from hub import light_matrix, motion_sensor
import runloop, sys
import motor_pair
import motor
import color_sensor
import math
import time
from hub import port

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def calculate(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class FLLBaseLib:
    def __init__(self,cfg=None):
        if cfg==None:
            cfg={
                "left_wheel_port":port.B,
                "right_wheel_port":port.A,
                "first_color_sensor_port":port.F,
                "second_color_sensor_port":port.E,
                "first_arm_port":port.C,
                "second_arm_port":port.D
                }




        self.left_wheel_port = cfg["left_wheel_port"] #port.B
        self.right_wheel_port = cfg["right_wheel_port"] #port.A
        self.first_color_sensor_port = cfg["first_color_sensor_port"] #port.F
        self.second_color_sensor_port = cfg["second_color_sensor_port"] #port.E
        self.first_arm_port = cfg["first_arm_port"] #port.D
        self.second_arm_port = cfg["second_arm_port"] #port.C

        self.cur_turn_deg=0
        motor_pair.pair(motor_pair.PAIR_1,self.left_wheel_port,self.right_wheel_port)
        motion_sensor.set_yaw_face(motion_sensor.FRONT)
        motion_sensor.reset_yaw(0)

    # cm, this is a constant for your robot
    WHEEL_CIRCUMFERENCE = 17.5
    TRACK = 8 #11.2 # cm - please measure your own robot.

    def follow_line(self,time_to_follow_in_ms):
        # Initialize PID controller with appropriate constants
        pid_controller = PIDController(Kp=0.1, Ki=0.01, Kd=0.05)

        # Set a base motor speed (adjust based on your requirements)
        base_speed = 30
        start_time=time.ticks_ms()
        # Example line-following loop
        while time.ticks_ms()-start_time < time_to_follow_in_ms:
            sensor_value = color_sensor.reflection(self.first_color_sensor_port)
            steering=math.floor(3/5)*sensor_value+base_speed
            # Setpoint is the center sensor value when the robot is perfectly on the line
            setpoint = 31
            # Calculate the error
            error = setpoint - sensor_value
            # Use PID controller to get the correction value
            pid_output = pid_controller.calculate(error)

            #print("steering:",steering)
            #print("time_elapsed:",time.ticks_ms()-start_time," given:",time_to_follow_in_ms)
            motor_pair.move(motor_pair.PAIR_1,math.floor(pid_output),velocity=100)
            #motor.run(self.right_wheel_port,math.floor(base_speed-pid_output),acceleration=100)
            #motor.run(self.left_wheel_port,math.floor(base_speed+pid_output),acceleration=100)
        motor_pair.stop(motor_pair.PAIR_1)
        motor.stop(self.left_wheel_port)
        motor.stop(self.right_wheel_port)
        print("follow line done,", time.ticks_ms()-start_time)

    # input must be in the same unit as WHEEL_CIRCUMFERENCE
    def deg_for_dist(self,distance_cm):
        # Add multiplier for gear ratio if needed
        return int((distance_cm/FLLBaseLib.WHEEL_CIRCUMFERENCE) * 360)

    async def move(self,distance_in_cm,velo=360,steer=0):
        distance_in_deg=self.deg_for_dist(distance_in_cm)
        await motor_pair.move_for_degrees(motor_pair.PAIR_1,distance_in_deg,steer,velocity=velo,stop=motor.BRAKE, acceleration=1000)

    async def reset_yaw(self):
        motion_sensor.reset_yaw(0)
        await runloop.until(motion_sensor.stable)

    # Function that returns true when the absolute yaw angle is 90 degrees
    def turn_done(self)->bool:
        #convert tuple decidegree into same format as in app and blocks
        return abs(motion_sensor.tilt_angles()[0] * 0.1) > abs(self.cur_turn_deg)

    async def turn_using_gyro(self,deg,speed=200):
        self.cur_turn_deg=deg
        #await runloop.sleep_ms(1500)
        motion_sensor.set_yaw_face(motion_sensor.FRONT)
        motion_sensor.reset_yaw(0)
        #runloop.sleep_ms(500)
        #await runloop.until(motion_sensor.stable)
        if deg<0:
            turn=-100
        else:
            turn=100
        start_yaw_angle=motion_sensor.tilt_angles()[0]
        print("from yaw:",motion_sensor.tilt_angles()[0], " start_yaw_angle:",start_yaw_angle)
        motor_pair.move(motor_pair.PAIR_1,turn,velocity=speed,acceleration=100)
        #await runloop.until(self.turn_done)
        curr=motion_sensor.tilt_angles()[0]
        diff = abs(curr)-abs(start_yaw_angle)
        print("from yaw:",abs(curr), " start_yaw_angle:",abs(start_yaw_angle)," diff:",abs(diff), "asked:",abs(deg*10))

        while abs(diff) <abs(deg*10): #yaw is decidegree hence multiply incoming angle by 10
            motor_pair.move(motor_pair.PAIR_1,turn,velocity=speed,acceleration=100)
            runloop.sleep_ms(100)
            #print("from yaw:",abs(curr), " start_yaw_angle:",abs(start_yaw_angle)," diff:",abs(diff), "asked:",abs(deg*10))
            curr=motion_sensor.tilt_angles()[0]
            diff = abs(curr)-abs(start_yaw_angle)

        print("after loop yaw:",motion_sensor.tilt_angles()[0])
        #runloop.sleep_ms(3000)
        #print("after 3sec yaw:",motion_sensor.tilt_angles()[0])
        motor_pair.stop(motor_pair.PAIR_1,stop=motor.COAST)
        #motion_sensor.reset_yaw(0)
        #motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,deg,0,360)

    async def turn(self,deg):
        motion_sensor.set_yaw_face(motion_sensor.FRONT)
        motion_sensor.reset_yaw(0)
        print("start yaw:",motion_sensor.tilt_angles()[0])
        if deg<0:
            await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,deg,0,100,acceleration=100)
        else:
            await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,deg,100,0,acceleration=100)
        await runloop.sleep_ms(3000)
        print("end yaw:",motion_sensor.tilt_angles()[0])

    async def spin_turn(self,degrees, mspeed=200):
        motion_sensor.set_yaw_face(motion_sensor.FRONT)
        motion_sensor.reset_yaw(0)
        print("start yaw:",motion_sensor.tilt_angles()[0])

        SPIN_CIRCUMFERENCE = FLLBaseLib.TRACK * math.pi
        # Add a multiplier for gear ratios if you’re using gears
        mot_degrees = int((SPIN_CIRCUMFERENCE/FLLBaseLib.WHEEL_CIRCUMFERENCE) * abs(degrees))
        if degrees > 0:
            # spin clockwise
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, mot_degrees, 100, velocity=mspeed)
        else:
            #spin counter clockwise
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, mot_degrees, -100, velocity=mspeed)
        #await runloop.sleep_ms(3000)
        print("end yaw:",motion_sensor.tilt_angles()[0])

    async def pivot_turn(self, robot_degrees, motor_speed=200):
        PIVOT_CIRCUMFERENCE = 2 *FLLBaseLib.TRACK * math.pi
        # Add a multiplier for gear ratios if you’re using gears
        motor_degrees = int((PIVOT_CIRCUMFERENCE/ FLLBaseLib.WHEEL_CIRCUMFERENCE) * abs(robot_degrees))
        if robot_degrees > 0:
            # pivot clockwise
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, motor_degrees, 50, velocity=motor_speed)
        else:
            #pivot counter clockwise
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, motor_degrees, -50, velocity=motor_speed)

    async def move_backward(self,distance_in_cm,velo=360,steer=0):
        await self.move(-distance_in_cm,velo=velo,steer=steer)

    async def move_forward(self,distance_in_cm,velo=360,steer=0):
        await self.move(distance_in_cm,velo=velo,steer=steer)

    async def turn_right(self,deg,speed=100): #200
        await self.spin_turn(deg,speed)
        #await self.pivot_turn(deg,speed)
        #await self.turn_using_gyro(deg,speed)

    async def turn_left(self,deg,speed=100): #200
        await self.spin_turn(-deg,speed)
        #await self.pivot_turn(-deg,speed)
        #await self.turn_using_gyro(deg,speed)

    async def back_arm_up(self,degree=130):
        await motor.run_to_absolute_position(self.second_arm_port,130,200,direction=motor.CLOCKWISE)
        print("back_arm_up done")

    async def back_arm_down(self,degree=200):
        await motor.run_to_absolute_position(self.second_arm_port,degree,200,direction=motor.COUNTERCLOCKWISE)
        print("back_arm_down done")

    async def front_arm_up(self,degree=4,speed=200):
        await motor.run_to_absolute_position(self.second_arm_port,degree,speed,direction=motor.CLOCKWISE)
        print("back_arm_up done")

    async def front_arm_down(self,degree=110,speed=200):
        await motor.run_to_absolute_position(self.second_arm_port,degree,speed,direction=motor.COUNTERCLOCKWISE)
        print("back_arm_down done")

    async def front_arm_go_relative(self,delta_degree,speed=200):
        await motor.run_to_relative_position(self.second_arm_port,delta_degree,speed)
        print("front_arm_go_relative done")

class FLL2023MasterPieceMissions(FLLBaseLib):
    def __init__(self, cfg):
        super().__init__(cfg)

    async def mission_1_3d_cinema_orig(self):
        await self.front_arm_up()
        await self.move_forward(30)
        await self.front_arm_down(160)
        # await self.move_backward(35)

    async def mission_1_3d_cinema(self):
        await self.front_arm_up()
        await self.move_forward(30)
        await self.front_arm_down(110)
        await self.move_backward(35)

    async def mission_2_theatre(self):
        speed=660
        await self.front_arm_up()
        await self.move_forward(67,600)
        await self.turn_left(43)
        for i in range(0,3):
            await self.front_arm_down(170,speed)
            await runloop.sleep_ms(1000)
            await self.move_backward(7)
            await self.front_arm_up()
            await self.move_forward(7)
        '''
        await self.front_arm_down(170)
        await runloop.sleep_ms(1000)
        await self.move_backward(4)
        await self.front_arm_up()

        await self.move_forward(4)
        await self.front_arm_down(170)
        await runloop.sleep_ms(1000)
        await self.front_arm_up()
        '''
        # await self.turn_right(45)
        # await self.move_backward(65)

    async def mission_1_and_2(self):
        await self.mission_1_3d_cinema()
        await self.move_backward(15)
        await self.turn_right(45)
        await self.move_forward(20)
        await self.turn_left(45)
        await self.mission_2_theatre()

    async def mission_4_master_piece(self):
        #it going to first move forward
        await self.move_forward(13)
        await self.turn_right(85)
        await self.move_forward(61)
        await self.turn_left(57)
        await self.move_forward(68)
        await self.move_backward(25)
        if False:
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,100,100)
            #await runloop.sleep_ms(1000)
            await move(1300,0,360)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,100,-40)
            await move(1030,0,360)
            await move(-360,0,360)

    async def mission_4_master_piece_v1(self):
        #it going to first move forward
        await self.move_forward(7)
        await self.turn_right(75)
        await self.move_forward(63)
        await self.turn_left(41)
        await self.move_forward(56)
        await self.move_backward(25)
        if False:
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,100,100)
            #await runloop.sleep_ms(1000)
            await move(1300,0,360)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,100,-40)
            await move(1030,0,360)
            await move(-360,0,360)

    async def mission_5_AR(self):
        #It turns right to go towards AR and turn the lever.
        await self.turn_right(30)
        await self.move_forward(36)
        #turning the lever from left to right by pushing straight
        await self.turn_right(57)
        if False:
            await self.move_forward(10)
            await self.turn_left(10)
            await self.move_forward(10)
            await self.turn_right(10)
            await self.move_forward(36)
        elif False:
            await self.move_forward(46)
        elif True:
            await self.move_forward(50)
            #next mission music concert
            await self.turn_right(42)
            await self.move_forward(60)

    async def mission_7_music_concert(self):
        #pass
        await self.move_forward(70,600)
        await self.turn_right(45)
        await self.move_forward(10)
        await self.move_backward(10)
        await self.turn_left(60)
        await self.move_backward(70,600)

    async def mission_13_printer(self):
        await self.move_forward(36)
        await self.move_backward(45)

    async def mission_8_camera(self):
        await self.move_forward(35,600)
        await self.move_backward(45,800)

    async def mission_9_boat_and_flip_camera_handle_2(self):
        #await self.move_forward(48)
        await self.front_arm_down(200)
        await runloop.sleep_ms(3000)
        #await self.move_backward(23)
        #await self.move_forward(27)
        await self.front_arm_up()

    async def mission_9_boat_and_flip_camera_handle(self):
        await self.front_arm_up()
        await self.move_forward(50)
        await self.front_arm_down(160)
        await self.move_backward(23)

        if True:
            await self.move_forward(25)
            await self.front_arm_up()
            await self.move_backward(55)
        else:
            await self.front_arm_up()
            await self.move_backward(20)

    async def mission_10_skate_board(self):
        await self.front_arm_up()
        #await self.front_arm_go_relative(250)
        #return
        await self.move_forward(20,500)
        await self.turn_right(90)
        await self.move_forward(70,500)
        await self.turn_left(78)
        await self.move_forward(45)

        if True:
            await self.front_arm_down(110)
            #await self.front_arm_go_relative(-75)
            await runloop.sleep_ms(1000)
            await self.front_arm_up()
            await self.move_backward(50,600)
            await self.turn_left(110)
            await self.move_forward(70,600)

async def main():
    # write your code here
    #light_matrix.write("Hi!")
    cfg={
        "left_wheel_port":port.B,
        "right_wheel_port":port.A,
        "first_color_sensor_port":port.F,
        "second_color_sensor_port":port.E,
        "first_arm_port":port.C,
        "second_arm_port":port.D
        }
    masterpiece_missions=FLL2023MasterPieceMissions(cfg)
    #await ls_robot.follow_line(10000)
    await masterpiece_missions.mission_4_master_piece()
    await masterpiece_missions.mission_5_AR()
    #await masterpiece_missions.mission_13_printer()
    #await masterpiece_missions.mission_7_music_concert()
    #await masterpiece_missions.mission_8_camera()
    #await masterpiece_missions.back_arm_up()
    #await masterpiece_missions.back_arm_down()
    #await masterpiece_missions.mission_9_boat_and_flip_camera_handle()
    #await masterpiece_missions.mission_1_3d_cinema()
    #await masterpiece_missions.mission_2_theatre()
    # await masterpiece_missions.mission_1_and_2()
    # await masterpiece_missions.mission_10_skate_board()

    if False:
        print("calling move reverse")
        await ls_robot.move_backward(20)
        print("calling move fwd")
        await ls_robot.move_forward(20)
        #await runloop.sleep_ms(100)
        print("calling move turn left")
        await ls_robot.turn_left(90)
        #await runloop.sleep_ms(2000)
        print("calling move turn right")
        await ls_robot.turn_right(90)
        print("calling move end of main")
    #sys.exit(0)

#main()
runloop.run(main())
