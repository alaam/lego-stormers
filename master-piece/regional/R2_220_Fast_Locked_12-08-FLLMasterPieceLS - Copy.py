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

        self.first_arm_init_deg = motor.absolute_position(self.first_arm_port)
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

    async def front_arm_up(self,degree=10,speed=200,force=False):
        #degree = (degree+self.first_arm_init_deg)%360
        cur_pos=motor.absolute_position(self.second_arm_port)
        #hack needed sometime motor does overdrive
        if not force and (cur_pos> 350 or cur_pos < 25): return
        await motor.run_to_absolute_position(self.second_arm_port,degree,speed,direction=motor.CLOCKWISE)
        print("back_arm_up done")


    async def front_arm_down(self,degree=180,speed=200,acce=1000):
        #degree = (degree+self.first_arm_init_deg)%360
        await motor.run_to_absolute_position(self.second_arm_port,degree,speed,direction=motor.COUNTERCLOCKWISE,acceleration=acce)
        print("back_arm_down done")


    async def front_arm_go_relative(self,delta_degree,speed=200):
        await motor.run_to_relative_position(self.second_arm_port,delta_degree,speed)
        print("front_arm_go_relative done")

class FLL2023MasterPieceMissions(FLLBaseLib):
    def __init__(self, cfg):
        super().__init__(cfg)

    async def mission_1_3d_cinema(self):
        await self.front_arm_up()
        await self.move_forward(28,steer=-3) #30
        await self.front_arm_down() #110 too low but works
        await self.front_arm_up()
        await self.move_backward(30)

    async def mission_2_theatre(self):
        speed=660
        if False:
            await self.move_forward(67,600)
        else:
            await self.front_arm_up()
            await self.move_forward(15)
            await self.turn_right(90)
            await self.move_forward(27)
            await self.turn_left(90)
            await self.move_forward(42,600) 
            await self.turn_left(30)
        for i in range(0,3):
            await self.front_arm_down(170,speed)
            await runloop.sleep_ms(1000)
            await self.move_backward(7)
            await self.front_arm_up()
            await self.move_forward(7)
        
        await self.move_backward(7)
        await self.turn_right(50)
        await self.move_backward(70)
        #next mission to immersive exp

    async def mission_1_and_2_slow_working(self):
        #{ await self.mission_1_3d_cinema()
        await self.front_arm_up()
        await self.move_forward(28,steer=-2) #30
        await self.front_arm_down(110) #110 works but too low
        await self.front_arm_up()
        await self.move_backward(25)
        #}

        #{ await self.mission_2_theatre()
        speed=660
        two_step=True
        await self.turn_right(79)

        await self.move_forward(53)
        await self.turn_left(110)
        if two_step:
            await self.move_forward(20,600)
            await self.move_forward(24,steer=3)
        else:
            await self.move_forward(49,500,steer=2)

        num_try=2
        for i in range(0,num_try):
            await self.front_arm_down(110,speed)
            await runloop.sleep_ms(500)
            await self.move_backward(5)
            await self.front_arm_up()
            if i < num_try-1:
                await self.move_forward(5)
        #await self.move_backward(7)
        #}
        #{mission_3_immersive_experience
        #await self.turn_right(80)
        await self.move_forward(50,steer=20)
        await self.move_forward(25)
        await self.turn_left(120)
        if not two_step:
            await self.move_forward(5)

        await self.front_arm_down(90) #90
        await runloop.sleep_ms(1000)
        await self.front_arm_up()
        #await runloop.sleep_ms(1000)
        #await self.move_backward(80,steer=3)
        await self.move_backward(35,600)
        await self.turn_left(110)
        await self.move_forward(70,600)
        #}


    async def mission_1_and_2_v1(self):
        st=time.time()
        print("start mission_1_and_2:",st)
        speed=660
        #{ await self.mission_1_3d_cinema()
        await self.front_arm_up()
        await self.move_forward(27,steer=-2) #30
        two_try=True
        await self.front_arm_down(120,speed,acce=5000) #110 works but too low
        await self.front_arm_up()
        if two_try:
            await self.front_arm_down(120,speed,acce=5000) #110 works but too low
            await self.front_arm_up()

        await self.move_backward(25)
        #}

        #{ await self.mission_2_theatre()
        await self.turn_right(79)
        await self.move_forward(16)
        await self.turn_left(65)
        await self.move_forward(50,500,steer=0)
        await self.turn_left(35)
        await self.move_forward(2)
        num_try=2
        for i in range(0,num_try):
            await self.front_arm_down(90,speed=350,acce=5000)
            await runloop.sleep_ms(500)
            await self.move_backward(5)
            await self.front_arm_up()
            if i < num_try-1:
                await self.move_forward(5)
                pass
        
        #await self.move_backward(7)
        #}
        
        #{mission_3_immersive_experience
        await self.move_forward(7)
        await self.turn_right(70)
        await self.move_forward(10)
        await self.turn_right(50)
        await self.move_forward(38)
        await self.turn_left(88)
        await self.move_forward(5)
        
        await self.front_arm_down(90,acce=5000) #90
        await runloop.sleep_ms(1000)
        await self.front_arm_up()
        #await runloop.sleep_ms(1000)
        #await self.move_backward(80,steer=3)
        await self.move_backward(35,600)
        await self.turn_left(125)
        await self.move_forward(70,600)
        #}
        print("end mission_1_and_2: secs:",time.time()-st)


    async def mission_1_and_2_and_3(self):
        st=time.time()
        print("start mission_1_and_2_and_3:",st)
        speed=660
        #{ await self.mission_1_3d_cinema()
        await self.front_arm_up()
        await self.move_forward(27,600,steer=-2) #30
        two_try=False
        await self.front_arm_down(120,speed,acce=5000) #110 works but too low
        await self.front_arm_up()
        if two_try:
            await self.front_arm_down(120,speed,acce=5000) #110 works but too low
            await self.front_arm_up()

        await self.move_backward(25,660)
        #}

        #{ await self.mission_2_theatre()
        await self.turn_right(79)
        await self.move_forward(16,500)
        await self.turn_left(65)
        await self.move_forward(50,500,steer=0)
        await self.turn_left(32)
        await self.move_forward(2)
        num_try=1
        for i in range(0,num_try):
            await self.front_arm_down(90,speed=350,acce=5000)
            await runloop.sleep_ms(500)
            await self.move_backward(6)
            await self.front_arm_up()
            if i < num_try-1:
                await self.move_forward(6)
                pass

        #await self.move_backward(7)
        #}

        #{mission_3_immersive_experience
        await self.move_forward(3)
        await self.turn_right(70)
        await self.move_forward(10)
        await self.turn_right(50)
        await self.move_forward(39)
        await self.turn_left(90)
        await self.move_forward(9)

        await self.front_arm_down(130,acce=5000) #90
        await runloop.sleep_ms(1000)
        await self.front_arm_up()
        #await runloop.sleep_ms(1000)
        #await self.move_backward(80,steer=3)
        await self.move_backward(35,600)
        await self.turn_left(125)
        await self.move_forward(70,600)
        #}
        print("end mission_1_and_2_and_3: secs:",time.time()-st)


    async def mission_3_immersive_experience(self):
        await self.front_arm_up()
        #await self.front_arm_go_relative(250)
        #return
        await self.move_forward(20,500)
        await self.turn_right(90)
        await self.move_forward(67,500)
        await self.turn_left(80)
        await self.move_forward(45)

        if True:
            await self.front_arm_down() #90
            #await self.front_arm_go_relative(-75)
            await runloop.sleep_ms(1000)
            await self.front_arm_up()
            await self.move_backward(50,600)
            await self.turn_left(110)
            await self.move_forward(70,600)



    async def mission_4_master_piece_v1(self):
        #it going to first move forward
        await self.move_forward(13)
        await self.turn_right(85)
        await self.move_forward(61)
        await self.turn_left(57)
        await self.move_forward(68)
        #await self.move_backward(25) moving to AR mission for dropping the expert
        if False:
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,100,100)
            #await runloop.sleep_ms(1000)
            await move(1300,0,360)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,100,-40)
            await move(1030,0,360)
            await move(-360,0,360)

    async def mission_5_AR_v1(self,drop_expert=True):

        if drop_expert:
            #continueing from mission4_master_piece_drop
            await self.move_backward(15)
            #drop step 1 the experts
            await self.front_arm_down(220)
            #drop step 2 the experts
            await runloop.sleep_ms(1000)
            await self.front_arm_up(force=True)
            #continueing from mission4_master_piece_drop
            await self.move_backward(10)
        else:
            #continueing from mission4_master_piece_drop
            await self.move_backward(25)

        #It turns right to go towards AR and turn the lever.
        await self.turn_right(30)
        
        await self.move_forward(34)
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

    async def mission_4_master_piece(self):
        #it going to first move forward
        await self.move_forward(13,660)
        await self.turn_right(85)
        await self.move_forward(61,660)
        await self.turn_left(57)
        await self.move_forward(68,660)
        #await self.move_backward(25) moving to AR mission for dropping the expert
        if False:
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,100,100)
            #await runloop.sleep_ms(1000)
            await move(1300,0,360)
            await motor_pair.move_for_degrees(motor_pair.PAIR_1,100,-40)
            await move(1030,0,360)
            await move(-360,0,360)

    async def mission_5_AR(self,drop_expert=True):

        if drop_expert:
            #continueing from mission4_master_piece_drop
            await self.move_backward(15)
            #drop step 1 the experts
            await self.front_arm_down(220)
            #drop step 2 the experts
            await runloop.sleep_ms(1000)
            await self.front_arm_up(force=True)
            #continueing from mission4_master_piece_drop
            await self.move_backward(10)
        else:
            #continueing from mission4_master_piece_drop
            await self.move_backward(25)

        #It turns right to go towards AR and turn the lever.
        await self.turn_right(30)

        await self.move_forward(33,400)
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
            await self.move_forward(50,660)
            #next mission music concert
            await self.turn_right(42)
            await self.move_forward(60,660)



    async def mission_7_music_concert(self):
        #pass
        await self.move_forward(70,600)
        await self.turn_right(45)
        await self.move_forward(10)
        await self.move_backward(10)
        await self.turn_left(60)
        await self.move_backward(70,600)


    async def mission_7_drop_audience_and_music_concert(self):
        #pass
        if True:
            await self.move_forward(40,600,steer=-3)
            await self.move_forward(44,600,steer=2)
            await self.move_backward(30)
            await self.turn_right(25)
        
        await self.move_forward(22)
        await self.move_backward(22)
        await self.turn_left(45)
        await self.move_backward(55,600)
        return

    async def mission_7_drop_audience_and_music_concert_13_push_printer(self):
        #pass
        #drop audience in music destination
        await self.move_forward(40,600,steer=-3)
        await self.move_forward(44,600,steer=2)
        await self.move_backward(30)
        await self.turn_right(28)

        #push the music stage drawer
        await self.move_forward(22,600)
        await self.move_backward(16,600)
        await self.turn_left(45)
        await self.move_backward(20,600)

        #push the printer tray
        await self.turn_left(45)
        await self.move_forward(18,600)
        await self.move_backward(20,600)
        #come home
        await self.turn_right(60)
        await self.move_backward(40,600)
        return


    async def mission_13_printer(self):
        await self.move_forward(40) #without attachment 36
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


    async def mission_9_boat_and_flip_camera_handle(self,camera_ring_target=False):
        await self.front_arm_up()
        await self.move_forward(50,500)
        await self.front_arm_down(160)            

        if camera_ring_target:
            await self.move_backward(16)
            await self.turn_left(28)
            await self.move_forward(12)
            await self.front_arm_up()
            await self.move_backward(55,660)
        else:
            await self.move_backward(20,660)
            await self.front_arm_up()
            await self.move_backward(39,660)



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

    race1=False
    race2=True
    race3=False
    race4=False
    race5=False
    race6=False    
    #race 1
    if race1:
        await masterpiece_missions.mission_9_boat_and_flip_camera_handle()
    #race 1 ends

    #race 2
    if race2:
        await masterpiece_missions.mission_1_and_2_and_3()
    #race 2 ends

    #race 3
    if race3:
        await masterpiece_missions.mission_4_master_piece()
        await masterpiece_missions.mission_5_AR()
    #race 3 ends

    #race4
    if race4:
        #await masterpiece_missions.mission_7_music_concert()
        #await masterpiece_missions.mission_7_drop_audience_and_music_concert()
        await masterpiece_missions.mission_7_drop_audience_and_music_concert_13_push_printer()
    
    #race 5
    if race5:
        await masterpiece_missions.mission_13_printer()

    #race6
    if race6:
        await masterpiece_missions.mission_8_camera()

    #races done

    #await masterpiece_missions.back_arm_up()
    #await masterpiece_missions.back_arm_down()
    #await masterpiece_missions.mission_1_3d_cinema()
    #await masterpiece_missions.mission_2_theatre()
    #await masterpiece_missions.mission_3_immersive_experience()
    

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








