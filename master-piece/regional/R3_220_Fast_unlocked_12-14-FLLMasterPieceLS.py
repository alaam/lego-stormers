from hub import light_matrix, motion_sensor
import runloop, sys
import motor_pair
import motor
import color_sensor
import math
import time
from hub import port

def myprint(*args, **kwargs):
    #pass
    print(args,kwargs)

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


def is_stable () -> bool:
    #myprint("accy", motion_sensor.acceleration(True)[0],"accy:", motion_sensor.acceleration(True)[1],"accz:", motion_sensor.acceleration(True)[2])
    return (motion_sensor.acceleration(True)[0] <100)
    #+ motion_sensor.acceleration(True)[1])==0

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
        ret=self.reset_yaw(0)


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


            #myprint("steering:",steering)
            #myprint("time_elapsed:",time.ticks_ms()-start_time," given:",time_to_follow_in_ms)
            motor_pair.move(motor_pair.PAIR_1,math.floor(pid_output),velocity=100)
            #motor.run(self.right_wheel_port,math.floor(base_speed-pid_output),acceleration=100)
            #motor.run(self.left_wheel_port,math.floor(base_speed+pid_output),acceleration=100)
        motor_pair.stop(motor_pair.PAIR_1)
        motor.stop(self.left_wheel_port)
        motor.stop(self.right_wheel_port)
        myprint("follow line done,", time.ticks_ms()-start_time)




    # input must be in the same unit as WHEEL_CIRCUMFERENCE
    def deg_for_dist(self,distance_cm):
        # Add multiplier for gear ratio if needed
        return int((distance_cm/FLLBaseLib.WHEEL_CIRCUMFERENCE) * 360)


    async def move(self,distance_in_cm,velo=360,steer=0,correct_error=False):
        distance_in_deg=self.deg_for_dist(distance_in_cm)
        sa=0
        if correct_error:
            await self.reset_yaw(0)
            sa=max(.1,self.get_360_mapped_yaw())

        await motor_pair.move_for_degrees(motor_pair.PAIR_1,distance_in_deg,steer,velocity=velo,stop=motor.BRAKE, acceleration=1000)

        if correct_error:
            ea=self.get_360_mapped_yaw(stablize_first=True)
            ed=self.compute_error(sa,ea,sa,requested_deg_per_achived=0,hw_err_threshold=150)
            ed = ed * (-1 if distance_in_cm<0 else 1)
            await self.turn_using_gyro(ed,speed=50,correct_error=False)


    async def reset_yaw(self,offset=0):
        #check_stable = lambda : (( motion_sensor.acceleration(True)[0] + motion_sensor.acceleration(True)[1] ) ==0 )
        #await runloop.until(is_stable)
        await runloop.sleep_ms(65)
        motion_sensor.reset_yaw(offset)
        await runloop.sleep_ms(65)
        #await runloop.until(motion_sensor.acceleration)

    def get_360_mapped_yaw(self,stablize_first=False):
        if stablize_first:
            time.sleep(0.065)
        cy=motion_sensor.tilt_angles()[0]
        #if (cy<0): return cy+3600
        return cy/10

    async def turn_using_gyro(self,deg,speed=200,correct_error=False):
        self.cur_turn_deg=deg
        await self.reset_yaw(0)
        turn = -100 if deg<0 else 100

        start_yaw_angle=self.get_360_mapped_yaw(stablize_first=True) #motion_sensor.tilt_angles()[0]
        curr=self.get_360_mapped_yaw() #motion_sensor.tilt_angles()[0]
        diff = abs(curr)-abs(start_yaw_angle)
        myprint("from yaw:",curr, " start_yaw_angle:",start_yaw_angle," diff:",diff, "asked:",deg*10)
        if True:
            motor_pair.move(motor_pair.PAIR_1,turn,velocity=speed)
        else:
            if deg < 0: #left turn
                motor.run(self.left_wheel_port,speed)
            else:
                motor.run(self.right_wheel_port,-speed)

        count=1
        requested_deg_per_achived= 1 if correct_error else 1
        while abs(diff) < abs(deg)*requested_deg_per_achived: #yaw is decidegree hence multiply incoming angle by 10
            #await runloop.sleep_ms(1)
            #myprint("from yaw:",abs(curr), " start_yaw_angle:",abs(start_yaw_angle)," diff:",abs(diff), "asked:",abs(deg*10))
            curr=self.get_360_mapped_yaw() #motion_sensor.tilt_angles()[0]
            diff = abs(curr)-abs(start_yaw_angle)
            count+=1
        motor_pair.stop(motor_pair.PAIR_1,stop=motor.BRAKE)
        #myprint("after loop yaw:",motion_sensor.tilt_angles()[0])
        #await runloop.sleep_ms(200)
        ea=self.get_360_mapped_yaw(stablize_first=True) #motion_sensor.tilt_angles()[0]
        myprint("after 200ms yaw ea:",ea," loop count:",count)
        sa=start_yaw_angle
        if correct_error:
            ed=self.compute_error(sa,ea,deg,requested_deg_per_achived)
            myprint("gyro: degrees:",deg,"ea:",ea," sa:",sa, "error correction:",ed)
            await self.turn_using_gyro(ed,speed=100,correct_error=False)
        myprint("end yaw:",ea)


    async def turn(self,deg):
        motion_sensor.set_yaw_face(motion_sensor.FRONT)
        await self.reset_yaw(0)
        myprint("start yaw:",self.get_360_mapped_yaw())
        if deg<0:
            await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,deg,0,100,acceleration=100)
        else:
            await motor_pair.move_tank_for_degrees(motor_pair.PAIR_1,deg,100,0,acceleration=100)
        myprint("end yaw:",self.get_360_mapped_yaw(stablize_first=True))


    def compute_error(self,gyro_starta,gyro_enda,requested_deg,requested_deg_per_achived,hw_err_threshold=0.3):
        ea=gyro_enda
        sa=gyro_starta
        degrees=requested_deg
        ed=abs(abs(ea)-abs(sa))-abs(degrees)
        if (abs(ed)>abs(degrees)*hw_err_threshold): #only correct 30% of error ; assuming beyond 30% is malfunction of hw but visibly its not
            myprint("compute error: HW mal function: degrees:",degrees,"ea:",ea," sa:",sa, "error correction:",ed)
            if (ed<0):
                ed=-abs(degrees)*(1-requested_deg_per_achived)
            else:
                ed=abs(degrees)*(1-requested_deg_per_achived)
            #ed=0 #not doing error correction if error is more than 30%

        if ed<0: #under run
            ed = ed * (-1 if degrees < 0 else 1)
        else: #over run
            ed = ed * (1 if degrees < 0 else -1) #reverse the direct
        myprint("compute error: degrees:",degrees,"ea:",ea," sa:",sa, "error correction:",ed)
        return ed

    async def spin_turn(self,degrees, mspeed=200,correct_error=False):
        #motion_sensor.set_yaw_face(motion_sensor.FRONT)
        await self.reset_yaw(0)

        sa=self.get_360_mapped_yaw() #(motion_sensor.tilt_angles()[0])/10

        myprint("start yaw:",sa," deg requsted:",degrees)


        SPIN_CIRCUMFERENCE = FLLBaseLib.TRACK * math.pi
        ed=0
#        while ed or degrees:
        # Add a multiplier for gear ratios if you’re using gears
        requested_deg_per_achived= 1 if correct_error else 1
        mot_degrees = int((SPIN_CIRCUMFERENCE/FLLBaseLib.WHEEL_CIRCUMFERENCE) * abs(degrees)*requested_deg_per_achived)
        if degrees > 0:
            # spin clockwise
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, mot_degrees, 100, velocity=mspeed)
        else:
            #spin counter clockwise
            await motor_pair.move_for_degrees(motor_pair.PAIR_1, mot_degrees, -100, velocity=mspeed)
        #await runloop.sleep_ms(150)
        ea=self.get_360_mapped_yaw(stablize_first=True) #(motion_sensor.tilt_angles()[0])/10

        if correct_error:
            ed=self.compute_error(sa,ea,degrees,requested_deg_per_achived)
            myprint("spin: degrees:",degrees,"ea:",ea," sa:",sa, "error correction:",ed)
            #await self.spin_turn(ed,correct_error=False,mspeed=100)
            await self.turn_using_gyro(ed,speed=50,correct_error=False)


        if False and correct_error:
            ed=abs(degrees)-abs(abs(ea)-abs(sa))
            #ed = ed * -1 if degrees < 0 else 1
            myprint("degrees:",degrees,"ea:",ea," sa:",sa, "error correction:",ed)
            await self.spin_turn(ed,correct_error=False,mspeed=100)
        myprint("end yaw:",ea)


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


    async def turn_right(self,deg,speed=200,use_gyro=False,correct_error=False): #200
        if use_gyro==False:
            await self.spin_turn(deg,speed,correct_error=correct_error)
            #await self.pivot_turn(deg,speed)
        else:
            await self.turn_using_gyro(deg,speed,correct_error=correct_error)


    async def turn_left(self,deg,speed=200,use_gyro=False,correct_error=False): #200
        if use_gyro==False:
            await self.spin_turn(-deg,speed,correct_error=correct_error)
            #await self.pivot_turn(-deg,speed)
        else:
            await self.turn_using_gyro(-deg,speed,correct_error=correct_error)



    async def back_arm_up(self,degree=130):
        await motor.run_to_absolute_position(self.second_arm_port,degree,200,direction=motor.CLOCKWISE)
        myprint("back_arm_up done")


    async def back_arm_down(self,degree=200):
        await motor.run_to_absolute_position(self.second_arm_port,degree,200,direction=motor.COUNTERCLOCKWISE)
        myprint("back_arm_down done")

    async def front_arm_up(self,degree=10,speed=200,force=False):
        #degree = (degree+self.first_arm_init_deg)%360
        cur_pos=motor.absolute_position(self.second_arm_port)
        print("front_arm_up: cur_pos:",cur_pos)
        #hack needed sometime motor does overdrive
        if not force and (cur_pos> 350 or cur_pos < 25): return
        await motor.run_to_absolute_position(self.second_arm_port,degree,speed,direction=motor.CLOCKWISE)
        myprint("back_arm_up done")


    async def front_arm_down(self,degree=180,speed=200,acce=1000):
        #degree = (degree+self.first_arm_init_deg)%360
        await motor.run_to_absolute_position(self.second_arm_port,degree,speed,direction=motor.COUNTERCLOCKWISE,acceleration=acce)
        myprint("back_arm_down done")


    async def front_arm_go_relative(self,delta_degree,speed=200):
        await motor.run_to_relative_position(self.second_arm_port,delta_degree,speed)
        myprint("front_arm_go_relative done")

class FLL2023MasterPieceMissions(FLLBaseLib):
    def __init__(self, cfg):
        super().__init__(cfg)

    async def mission_1_3d_cinema(self):
        await self.move_forward(34,velo=660,steer=-5) #30
        await self.front_arm_down(293,speed=600) #253,260 touching floor; #293 mid-point point; #330 vertical
        #await self.front_arm_down() #110 too low but works
        #await self.front_arm_up()
        await self.turn_right(60)
        await self.turn_left(40)
        await self.front_arm_up(speed=660,force=True)
        await self.move_backward(30,velo=660)


    async def mission_1_3d_cinema_regional(self):
        await self.front_arm_up()
        await self.move_forward(28,steer=-3) #30
        await self.front_arm_down() #110 too low but works
        await self.front_arm_up()
        await self.move_backward(30)

    async def mission_2_theatre_wec(self):
        speed=660
        if False:
            await self.move_forward(67,600)
        else:
            await self.front_arm_up()
            await self.move_forward(15,500)
            await self.turn_right(90,correct_error=True)
            await self.move_forward(22,500)
            await self.turn_left(90,correct_error=True)
            await self.move_forward(45,600)
            #await self.turn_left(35,correct_error=False)
            await self.turn_left(45,correct_error=True)
            await self.move_forward(5,500)
        num_try=2
        for i in range(0,num_try):
            #await self.front_arm_down(170,speed)
            #await self.front_arm_down(90,speed=350,acce=5000)
            await runloop.sleep_ms(500)
            await self.move_backward(5)
            #await self.front_arm_up()
            if i < num_try-1:
                await self.move_forward(5)
                pass

        #await self.move_backward(7,velo=660)
        await self.turn_right(50,correct_error=False)
        await self.move_backward(60,velo=660)
        #next mission to immersive exp



    async def mission_2_theatre_nec(self):
        speed=660
        if False:
            await self.move_forward(67,600)
        else:
            await self.front_arm_up()
            await self.move_forward(15)
            await self.turn_right(90, correct_error=False)
            await self.move_forward(25)
            await self.turn_left(90, correct_error=False)
            await self.move_forward(42,600)
            await self.turn_left(32, correct_error=False)
            await self.move_forward(13)
        num_try=2
        for i in range(0,num_try):
            #await self.front_arm_down(170,speed)
            #await self.front_arm_down(90,speed=350,acce=5000)
            await runloop.sleep_ms(500)
            await self.move_backward(5)
            #await self.front_arm_up()
            if i < num_try-1:
                await self.move_forward(5)
                pass

        #await self.move_backward(7,velo=660)
        await self.turn_right(50, correct_error=True)
        await self.move_backward(60,velo=660)
        #next mission to immersive exp



    async def mission_2_theatre_regional(self):
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
        myprint("start mission_1_and_2:",st)
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
        myprint("end mission_1_and_2: secs:",time.time()-st)

    async def mission_1_and_2_and_3_wec(self):
        st=time.time()
        myprint("start mission_1_and_2_and_3:",st)
        speed=660
        #{ await self.mission_1_3d_cinema()
        sa=self.get_360_mapped_yaw(stablize_first=True)
        await self.move_forward(34,velo=660,steer=-5) #30
        await self.front_arm_down(310,speed=600)
        #await self.front_arm_down() #110 too low but works
        #await self.front_arm_up()
        await self.turn_right(40)
        await self.turn_left(40)
        await self.front_arm_up(speed=660,force=True)
        await self.move_backward(25,660)
        if False:
            ea=self.get_360_mapped_yaw(stablize_first=True)
            ed=self.compute_error(sa,ea,0.1,1,400)
            await self.turn_right(abs(ed),correct_error=False)
        #}
        
        #{ await self.mission_2_theatre()
        #}
        #{mission_3_immersive_experience
        await self.front_arm_up()
        #await self.front_arm_go_relative(250)
        #return
        #await self.move_forward(25,600)
        await self.turn_right(117,correct_error=False)
        
        await self.move_forward(68,600)
        await self.turn_left(85)
        await self.move_forward(46,velo=600)
        return
        if True:
            await self.front_arm_down() #90
            #await self.front_arm_go_relative(-75)
            await runloop.sleep_ms(1000)
            await self.front_arm_up(force=True)
            await self.move_backward(45,600)
            await self.turn_right(65)
            await self.move_backward(80,600)

        #}
        myprint("end mission_1_and_2_and_3: secs:",time.time()-st)


    async def mission_1_and_2_and_3(self):
        st=time.time()
        myprint("start mission_1_and_2_and_3:",st)
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
        myprint("end mission_1_and_2_and_3: secs:",time.time()-st)


    async def mission_3_immersive_experience(self):
        await self.front_arm_up()
        #await self.front_arm_go_relative(250)
        #return
        await self.move_forward(25,600)
        await self.turn_right(90,correct_error=False)
        await self.move_forward(68,600)
        await self.turn_left(85)
        await self.move_forward(46,velo=600)

        if True:
            await self.front_arm_down() #90
            #await self.front_arm_go_relative(-75)
            await runloop.sleep_ms(1000)
            await self.front_arm_up(force=True)
            await self.move_backward(45,600)
            await self.turn_right(65)
            await self.move_backward(80,600)



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
        await self.front_arm_up(90,force=True) #going more vertical to avoid hitting AR blue leafs
        #it going to first move forward
        await self.move_forward(13,660)
        await self.turn_right(85,speed=100)
        await self.move_forward(61,660)
        await self.turn_left(63,speed=100)
        await self.move_forward(71,660)
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
            await self.front_arm_up(330,force=True) #253,260 touching floor; #293 mid-point point; #330 vertical
            
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
        await self.move_forward(29,600)
        await self.move_backward(20,600)
        #come home
        await self.turn_right(60)
        await self.move_backward(40,600)
        return


    async def mission_13_printer(self):
        await self.move_forward(40) #without attachment 36
        await self.move_backward(45)

    async def mission_8_camera(self):
        await self.move_forward(35,velo=600,steer=-2)
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
        #await self.turn_right(17,speed=50)
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

    async def go_between_launch_zones(self):
        await self.move_forward(200,velo=700)



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
    race2=False
    race3=False
    race4=False
    race5=False
    race6=False
    race7=False
    race9=True
    #race 1
    if race1:
        #await masterpiece_missions.mission_9_boat_and_flip_camera_handle()
        await masterpiece_missions.mission_2_theatre_nec()
    #race 1 ends

    #race 2
    if race2:
        await masterpiece_missions.mission_9_boat_and_flip_camera_handle()
        #await masterpiece_missions.mission_1_and_2_and_3_wec()
        #await masterpiece_missions.mission_1_3d_cinema()

    #race 2 ends

    #race 3
    if race3:
        await masterpiece_missions.mission_1_3d_cinema()
        #await masterpiece_missions.mission_4_master_piece()
        #await masterpiece_missions.mission_5_AR()
        #await masterpiece_missions.mission_2_theatre_wec()
        #await masterpiece_missions.mission_2_theatre_nec()
    #race 3 ends

    #race4
    if race4:
        #await masterpiece_missions.mission_7_music_concert()
        #await masterpiece_missions.mission_7_drop_audience_and_music_concert()
        #await masterpiece_missions.mission_7_drop_audience_and_music_concert_13_push_printer()
        await masterpiece_missions.mission_3_immersive_experience()

    #race 5
    if race5:
        await masterpiece_missions.mission_4_master_piece()
        await masterpiece_missions.mission_5_AR(drop_expert=False)
        #await masterpiece_missions.mission_13_printer()

    #race6
    if race6:
        await masterpiece_missions.mission_7_drop_audience_and_music_concert_13_push_printer()

    if race7:
        await masterpiece_missions.mission_8_camera()

    if race9:
        await masterpiece_missions.go_between_launch_zones()
    #races done

    #await masterpiece_missions.back_arm_up()
    #await masterpiece_missions.back_arm_down()
    #await masterpiece_missions.mission_1_3d_cinema()
    #await masterpiece_missions.mission_2_theatre()
    #await masterpiece_missions.mission_3_immersive_experience()


    if False:
        myprint("calling move reverse")
        await masterpiece_missions.move_backward(60,velo=1110)
        myprint("calling move fwd")
        await masterpiece_missions.move_forward(60,velo=1110)
        #await runloop.sleep_ms(100)
        myprint("calling move turn left")
        #await masterpiece_missions.turn_left(90,use_gyro=False,correct_error=True,speed=600)
        await runloop.sleep_ms(500)
        myprint("calling move turn right")
        #await masterpiece_missions.turn_right(90,use_gyro=False,correct_error=True,speed=600)
        #await masterpiece_missions.turn_right(90,use_gyro=True)
        myprint("calling move end of main")
    #sys.exit(0)


#main()
runloop.run(main())








