# Game Theory Planning
# Husnul Amri (2021)
# github : @moezeus
# adapted from RobustDecisionMaking (Gokul)

import numpy as np
import time
import math
import serial
from ev3dev.ev3 import *
# from thesis.msg import Autonomous_Game
import rospy
# from thesis.msg import Game_Theory_Logger
# from thesis.msg import State_Estimator
# import rospy
# import os

#for used in master PC (with ROS)
# using MQTT
import paho.mqtt.client as mqtt
import numpy as np

#Serial communication Arduino Mega 2560 - EV3
arduino = serial.Serial(port='/dev/tty_ev3-ports:in1', baudrate=9600, timeout=0.01)
ev3_port = LegoPort('in1')
ev3_port.mode = 'other-uart'

# MQTT Stuff
broker ="192.168.1.101"
port = 1883
topic = "/EV3_movement/#"

# ROS thing
rospy.init_node('controller')
freq = 20 # Hz
# pub = rospy.Publisher('/game_theory_AV', Autonomous_Game, queue_size=1)
pub = rospy.Publisher('/jackknife_control', Game_Theory_Logger, queue_size=1)
rate = rospy.Rate(freq) # Hz
# pub_msg = Autonomous_Game()
pub_msg = Game_Theory_Logger()
pub_msg.header.frame_id = 'agv_trajectory_control_' + "game_theory"
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()

dt_sim = 1/freq

#Parameter Lyapunov Anti-Jackknife Control
xoffset = -36           #imu xoffset trailler in Mag
yoffset = 15.5          #imu yoffset trailler in Mag

lh = 0.145              #length of head truck (m)
lt = 0.313              #length of trailer (m) 
rb = 0.6                #Turning Radius (m)
vh_max = 0.2*1100/63    #Maksimum value of head velocity (m/s)
vh1 = vh_max*0.2        #Head velocity (m/s)
Qh = 0                  #the angle of head truck to x axis (degree)
Qt = 0                  #the angle of trailer to x axis in (degree)
Qs = 0                  #steering angle (degree)
Qd = 0                  #desired angle (degree)
dQs = 0                 #Lyapunov steering angle (degree)
Qh_e = 0                #error of head truck angle (to desired angle)
Qt_e = 0                #error of trailer angle (to desired angle)
Qht_e = 0               #error of angle between head and trailer 
h_eh = 0                #error distance of head (to desired path)
h_et = 0                #error distance of trailer (to desired path)

kh_eh = 1               #Coefficient of head distance error
kQh_e = 1               #Coefficient of head angle error
kh_et= 1                #Coefficient of trailer distance error
kQt_e = 1               #Coefficient of trailer angle error

print("Waiting.....")
time.sleep(5) #wait for live plot node (start it manually)
print("Simulation Start!")


# function definition
def connect_mqtt() -> mqtt:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def sqrt(n):
    assert n > 0
    with decimal.localcontext() as ctx:
        ctx.prec += 2 # increase precision to minimize round off error
        x, prior = decimal.Decimal(n), None
        while x != prior: 
            prior = x
            x = (x + n/x) / 2 # quadratic convergence 
    return +x # round in a global context

def signum(x):              #Define Sign Function
    if x > 0:
        return 1
    elif x < 0:
        return -1
    elif x == 0:
        return 0
    else:
        return x

def sinc(x):                #Define Sinc Function
    if x == 0:
        return 1
    if x != 0:
        return math.sin((math.pi*(x)/180)/(math.pi*(x)/180))

# Anti-Jackknife based lyapunov
def anti_jackknife_control(xt, yt, xh, yh, yaw_t, yaw_h, x_wp, y_wp, v_wp):
    # check waypoint to follow based on time trajectory   
    close_idx = np.argmin(np.sum(np.square((xt - x_wp), (yt - y_wp))))
    if((x_wp[close_idx] < xpos[0]) and (y_wp[close_idx] > ypos[3])):
        h_et = yt - y_wp
        h_eh = yh - y_wp
        Qd = 0
    elif(((x_wp[close_idx] > xpos[1]) and (y_wp[close_idx] > ypos[3])) or (x_wp[close_idx] < xpos[3])):
        h_et = xt - x_wp
        h_eh = xh - x_wp    
        Qd = 90
    
    Qh_e = abs((Qh - Qd) * math.pi / 180)
    Qt_e = abs((Qt - Qd) * math.pi / 180)
        
    vh1 = vh_max / (1 + 0.5*kQt_e * abs(Qt_e) + 0.5*kQh_e * abs(Qht_e) + kh_eh * abs(h_et))
    V_AV = vh1*1000*63/(1100*2)
    
    # Steering control based lyaponov
    if xt == 0 : 
        cs_steer = 0
    else :       
        #Steering Angle Control Function
        Qht_e = Qh_e - Qt_e
        if abs(Qht_e) >= math.pi/5 :
            Qht_e = signum(Qht_e) * math.pi/5
        else :
            Qht_e = Qht_e
        
        psi1 = -math.atan(lt/(vh1)*(0.01*kQt_e*Qt_e/math.cos(Qht_e)+kQt_e*h_et*vh1*sinc(Qt_e/math.pi)))
        z = psi1 - Qht_e
        cs_steer = -math.atan(lh/(vh1)*(-0.01*kQh_e*z+vh1/lt*math.tan(psi1)*math.cos(Qht_e)))

        if abs(cs_steer) >= 19/180 * math.pi :
            cs_steer = signum(cs_steer) * 19 * math.pi / 180
        else :
            cs_steer = cs_steer
        #cs_steer = (dQs * 180 / math.pi)
    
    return V_AV, -cs_steer

# update AV actual state
# subscribe dari vehicle_state
def main_function(msg):

    global xt_AV, yt_AV, xh_AV, yh_AV, yaw_t, yaw_h, V_AV

    xt_AV = msg.xt_est
    yt_AV = msg.yt_est
    xh_AV = msg.xh_est
    yh_AV = msg.yh_est
    yaw_t = msg.yaw_t
    yaw_h = msg.yaw_h
    V_AV = msg.v_est

# calculate and update actual steer angle
# subscribe dari ESP
def steer_angle_callback(client, userdata, message):
    global cs_steer_actual

    cs_steer_actual = int(message.payload.decode("utf-8"))
    cs_steer_actual = 0.002306 * cs_steer_actual - 1.0898

# MQTT stuff
client = connect_mqtt()
client.subscribe(topic)
client.message_callback_add("/EV3_movement/steer_angle_ADC", steer_angle_callback)
client.loop_start()

# subscribe node vehicle state
sub = rospy.Subscriber('/vehicle_state', State_Estimator, main_function)
# rospy.spin()

# loop utama
while True:
    #======================Controller Part===================================
    if kondisi == '1' :     #X Axis Movement (from x min to max)
        if (xt_AV > xpos[1]):
            h_et = yt_AV - y_wp
            h_eh = yh_AV - y_wp
            Qd = 0
            cs_long, cs_steer = calculate_stanley_control(X_AV, Y_AV, yaw_t, yaw_h, time_now, sig_x, sig_y, dt, V_ref_tr, yaw_AV)
        else:
            kondisi = '2'
            Qd = 90
    elif kondisi == '2' :     #Y Axis Movement (from y min to y max)
        if yt_AV > ypos[4] :
            h_et = xt_AV - x_wp
            h_eh = xh_AV - x_wp
            Qd = 180
            #m.run_to_rel_pos(position_sp = int(Qd), speed_sp = 500)
        else :
            #kondisi = '3'
            Qd = 180
    
    # send command to ev3 via master_auto_control node 
    client.publish("/EV3_movement/steer_command_rad",cs_steer,qos=0)
    client.publish("/EV3_movement/speed_command",cs_long,qos=1)
    # print(dt)  

    # Store calculated value to pub_msg
    # disesuaikan saja
    # AV Actual   
    pub_msg.actual_x_av = X_AV
    pub_msg.actual_y_av = Y_AV
    pub_msg.actual_yaw_av = yaw_pub
    pub_msg.actual_speed_av = V_AV
    pub_msg.actual_steer_av = cs_steer_actual
    pub_msg.actual_action_av = Action_Game
    pub_msg.path_x_av = sig_x
    pub_msg.path_y_av = sig_y
    # AV Target
    pub_msg.target_x_av = sig_x[iterate_waypoints]
    pub_msg.target_y_av = sig_y[iterate_waypoints]
    pub_msg.target_yaw_av = yaw_AV[iterate_waypoints]
    pub_msg.target_speed_av = V_ref_tr[iterate_waypoints]
    pub_msg.target_steer_av = cs_steer
    # Header
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.header.seq += 1
    pub.publish(pub_msg)

    rate.sleep()
