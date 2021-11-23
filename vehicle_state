import time
import rospy
from marvelmind_nav.msg import hedge_pos_a
from thesis.msg import State_Estimator
import numpy as np

#for used in master PC (with ROS)
# using MQTT
import paho.mqtt.client as mqtt
import numpy as np

# MQTT Stuff
broker ="192.168.1.101"
port = 1883
topic = "/EV3_movement/#"

# declare constant/initial parameter
def get_params():
    class Bunch:
        def __init__(self, **kwds):
            self.__dict__.update(kwds)
    # Declare constant parameters
    params = Bunch(
                xbef = 2.68,
                xnow = 2.68,
                ybef = -0.72,
                ynow = -0.72,
                t_bef = 0,
                t_now = 0,
                yaw = 3.14,
                v_now = 0 )

    return params

def connect_mqtt() -> mqtt:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt.Client()
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

# def speed_callback(client, userdata, message):
#     params.v_now= float(message.payload.decode("utf-8"))
#     print(params.v_now)

def save_speed_value(client, userdata, message): 
    speed = float(message.payload.decode("utf-8"))
    speed = (speed * 2.1)/100
    params.v_now = -speed

def vehicle_state(msg):
    if params.v_now > 0: #vehicle move
        params.xnow = msg.x_m
        params.ynow = msg.y_m
        dist = np.sqrt((params.ynow-params.ybef)**2-(params.xnow-params.xbef)**2)

        if dist>0.1 : #its impossible to do 10cm displacement instantly
            params.xnow = params.xbef
            params.ynow = params.ybef

        if abs(params.xnow - params.xbef) > 0.005:  #only update yaw under this condition
            params.yaw = np.arctan2((params.ynow-params.ybef),(params.xnow-params.xbef))
            params.xbef = params.xnow
            params.ybef = params.ynow

        # params.xbef = params.xnow
        # params.ybef = params.ynow

        # print(params.xnow, params.ynow, params.yaw, params.v_now)
    # else : 
        # print(params.xnow, params.ynow, params.yaw, params.v_now)
    pub_msg.header.seq = pub_msg.header.seq + 1
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.yaw_est = params.yaw
    pub_msg.x_est = params.xnow
    pub_msg.y_est = params.ynow
    pub_msg.v_est = params.v_now
    pub.publish(pub_msg)

params = get_params()
client = connect_mqtt()
client.subscribe(topic)
# client.message_callback_add("/EV3_movement/speed_command", speed_callback)
client.message_callback_add("/EV3_movement/speed_AV", save_speed_value)
client.loop_start()

rospy.init_node('vehicle_state')

freq = 50 # Hz

pub_msg = State_Estimator()
pub_msg.header.frame_id = 'vehicle_state'
pub_msg.header.seq = 0
pub_msg.header.stamp = rospy.Time.now()
sub = rospy.Subscriber('/hedge_pos_a', hedge_pos_a, vehicle_state)
pub = rospy.Publisher('/vehicle_state', State_Estimator, queue_size=1)
rate = rospy.Rate(freq) # Hz

rospy.spin()
# client.loop_forever()
while True : 
    # print("Lets Rock")
    rate.sleep()

