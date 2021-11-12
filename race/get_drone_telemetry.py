import rospy
import sys

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import PositionTarget, State, ExtendedState
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import String

from mavros_msgs.srv import SetMode, CommandBool, CommandVtolTransition, CommandHome

instances_num = 6
data = {}

# Стартовая функция для подписывания на топики
def subscribe_on_topics():
    # глобальная (GPS) система координат
    subscribe_on_mavros_topics("global_position/global", NavSatFix)

    # локальная система координат, точка отсчета = место включения аппарата
    subscribe_on_mavros_topics("local_position/pose", PoseStamped)
    subscribe_on_mavros_topics("local_position/velocity_local", TwistStamped)

    # состояние
    subscribe_on_mavros_topics("state", State)
    subscribe_on_mavros_topics("extended_state", ExtendedState)




# Подписываемся на Mavros топики всех аппаратов
def subscribe_on_mavros_topics(suff, data_class):
    for n in range(1, instances_num + 1):
        data[n] = {'wall_num': 0}
        topic = f"/mavros{n}/{suff}"
        rospy.Subscriber(topic, data_class, topic_cb, callback_args=(n, suff))




# При публикаци нового сообщения дронов
def topic_cb(msg, callback_args):
    n, suff = callback_args
    data[n][suff] = msg

if len(sys.argv) > 1:
    instances_num = int(sys.argv[1])
rospy.init_node('watcher')
subscribe_on_topics()

while True:
    inp = int(input('Enter copter number '))
    telemetry = data[inp].get('local_position/pose')
    print(telemetry)
