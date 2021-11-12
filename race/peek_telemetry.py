import rospy
import random
import sys
from std_msgs.msg import String

instanes_num = 6
data = {}

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

rospy.init_node('watcher' + str(random.randint(0,1000000)))
if len(sys.argv) > 1:
    instances_num = int(sys.argv[1])
subscribe_on_mavros_topics('telemetry', String)
while True:
    inp = int(input('Enter copter number '))
    target = data[inp].get('telemetry')
    print(target)