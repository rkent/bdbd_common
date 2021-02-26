try:
    from Queue import Queue
except:
    from queue import Queue
import rospy

def messageSingle(topic, type):
    responseQueue = Queue()
    sub = rospy.Subscriber(topic, type, lambda msg:responseQueue.put(msg))
    result = responseQueue.get()
    sub.unregister()
    return result

if __name__ == '__main__':
    from sensor_msgs.msg import CameraInfo
    rospy.init_node('test')
    while not rospy.is_shutdown():
        print(messageSingle('/bdbd/pantilt_camera/camera_info', CameraInfo))
        break
