# node requests (topic, service, action) with automatic node startup

import rospy
import rostopic
import traceback
import rosservice
from bdbd_common.srv import NodeCommand

TOPIC_SERVICE = '/bdnodes/topic'
SERVICE_SERVICE = '/bdnodes/service'

# data structure for status of topic nodes
class ActiveTopic():
    def __init__(self, type):
        # I don't currently use lastUse
        self.lastUse = rospy.get_time()
        self.type = type

active_doers = {}

class DoerRequest():
    def __init__(self):
        pass

    def ServiceProxy(self, name, service_class, persistent=False, headers=None, timeout=5.0):
        name = rospy.resolve_name(name)
        self.ensure_doer(name, 'service', timeout)
        return rospy.ServiceProxy(name, service_class, persistent, headers)

    def Subscriber(self, name, *args, **kwargs):
        timeout = 5.0 if 'timeout' not in kwargs else kwargs['timeout']
        name = rospy.resolve_name(name)
        self.ensure_doer(name, 'topic', timeout=timeout)
        return rospy.Subscriber(name, *args, **kwargs)

    def cancel_doer(self, name, timeout=5.0):
        name = rospy.resolve_name(name)
        if not name in active_doers:
            rospy.loginfo('topic {} not active in this node'.format(name))
            return;
        bdnodes_result = 'failed'
        type = active_doers[name].type
        try:
            bdnodes_svc = self.get_starter(type, timeout)
            rospy.loginfo('sending service request to stop node for topic {}'.format(name))
            bdnodes_result = bdnodes_svc(name, 'stop')
        except rospy.ROSException:
            rospy.logwarn('Could not find bdnodes topic service')
        except rospy.ServiceException:
            rospy.logwarn('Error from bdnodes while trying to stop doer for name {}'.format(name))
        rospy.loginfo('stop result: {}'.format(bdnodes_result))
        del active_doers[name]
        return

    def get_starter(self, type, timeout=5.0):
        # throws rospy.ROSException upon timeout if fails
        rospy.loginfo('waiting for bdnodes service')
        if type == 'topic':
            service_name = TOPIC_SERVICE
        else:
            service_name = SERVICE_SERVICE
        rospy.wait_for_service(service_name, timeout=timeout)
        return rospy.ServiceProxy(service_name, NodeCommand)

    def ensure_doer(self, name, type, timeout=5.0):
        name = rospy.resolve_name(name)
        isActive = False
        haveStarter = rosservice.get_service_list().count('/bdnodes/service') > 0
        if type == 'service':
            isActive = rosservice.get_service_list().count(name) > 0
        else:
            (pubs, _) = rostopic.get_topic_list()
            for (a_topic, _, _) in pubs:
                if a_topic == name:
                    isActive = True

        if name not in active_doers and haveStarter:
            # try to call started even if active, so it has the node reference
            rospy.loginfo('trying to start doer for name {}'.format(name))
            bdnodes_result = 'failed'
            try:
                bdnodes_svc = self.get_starter(type, timeout)
                rospy.loginfo('sending service request to start doer for name {}'.format(name))
                bdnodes_result = bdnodes_svc(name, 'start').response
            except rospy.ROSException:
                rospy.logwarn('Could not find bdnodes service')
            except rospy.ServiceException:
                rospy.logwarn('Error from bdnodes while trying to start doer for {}'.format(name))
            if bdnodes_result in ['started', 'active']:
                isActive = True
                try:
                    if type == 'service':
                        rospy.wait_for_service(name, timeout=timeout)
                except:
                    rospy.logwarn('Probable timeout waiting for service {}'.format(name))
                    isActive = False
                rospy.loginfo('doer startup status for name {}: {}'.format(name, isActive))
        if isActive:
            active_doers[name] = ActiveTopic(type)
        else:
            if not haveStarter:
                rospy.logwarn('bdnodes not available while trying to start doer for {}'.format(name))
            raise Exception('Failed to ensure doer for {}'.format(name))

    def wait_for_message(self, topic, topic_type, timeout=5.0):
        topic = rospy.resolve_name(topic)
        # like rospy.wait_for_message but starts doer node if needed
        message = None
        active_topic = None
        self.ensure_doer(topic, 'topic', timeout)
        try:
            message = rospy.wait_for_message(topic, topic_type, timeout)
            if active_topic:
                active_topic.lastUse = rospy.get_rostime()
            else:
                active_doers[topic] = ActiveTopic('topic')
        except rospy.ROSException:
            raise Exception('Could not get message for topic {}'.format(topic))
        return message

if __name__ == '__main__':
    try:
        from bdbd_common.srv import SpeechCommand
        #from sensor_msgs.msg import CompressedImage
        TOPIC = '/bdbd/pantilt_camera/image_raw/compressed'
        SERVICE = '/bdbd/chat'
        rospy.init_node('doerRequest')
        nr = DoerRequest()
        nr.ensure_doer('/bdbd/dialog', 'service', timeout=60.0)
        '''
        result = nr.wait_for_message(TOPIC, CompressedImage)
        print(result.header)
        result = nr.wait_for_message(TOPIC, CompressedImage)
        print(result.header)
        nr.cancel_topic(TOPIC)
        '''
        chat_srv = nr.ServiceProxy(SERVICE, SpeechCommand)
        for count in range(3):
            sayit = chat_srv('chatservice', 'Are you a robot?').response
            print(sayit)
            rospy.sleep(5.0)
        # nr.cancel_doer(SERVICE)
    except:
        print(traceback.format_exc())

