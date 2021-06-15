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
        bdnodes_result = self.ensure_doer(name, 'service', timeout)
        if not bdnodes_result in ('started', 'active'):
            services = rosservice.get_service_list()
            if services.count(name) == 0:
                raise Exception('bdnodes missing, and service not started')
        rospy.wait_for_service(name, timeout=timeout)
        return rospy.ServiceProxy(name, service_class, persistent, headers)

    def cancel_doer(self, name, timeout=5.0):
        if not name in active_doers:
            rospy.loginfo('topic {} not active in this node'.format(name))
            return;
        bdnodes_result = 'failed'
        type = active_doers[name].type
        try:
            svc = self.ensure_starter(type, timeout)
            rospy.loginfo('sending service request to stop node for topic {}'.format(name))
            bdnodes_result = svc(name, 'stop')
        except rospy.ROSException:
            rospy.logwarn('Could not find bdnodes topic service')
        except rospy.ServiceException:
            rospy.logwarn('Error from bdnodes while trying to stop doer for name {}'.format(name))
        rospy.loginfo('stop result: {}'.format(bdnodes_result))
        del active_doers[name]
        return

    def ensure_starter(self, type, timeout=5.0):
        # throws rospy.ROSException upon timeout if fails
        rospy.loginfo('waiting for bdnodes service')
        if type == 'topic':
            service_name = TOPIC_SERVICE
        else:
            service_name = SERVICE_SERVICE
        rospy.wait_for_service(service_name, timeout=timeout)
        return rospy.ServiceProxy(service_name, NodeCommand)

    def ensure_doer(self, name, type, timeout=5.0):
        if name not in active_doers:
            # doer is not active
            rospy.loginfo('trying to start doer for name {}'.format(name))
            # try to use bdnodes to start
            bdnodes_result = 'failed'
            try:
                svc = self.ensure_starter(type, timeout)
                rospy.loginfo('sending service request to start doer for name {}'.format(name))
                bdnodes_result = svc(name, 'start').response
            except rospy.ROSException:
                rospy.logwarn('Could not find bdnodes topic service')
            except rospy.ServiceException:
                rospy.logwarn('Error from bdnodes while trying to start doer for {}'.format(name))
            rospy.loginfo('doer startup result: {}'.format(bdnodes_result))
            active_doers[name] = ActiveTopic(type)
            return bdnodes_result

    def wait_for_message(self, topic, topic_type, timeout=5.0):
        # like rospy.wait_for_message but starts doer node if needed
        message = None
        active_topic = None
        bdnodes_result = self.ensure_doer(topic, 'topic', timeout)
        if not bdnodes_result in ('started', 'active'):
            rospy.loginfo('Failed bdnodes, checking if already published')
            (pubs, _) = rostopic.get_topic_list()
            doer_active = False
            for pub in pubs:
                (a_topic, _, _) = pub
                if a_topic == topic:
                    rospy.loginfo('found topic published')
                    doer_active = True
                    break
            if not doer_active:
                raise Exception('topic not published, and cannot be started: {}'.format(topic))
        try:
            message = rospy.wait_for_message(topic, topic_type, timeout)
            if active_topic:
                active_topic.lastUse = rospy.get_rostime()
            else:
                active_doers[topic] = ActiveTopic('topic')

        except rospy.ROSException:
            rospy.logwarn('Could not get message for topic {}'.format(topic))
            rospy.logerr(traceback.format_exc())
        return message

if __name__ == '__main__':
    from bdbd_common.srv import SpeechCommand
    #from sensor_msgs.msg import CompressedImage
    TOPIC = '/bdbd/pantilt_camera/image_raw/compressed'
    SERVICE = '/bdbd/chat'
    rospy.init_node('doerRequest')
    nr = DoerRequest()
    nr.ensure_doer('/bdbd/dialog', 'service')
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

