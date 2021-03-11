# single-shot rectify an image, and publish
import cv2
from sensor_msgs.msg import CompressedImage, CameraInfo
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge
import rospy
from bdbd_common.messageSingle import messageSingle

cvBridge = CvBridge()

class ImageRectify():
    def __init__(self, topic_base='/t265/fisheye1', do_publish=True):
        rospy.loginfo('Getting camera_info for ' + topic_base)
        info_msg = messageSingle(topic_base + '/camera_info', CameraInfo)
        rospy.loginfo('Got camera_info')
        self.camera_model(info_msg)
        self.topic_base = topic_base
        self.rect_pub = do_publish and rospy.Publisher(topic_base + '/image_rect/compressed', CompressedImage, queue_size=1)

    def camera_model(self, msg):
        pcm = PinholeCameraModel()
        pcm.fromCameraInfo(msg)
        (self.m1, self.m2) = cv2.fisheye.initUndistortRectifyMap(
            pcm.K, pcm.D[:4], pcm.R, pcm.P,
            (msg.width, msg.height), cv2.CV_32FC1
        )

    def get(self, cam_msg):
        frame = cvBridge.compressed_imgmsg_to_cv2(cam_msg, desired_encoding='bgr8')
        img_rect = cv2.remap(src=frame, map1=self.m1, map2=self.m2, interpolation = cv2.INTER_LINEAR)
        if self.rect_pub:
            img_rect_msg = cvBridge.cv2_to_compressed_imgmsg(img_rect)
            self.rect_pub.publish(img_rect_msg)
        return img_rect

# testing
if __name__ == '__main__':
    rospy.init_node('imageRectify')
    rate = rospy.Rate(10)
    imageRectify = ImageRectify()
    while not rospy.is_shutdown():
        cam_msg = messageSingle(imageRectify.topic_base + '/image_raw/compressed', CompressedImage)
        image_rect = imageRectify.get(cam_msg)
        rate.sleep()
