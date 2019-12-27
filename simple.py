#!/usr/bin/env python3
import sys
# ----------------------
# 消除ros自带opencv 对环境的影响,自带的是2.7 我们用的3.5
print(sys.path)
list_path = sys.path  # type:list
for index in list_path:
    if '.local/lib/python3.5/site-packages' in index:
        list_path.remove(index)
        list_path.insert(0, index)
        break
print(sys.path)
# --------------------------
import rospy
import time
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import json
import uuid
from threading import Thread
import requests
import cv2
import os
import configparser
import signal
import subprocess


def json_to_dict(json_str) -> (str, dict):
    try:
        dict_ret = json.loads(json_str)
        return 'ok', dict_ret
    except json.JSONDecodeError as e:
        return e, {}


def ros_msg_to_dict(ros_data) -> (str, dict):
    return json_to_dict(ros_data.data)


def get_mac_address():
    node = uuid.getnode()
    mac = uuid.UUID(int=node).hex[-12:]
    mac = str(mac[0:2]) + '-' + str(mac[2:4]) + '-' + str(mac[4:6]) + '-' + str(mac[6:8]) + '-' + str(mac[8:10]) + '-' + str(mac[10:12])
    return mac


task_type_relations = {
    1: 'doorNotCloseCheck',
    2: 'goodsPlacementCheck'
}


class Simple(object):
    def __init__(self):
        self.config_path = os.path.join(os.environ['HOME'], 'catkin_ws/src/xiaoyuan_robot_v2/launch/config.ini')
        if not os.path.isfile(self.config_path):
            list_path = str(self.config_path).split('/')
            config_name = list_path[len(list_path) - 1]
            config_path = str(self.config_path).strip(config_name)
            raise ValueError('config.ini is not existed, please input config.ini to ' + config_path)
        self.config = configparser.ConfigParser()
        self.config.read(self.config_path)
        rospy.init_node('simple')
        rospy.loginfo('init node ok')
        self.amcl_x = 0
        self.amcl_y = 0
        self.progress_percent = 0
        self.line_speed = 0
        self.angle_speed = 0
        self.voltage_percent = 0
        self.index = 0
        self.task_type = ''
        self.picture_path = './snap.jpg'
        self.task_logger_status = False
        self.cap = None  # type:cv2.VideoCapture
        self.status_url = None
        self.logger_url = None
        self.companyId = None
        self.video_ip = None
        self.video_username = None
        self.video_password = None

        self.callback_dict = {}
        self.__init_callback()
        self.get_config()
        task_cap = Thread(target=self.get_rtsp_cap)
        task_cap.start()

        self.robotCode = 'robotId_' + str(get_mac_address())
        rospy.loginfo('robot id is :' + str(self.robotCode))

        rospy.Subscriber('/task_type', Int32, self.task_type_callback)
        self.pub_task_finsh = rospy.Publisher('/task_finish', Bool, queue_size=100)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        rospy.Subscriber('/ros_ui_topic', String, self.callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/closest_wp_index', Int32, self.index_callback)

    def index_callback(self, index:Int32):
        self.index = index.data
        pass

    def get_config(self):
        self.status_url = self.config.get('CONFIG', 'status_url')
        rospy.loginfo('get status_url :' + str(self.status_url))
        if self.status_url is None:
            self.status_url = 'http://10.0.33.25:8090/patrol/noAuthApi/log/statusLog'
        rospy.loginfo('set status_url :' + str(self.status_url))

        self.logger_url = self.config.get('CONFIG', 'logger_url')
        rospy.loginfo('get logger_url: ' + str(self.logger_url))
        if self.logger_url is None:
            self.logger_url = 'http://10.0.33.25:8090/zuul/patrol/noAuthApi/log/checkLog.do'
        rospy.loginfo('set logger_url: ' + str(self.logger_url))

        self.companyId = self.config.get('CONFIG', 'companyId')
        rospy.loginfo('get companyId: ' + str(self.companyId))
        if self.companyId is None:
            self.companyId = 'e4a5d42eb58911e89257000c293b40de'
        rospy.loginfo('set companyId: ' + str(self.companyId))

        self.video_ip = self.config.get('CONFIG', 'video_ip')
        rospy.loginfo('get video_ip: ' + str(self.video_ip))
        if self.video_ip is None:
            self.video_ip = '192.168.1.61'
        rospy.loginfo('set video_ip: ' + str(self.video_ip))

        self.video_username = self.config.get('CONFIG', 'video_username')
        rospy.loginfo('get video_username: ' + str(self.video_username))
        if self.video_username is None:
            self.video_username = 'admin'
        rospy.loginfo('set video_username: ' + str(self.video_username))

        self.video_password = self.config.get('CONFIG', 'video_password')
        rospy.loginfo('get video_password: ' + str(self.video_password))
        if self.video_password is None:
            self.video_password = 'urty1234'
        rospy.loginfo('set video_password: ' + str(self.video_password))

    def __init_callback(self):
        # 内部处理函数初始化
        self.callback_dict['send_info_req'] = self.send_info_req_callback
        self.callback_dict['progress_notify'] = self.progress_callback

    def imu_callback(self, imu_data: Imu):
        self.line_speed = imu_data.linear_acceleration.x
        self.angle_speed = imu_data.linear_acceleration.y

    def send_info_req_callback(self, data):
        if 'voltage_percent' in data:
            self.voltage_percent = data['voltage_percent']
        pass

    def progress_callback(self, data):
        rospy.loginfo('receive process ' + str(data))
        if 'progress_percent' in data:
            self.progress_percent = data['progress_percent']
        pass

    def callback(self, data):
        ret, data_dict = ros_msg_to_dict(data)
        if ret == 'ok' and data_dict is not None:
            if 'msg_id' in data_dict and 'msg_type' in data_dict:

                msg_id = data_dict['msg_id']
                msg_data = ''

                if 'msg_data' in data_dict:
                    msg_data = data_dict['msg_data']

                callback = self.callback_dict.get(msg_id)
                if callback is not None:
                    callback(msg_data)
        else:
            rospy.logwarn(str(data_dict) + '--- msg error : ' + str(ret))
        return

    def get_rtsp_cap(self):
        try:
            result = subprocess.getoutput('ping -c 4 ' + str(self.video_ip))
            if '0 received' in result:
                rospy.logfatal('video ip is not reachable please check!')
                return
            rospy.loginfo('get_rtsp_cap')
            rtsp_url = 'rtsp://' + str(self.video_username) + ':' + str(self.video_password) + '@' + str(self.video_ip) \
                       + '/h264/ch1/main/av_stream'
            self.cap = cv2.VideoCapture(rtsp_url)
            rospy.loginfo('[get_rtsp_cap] get cap')

            if not self.cap.isOpened():
                raise ValueError('rtsp video is not connect,please check!')
        except :
            rospy.logfatal('cant open rtsp video :')

    def record_snap_pic(self):
        rospy.loginfo('record_snap_pic')
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                # cv2.imshow(frame)
                cv2.imwrite(self.picture_path, frame)
            else:
                rospy.logwarn('cant get pic from rtsp :' + str(ret))
        pass

    def task_type_callback(self, task_type:Int32):
        try:
            self.task_type_logic(task_type.data)
        except Exception as e:
            rospy.logwarn('task_type_callback get except: ' + str(e))
        finally:
            result = Bool()  # type:Bool
            result.data = False
            self.pub_task_finsh.publish(result)

    def task_type_logic(self, task_type):
        self.task_type = task_type_relations.get(task_type)
        if self.cap is None:
            rospy.logfatal('rtsp is not connect please check')
            return
        if not self.cap.isOpened():
            rospy.logwarn('rtsp video is not alive, please check')
            return
        self.record_snap_pic()
        if self.task_type is None:
            rospy.logwarn('task_type:' + str(task_type) + ' is not support')
            return

        if self.task_logger_status:
            rospy.logwarn('logger_status is post already please wait!')
            return

        if not os.path.isfile(self.picture_path):
            rospy.logwarn(self.picture_path + ' is not save')
            return

        self.task_logger_status = True
        task_logger = Thread(target=self.do_logger_update)
        task_logger.start()
        pass

    def do_logger_update(self):
        rospy.loginfo('begin to do do_logger_update')
        msg_data = {'companyId': self.companyId, 'robotCode': self.robotCode,
                    'pos': str(self.amcl_x) + ',' + str(self.amcl_y), 'checkLocId': self.index,
                    'checkLocName': 'hahaha',
                    'checkType': str(self.task_type)}
        files = {'img': (self.picture_path, open(self.picture_path, 'rb'), 'image/jpg', {})}
        try:
            requests.post(self.logger_url, data=msg_data, files=files, timeout=20)
        except requests.exceptions.ConnectionError:
            rospy.loginfo('[do_logger_update] something is wrong: ')
        finally:
            self.task_logger_status = False
            os.remove(self.picture_path)
            rospy.loginfo('end to do do_logger_update')

    def amcl_pose_callback(self, pose: PoseWithCovarianceStamped):
        self.amcl_x = pose.pose.pose.position.x
        self.amcl_y = pose.pose.pose.position.y
        rospy.loginfo('receive_position x:' + str(self.amcl_x) + ' y:' + str(self.amcl_y))

    def status_update_task(self):
        try:
            self.status_update_task_proc()
        except BaseException as e:
            print('lets kill self:' + str(e))
            os.kill(os.getpid(), signal.SIGTERM)

    def status_update_task_proc(self):
        update_cnt = 0
        while not rospy.is_shutdown():
            time.sleep(1)
            update_cnt += 1
            # test = Int32()
            # test.data = 1
            # self.task_type_callback(test)
            if update_cnt >= 10:
                update_cnt = 0
                rospy.loginfo('start to send msg to status url : ' + str(self.status_url))
                msg_data = {'companyId': self.companyId, 'robotCode': self.robotCode, 'voltage': self.voltage_percent,
                            'pos': str(self.amcl_x) + ',' + str(self.amcl_y), 'speed': self.line_speed,
                            'angle': self.angle_speed}

                try:
                    req = requests.post(self.status_url, data=msg_data, timeout=10)
                    print(req.text)
                except requests.exceptions.ConnectionError as e:
                    rospy.loginfo('[status_update_task]something is wrong: ' + str(e))

        print('exit status_update_task by user')
        os.kill(os.getpid(), signal.SIGTERM)

    def start(self):
        rospy.loginfo('start simple')

        task_update = Thread(target=self.status_update_task)
        task_update.start()

        while not rospy.is_shutdown():
            time.sleep(1)

        print('start end')


if __name__ == '__main__':
    print('into simple node')
    simple = Simple()
    simple.start()

