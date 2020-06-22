from robotcontrol import *
import cv2
from PIL import Image
import numpy as np
from biaoding import undistortion
from timeit import default_timer as timer
font = cv2.FONT_HERSHEY_SIMPLEX  # 设置字体样式
from numpy import cos,sin

npzfile = np.load('calibrate.npz')
mtx = npzfile['mtx']
dist = npzfile['dist']


def main():

    # 初始化logger
    logger_init()
    # 启动测试
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))
    # 系统初始化
    Auboi5Robot.initialize()
    # 创建机械臂控制类
    robot = Auboi5Robot()
    # 创建上下文
    handle = robot.create_context()
    # 打印上下文
    logger.info("robot.rshd={0}".format(handle))
    try:
        # 链接服务器
        ip = '192.168.1.150'
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            # # 上电
            robot.robot_startup()
            # # 设置碰撞等级
            robot.set_collision_class(6)
            # 初始化全局配置文件
            robot.init_profile()
            # 设置关节最大加速度
            robot.set_joint_maxacc((0.5, 0.5, 0.5, 0.5, 0.5, 0.5))
            # 设置关节最大加速度
            robot.set_joint_maxvelc((0.5, 0.5, 0.5, 0.5, 0.5, 0.5))
           # oneplace(robot)

            vid = cv2.VideoCapture(1)
            #vid.set(3,640)
            #vid.set(4,480)
            if not vid.isOpened():
                raise IOError("Couldn't open webcam or video")
            #video_FourCC = int(vid.get(cv2.CAP_PROP_FOURCC))
            #video_fps = vid.get(cv2.CAP_PROP_FPS)
            video_size = (int(vid.get(cv2.CAP_PROP_FRAME_WIDTH)),
                          int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT)))

            #out = cv2.VideoWriter('1.mp4', video_FourCC, video_fps, video_size)
            th = 10
            #lower = np.array([0, 0, 50])
            #upper = np.array([255, 255, 60])

            #lower = np.array([0, 50, 50])
            #upper = np.array([10, 255, 255])
            lower = np.array([11, 43, 46])
            upper = np.array([25, 255, 255])
            #lower = np.array([0, 0, 0])
            #upper = np.array([180,255,46])
            stride = 0.02

            while True:
                return_value, frame = vid.read()
                frame = undistortion(frame, mtx, dist[0:4])
                #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                #gray = cv2.GaussianBlur(gray, (5, 5), 0)
                HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  #
                mask = cv2.inRange(HSV, lower, upper)
                #res = cv2.bitwise_and(frame, frame, mask=mask)
                edges = cv2.Canny(mask, 50, 100)

                # 圆
                circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 100, param1=100, param2=10, minRadius=10, maxRadius=500)
                if circles is not None:
                    x = int(circles[0][0][0])
                    y = int(circles[0][0][1])
                    r = int(circles[0][0][2])
                    cv2.circle(frame, (x, y), r, (0, 0, 255), 3)
                    cv2.circle(frame, (x, y), 3, (255, 255, 0), -1)

                    #cv2.putText(frame, "center", (x - 20, y - 20),
                    #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


                    image_c = [int(video_size[0]/2), int(video_size[1]/2)]
                    #image_offset = [x - image_c[0], y -image_c[1]]

                    #while abs(x - image_c[0]) > 20 and abs(y - image_c[1]) > 20:
                    if abs(x - image_c[0]) > th:
                        if x < image_c[0]:
                            offset(robot, direction=1, distance=-stride)
                            time.sleep(0.2)
                        else:
                            offset(robot, direction=1, distance=stride)
                            time.sleep(0.2)

                    if abs(y - image_c[1]) > th:
                        if y < image_c[1]:
                            offset(robot, direction=0, distance=-stride)
                            time.sleep(0.2)
                        else:
                            offset(robot, direction=0, distance=stride)
                            time.sleep(0.2)

                    if abs(x - image_c[0]) < th and abs(y - image_c[1]) < th:
                        offset(robot,direction=2, distance=0.2)
                        time.sleep(0.2)
                        break

                cv2.imshow('frame', frame)
                #cv2.imshow('mask', mask)
               # out.write(frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # 断开服务器链接
            robot.disconnect()

    except RobotError as e:
        logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))


    finally:
        # 断开服务器链接
        if robot.connected:
            # 关闭机械臂
            robot.robot_shutdown()
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))



def offset(robot,direction=0, distance=0.1,zitai=True):
    ## 偏移
    ## direction: x-0, y-1, z-2
    ## distance: mm
    current_pos = robot.get_current_waypoint()
    current_pos['pos'][direction] -= distance
    ik_result = robot.inverse_kin(current_pos['joint'], current_pos['pos'], current_pos['ori'])
    if zitai:
        robot.move_line(ik_result['joint'])
    else:
        robot.move_joint(ik_result['joint'])

def oneplace(robot):
 ###  合适的位置
    joint_radian = (0, -0.225068, 0.948709, -0.397018, pi / 2, -0.541673)
    robot.move_joint(joint_radian)

def zeroplace(robot):
    ##  零位
    joint_radian = (0, 0, 0, 0, 0, 0)
    robot.move_joint(joint_radian)


if __name__ == '__main__':
    main()
