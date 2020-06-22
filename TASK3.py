import cv2
import numpy as np
from PIL import Image, ImageFont, ImageDraw
from yolo_tt import YOLO
from robotcontrol import *

def calc_center(out_boxes, out_classes, out_scores, score_limit=0.3):
    outboxes_filter = []
    for x, y, z in zip(out_boxes, out_classes, out_scores):
        if z > score_limit:
            if y == 76:  # 标签为64\76
                outboxes_filter.append(x)

    centers = []
    number = len(outboxes_filter)
    for box in outboxes_filter:
        top, left, bottom, right = box
        center = np.array([[(left + right) // 2], [(top + bottom) // 2]])
        centers.append(center)
    return centers, number,outboxes_filter



#加载keras yolov3 voc预训练模型
yolo_test_args = {
    "model_path": 'model_data/yolo.h5',
    "anchors_path": 'model_data/yolo_anchors.txt',
    "classes_path": 'model_data/coco_classes.txt',
    "score" : 0.3,
    "iou" : 0.45,
    "model_image_size" : (416, 416),
    "gpu_num" : 1,
}

yolo_test = YOLO(**yolo_test_args)


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
            if not vid.isOpened():
                raise IOError("Couldn't open webcam or video")

            video_size = (int(vid.get(cv2.CAP_PROP_FRAME_WIDTH)),
                          int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT)))

            video_FourCC = int(vid.get(cv2.CAP_PROP_FOURCC))
            video_fps = vid.get(cv2.CAP_PROP_FPS)

            out = cv2.VideoWriter('test2.mp4', video_FourCC, video_fps, video_size)

            th = 15
            stride = 0.02

            while True:
                return_value, frame = vid.read()
                #frame = undistortion(frame, mtx, dist[0:4])
                image = Image.fromarray(frame)
                r_image, out_boxes, out_scores, out_classes = yolo_test.detect_image(image)
                #print(out_classes)
                centers, number, outboxes_filter = calc_center(out_boxes, out_classes, out_scores, score_limit=0.3)
                #print(number)
                if number == 0:
                    cv2.imshow('Image',frame)
                    out.write(frame)
                    continue
                else:
                    A = np.array(outboxes_filter).reshape(-1)
                    cv2.rectangle(frame,(A[1], A[0]), (A[3],A[2]), (0, 255, 0), 4)
                    cv2.imshow('Image', frame)
                    out.write(frame)


                # center = np.mean(centers,axis=1)

                zb = np.array(centers).reshape(-1)
                x = zb[0]
                y = zb[1]

                image_c = [int(video_size[0]/2), int(video_size[1]/2)]
                if x < image_c[0] and y < image_c[1]:
                    offset(robot, direction=1, distance=-stride)
                    offset(robot, direction=0, distance=-stride)
                    time.sleep(0.2)

                if x < image_c[0] and y >= image_c[1]:
                    offset(robot, direction=1, distance=-stride)
                    offset(robot, direction=0, distance=stride)
                    time.sleep(0.2)
                if x >= image_c[0] and y < image_c[1]:
                    offset(robot, direction=1, distance=stride)
                    offset(robot, direction=0, distance=-stride)
                    time.sleep(0.2)
                if x >= image_c[0] and y >= image_c[1]:
                    offset(robot, direction=1, distance=stride)
                    offset(robot, direction=0, distance=stride)
                    time.sleep(0.2)

                #if abs(x - image_c[0]) < th and abs(y - image_c[1]) < th:
                #    offset(robot,direction=2, distance=0.05)
                #    break


                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # 断开服务器链接q
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

