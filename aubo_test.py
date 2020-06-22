from robotcontrol import *


def test(test_count):
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
            #
            # # 设置碰撞等级
            robot.set_collision_class(6)
            # 初始化全局配置文件
            robot.init_profile()

            # 设置关节最大加速度
            robot.set_joint_maxacc((0.5, 0.5, 0.5, 0.5, 0.5, 0.5))

            # 设置关节最大加速度
            robot.set_joint_maxvelc((0.5, 0.5, 0.5, 0.5, 0.5, 0.5))

            zeroplace(robot)
            oneplace(robot)
            offset(robot,direction=2,distance=0.1)
            offset(robot, direction=1, distance=0.1)


            # 循环测试
            while test_count > 0:
                test_count -= 1

                offset(robot,direction=2,distance=0.1)
                offset(robot, direction=1, distance=0.1)

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
    #test(test_count=2)
    zeroplace()
