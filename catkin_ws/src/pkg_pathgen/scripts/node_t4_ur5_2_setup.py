#!/usr/bin/env python

from warehouse.warehouse import *


def main():
    # Start a ROS node with name 'node_t3_ur5_1_pick_place'
    rospy.init_node('node_t3_ur5_1', anonymous=True, disable_signals=True)

    # Create an object of the class UR5_1
    cam = Camera2D()
    rospy.sleep(0.5)

    colors = cam.get_colors()
    shelf = Shelf(colors)
    ur5_1 = UR5('ur5_1')
    rounds = 10
    plan_attempts = 25

    row = [1.89, 1.647, 1.4275, 1.195]
    column = [0.28, 0, -0.28]

    # ur5_1.move_group.set_planning_time(60)

    for i in range(12):
        package = shelf.get(i)
        if package is not None:
            x, y = package.position
            # 6.59
            ur5_1.add_box(package, column[y], -0.415, row[x])

    joint_angles = [
        [-1.039598117880531, -1.2263407308198744,
         0.08766887923253197, -2.0040433272824654,
         -2.1016409757635053, 3.140544837838009],
        [-2.1173800934124323, -1.1568549046240593,
         -0.3926495162989294, -1.5912018726238966,
         -1.0251209147523408, 3.141054694809503],
        [1.0397681935457719, -1.9126739136003623,
         -0.09278965547275764, -1.1359764411575597,
         2.102102577115158, 3.1411611510784994],
        [-0.9604543910366887, -1.4300591044500752,
         0.6286661035013914, 0.8010355345825566,
         2.1807513545214725, -4.652179845354709e-05],
        [-2.1178226342326436, -1.7800945394870462,
         0.9600970907171131, 0.818852786109403,
         1.0236410292815883, 0.00038336247137316093],
        [0.9607018865909129, -1.7114205967685603,
         -0.6292788710127359, 2.3397495908311186,
         -2.180496021692367, -3.457100070125563e-05],
        [-0.9607542320654758, -1.7004420123819601,
         1.5430589876378304, 0.1569416191046482,
         2.1804826024402093, -0.0011171343740086215],
        [-2.1173937279883663, -2.0554227863706025,
         1.7794153693662125, 0.2761327317500566,
         1.0244997580809692, 0.0001159282947940099],
        [0.9608232301215871, -1.4416325345470273,
         -1.542353394929746, 2.9829822485761266,
         -2.181262433808868, -0.0008312593490700237],
        [-0.9609192089764953, -1.6092656004622077,
         2.083684802209344, -0.4737098360681413,
         2.1798764814624683, 0.0010164451194416557],
        [-2.117058728768468, -2.03062812314778,
         2.3830978312053315, -0.3514397131160969,
         1.0250250419482914, -0.0005404553851322547],
        [0.9605936424587247, -1.5259875995406365,
         -2.0752874172898856, -2.681681794352338,
         -2.1812980830328303, 0.0008248485476878287]]

    play_traj(ur5_1, 'ur5_1_default')
    ur5_1.move_group.stop()

    jas = ur5_1.move_group.get_current_joint_values()

    n = [11]

    for pack in n:
        package = shelf.get(pack)
        x, y = package.position
        p = geometry_msgs.msg.Pose()
        p.position.x = column[y]
        p.position.y = -0.21
        p.position.z = row[x]
        if x == 0:
            if y == 0:
                p.position.x -= 0.03
            elif y == 2:
                p.position.x += 0.03
            else:
                p.position.z = 1.915
        p.orientation.x = -1.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 0.0

        t_min = rospy.rostime.Duration(0, read_dur('{}{}'.format(x, y)))
        print t_min

        for i in range(rounds):
            print i + 1
            for _ in range(plan_attempts):
                t1 = rospy.Time.now()
                play_traj(ur5_1, 'ur5_1_default_{}{}'.format(x, y))
                ur5_1.move_group.stop()
                t2 = rospy.Time.now()
                if (t2 - t1).secs > 1:
                    break
            else:
                print 'DED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
                rospy.spin()
            ur5_1.attach_box(package)
            rospy.sleep(0.5)
            ur5_1.move_group.set_joint_value_target(jas)
            rospy.sleep(0.5)
            for _ in range(plan_attempts):
                plan2 = ur5_1.move_group.plan()
                t3 = rospy.Time.now()
                f2 = ur5_1.move_group.execute(plan2, wait=True)
                ur5_1.move_group.stop()
                t4 = rospy.Time.now()
                if f2 and (t4 - t3).secs > 1:
                    break
            else:
                print 'DED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
                ur5_1.detach_box(package)
                ur5_1.move_group.set_joint_value_target(jas)
                for _ in range(plan_attempts):
                    plan1 = ur5_1.move_group.plan()
                    t1 = rospy.Time.now()
                    f1 = ur5_1.move_group.execute(plan1, wait=True)
                    ur5_1.move_group.stop()
                    if f1:
                        break
                continue
            ur5_1.detach_box(package)
            ur5_1.remove_box(package)
            rospy.sleep(0.5)
            ur5_1.add_box(package, column[y], -0.415, row[x])

            # noinspection PyUnboundLocalVariable
            dt = t4 - t3 + t2 - t1
            print '**********************************************************************'
            print 'current:\t', dt
            print 'best:\t\t', t_min
            print '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
            # noinspection PyUnboundLocalVariable
            if dt < t_min and f2:
                t_min = dt
                # noinspection PyUnboundLocalVariable
                # plan1min = plan1
                # noinspection PyUnboundLocalVariable
                plan2min = plan2

                # noinspection PyUnboundLocalVariable
                # save_traj(plan1min, 'ur5_1_default_{}{}'.format(x, y))
                # noinspection PyUnboundLocalVariable
                save_traj(plan2min, 'ur5_1_{}{}_default'.format(x, y))
                save_txt(str(t_min), '{}{}'.format(x, y))


if __name__ == '__main__':
    try:
        main()
    except Exception, e:
        cv2.destroyAllWindows()
        print e
