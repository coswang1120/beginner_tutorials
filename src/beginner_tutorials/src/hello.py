
# -*- coding: utf-8 -*
import rospy                             # import rospy 模組

rospy.init_node('hello_python_node')     # 初始化 hello_python_node

while not rospy.is_shutdown():           # 在 rospy 還沒結束前，執行下列指令:
    rospy.loginfo('Hello World')         # 印出 Hello World
    rospy.sleep(1)                       # 間隔 1 秒