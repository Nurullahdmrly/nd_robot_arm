#! /usr/bin/env python
import sys
import time
import os
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
import math 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from PyQt5 import QtWidgets
from moveit_commander.exception import MoveItCommanderException
from moveit_commander import move_group

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

def wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
    return False

def add_box(timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    global box_name
    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene between the fingers:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 0
    box_pose.pose.position.z = 0.1 # above the panda_hand frame
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(2, 2, 0.1))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    return wait_for_state_update(box_is_known=True, timeout=timeout)

#add_box()

def plan_cartesian_path( scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL


def display_trajectory( plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    ## END_SUB_TUTORIAL


def execute_plan(plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan, wait=True)


def go_to_joint_state(j0,j1,j2,j3,):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.


    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = j0
    joint_goal[1] = j1
    joint_goal[2] = j2
    joint_goal[3] = j3


    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)
    print(joint_goal)
    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

def go_to_pose_goal(b,c,d,x,y,z,w):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    #arm = move_group.MoveGroupCommander("arm")
    group.set_goal_orientation_tolerance(0.8)
    group.set_goal_position_tolerance(0.02)
    #plan2 = group.go()
    print(group.get_goal_orientation_tolerance(),group.get_goal_position_tolerance())
    rospy.sleep(0.5)
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = x
    pose_goal.orientation.y = y
    pose_goal.orientation.z = z
    pose_goal.orientation.w = w
    pose_goal.position.x = b
    pose_goal.position.y = c
    pose_goal.position.z = d
    

    group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    print("plan executed")
    print(group.get_current_pose())
    print(group.get_current_rpy())
    # For testing:
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()
    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

if __name__ == "__main__":
    # Create a node

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

class Pencere (QtWidgets.QWidget):

    def __init__(self):

        super(Pencere,self).__init__()  #class da tanimladigimiz Qwidget clasina burda ulastik

        self.init_ui()
    
    def init_ui(self):


        self.yazix = QtWidgets.QLineEdit("0")
        self.yaziy = QtWidgets.QLineEdit("0")
        self.yaziz = QtWidgets.QLineEdit("0")
        self.yazi_alani =QtWidgets.QLineEdit("0")
        self.euy =QtWidgets.QLineEdit("0")
        self.euz =QtWidgets.QLineEdit("0")
        self.temizle = QtWidgets.QPushButton("Temizle")
        self.yazdir = QtWidgets.QPushButton("Yazdir")
        self.hesapla = QtWidgets.QPushButton("Hedefe Git")
        self.x = QtWidgets.QLabel("x")
        self.y = QtWidgets.QLabel("y")
        self.z = QtWidgets.QLabel("z")
        self.ex = QtWidgets.QLabel("r")
        self.ey = QtWidgets.QLabel("p")
        self.ez = QtWidgets.QLabel("y")

        self.hes =QtWidgets.QPushButton("Eklemleri oynat")
        self.j0 = QtWidgets.QLineEdit("0")
        self.j1 = QtWidgets.QLineEdit("0")
        self.j2 = QtWidgets.QLineEdit("0")
        self.j3 = QtWidgets.QLineEdit("0")
        self.y0 = QtWidgets.QLabel("Eklem 1")
        self.y1 = QtWidgets.QLabel("Eklem 2")
        self.y2 = QtWidgets.QLabel("Eklem 3")
        self.y3 = QtWidgets.QLabel("Eklem 4")

        h_box = QtWidgets.QHBoxLayout()
        v_box = QtWidgets.QVBoxLayout()
        c_box = QtWidgets.QVBoxLayout()

        c_box.addWidget(self.y0)
        c_box.addWidget(self.j0)
        c_box.addWidget(self.y1)
        c_box.addWidget(self.j1)
        c_box.addWidget(self.y2)
        c_box.addWidget(self.j2)
        c_box.addWidget(self.y3)
        c_box.addWidget(self.j3)
        c_box.addWidget(self.hes)

        v_box.addWidget(self.x)
        v_box.addWidget(self.yazix)
        v_box.addWidget(self.y)
        v_box.addWidget(self.yaziy)
        v_box.addWidget(self.z)
        v_box.addWidget(self.yaziz)
        v_box.addWidget(self.ex)
        v_box.addWidget(self.yazi_alani)
        v_box.addWidget(self.ey)
        v_box.addWidget(self.euy)
        v_box.addWidget(self.ez)
        v_box.addWidget(self.euz)
        
        v_box.addWidget(self.temizle)
        v_box.addWidget(self.yazdir)
        v_box.addWidget(self.hesapla)
        
        h_box.addStretch()
        h_box.addLayout(c_box)
        h_box.addStretch()
        h_box.addLayout(v_box)
        h_box.addStretch()

        self.setLayout(h_box)
        self.temizle.clicked.connect(self.click)
        self.yazdir.clicked.connect(self.click)
        self.hesapla.clicked.connect(self.click)
        self.hes.clicked.connect(self.click)

        self.setWindowTitle("Robot arm goal")
        self.setGeometry(100,100,500,500)

        self.show()

    def click(self):
        sender = self.sender()

        if sender.text() == "Temizle":
            self.yazi_alani.clear()
            self.yazix.clear()
            self.yaziy.clear()
            self.yaziz.clear()
            self.euy()
            self.euz()
        elif sender.text() == "Yazdir":    
            print("x: " + self.yazix.text() + "y: " + self.yaziy.text() + "z: " +self.yaziz.text() + "r: " + self.yazi_alani.text() + "p: " +self.euy.text() + "y: " +self.euz.text())
        elif sender.text() == "Hedefe Git":
            e =float(self.euy.text())
            f = float(self.euz.text())
            d = float(self.yazi_alani.text())
            a = float(self.yazix.text())
            b = float(self.yaziy.text())
            c = float(self.yaziz.text())
            q = quaternion_from_euler(d, e, f)
            f1 = q[0]
            f2 = q[1]
            f3 = q[2]
            f4 = q[3]
            print(f1,f2,f3,f4 )
            go_to_pose_goal(a,b,c,f1,f2,f3,f4)
        else:
            e1 = float(self.j0.text()) 
            e2 = float(self.j1.text()) 
            e3 = float(self.j2.text()) 
            e4= float(self.j3.text()) 
            
            go_to_joint_state(e1,e2,e3,e4)

app = QtWidgets.QApplication(sys.argv)

pencere = Pencere()

sys.exit(app.exec_())
        
