#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy

class MoveGroupPyInterface(object):
    """MoveGroupPyInterface"""

    def __init__(self):
        super(MoveGroupPyInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("move_group_python_interface", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        '''
        cylpose = geometry_msgs.msg.PoseStamped()
        cylpose.pose.position.x = 0.65
        cylpose.pose.position.y = 0
        cylpose.pose.position.z = 0.769067
        cylpose.pose.orientation.w = 1.0
        self.scene.add_box("unit_cylinder", cylpose,size = (0.04,0.04,0.1))
        '''

    def go_to_pose_goal(self, x, y, z,  ox, oy, oz, ow):

        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()

        #Quaternion = get_quaternion_from_euler(-2.48, -0.1, -0.6775759)
        # pose_goal.orientation.x = Quaternion[0]
        # pose_goal.orientation.y = Quaternion[1]
        # pose_goal.orientation.z = Quaternion[2]
        # pose_goal.orientation.w = Quaternion[3]
        pose_goal.orientation.x = ox #0.878599
        pose_goal.orientation.y = oy #0.352386
        pose_goal.orientation.z = oz #0.156799
        pose_goal.orientation.w = ow #-0.281607
        pose_goal.position.x = x #0.03
        pose_goal.position.y = y #0.1
        pose_goal.position.z = z #1.4
        
        move_group.set_pose_reference_frame("panda_link0")
        move_group.set_pose_target(pose_goal,"camera_link")

        plan = move_group.plan()
        print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        print("Plan: ",plan[0])
        move_group.set_goal_position_tolerance(0.00001)
        success = move_group.go(wait=True)
        print("Success: ",success)
        print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
	
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose(end_effector_link = "camera_link").pose
        return [plan[0], success]#all_close(pose_goal, current_pose, 0.01)

    def execute_plan(self, plan):

        move_group = self.move_group

        move_group.execute(plan, wait=True)

    def addCollisionObjects(self):
        scene = self.scene
        
        table = geometry_msgs.msg.PoseStamped()
        table.header.frame_id = self.planning_frame 
        table.pose.position.x =  0
        table.pose.position.y = 0
        table.pose.position.z = 0.365
        table.pose.orientation.z = 0
        scene.add_box("table", table,size = (0.8,1.58,0.72))
        
        wall = geometry_msgs.msg.PoseStamped()
        wall.header.frame_id = self.planning_frame 
        wall.pose.position.x =  -0.54
        wall.pose.position.y = 0
        wall.pose.position.z = 1.25
        wall.pose.orientation.z = 0
        scene.add_box("wall", wall,size = (0.1,2.5,2.5))
        
        cylpose = geometry_msgs.msg.PoseStamped() #0.210321, -0.210319
        cylpose.header.frame_id = self.planning_frame 
        cylpose.pose.position.x = 0.649997
        cylpose.pose.position.y = 0
        cylpose.pose.position.z = 0.725 + 0.1/2
        #-0.93604324, -0.00451694, -0.02485613,  0.35097694
        #cylpose.pose.orientation.x = 0#-0.93604324
        #cylpose.pose.orientation.y = 0#-0.00451694
        #cylpose.pose.orientation.z = 0#-0.02485613
        #cylpose.pose.orientation.w = 1#0.35097694
        scene.add_box("unit_cylinder", cylpose,size = (0.08, 0.08, 0.1))

    def plan_cartesian_path(self, scale=1):
        move_group = self.move_group
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold
        
        return plan, fraction

def main():

    move = MoveGroupPyInterface()
    move.addCollisionObjects()

    move.go_to_pose_goal(0.35745, 0.028111, 0.532,  -0.00066075, 0.71554, -0.0004089, 0.698572)

    cartesian_plan, fraction = move.plan_cartesian_path()
    
    move.execute_plan(cartesian_plan)

if __name__ == '__main__':
    main()