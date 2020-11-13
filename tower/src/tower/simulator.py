""" 
Uses the MoveIt Python API to create a planning scene and perform some path planning tasks
SERVICES:
  + <reset> (<Empty>) ~ adds the realsense box to the scene and moves robot to the Home position
  + <step> (<Step>) ~ moves the robot to a user-speciified position
  + <follow> (<Empty>) ~ has the robot move to a specified set of waypoints 
"""
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import MoveItErrorCodes




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


"""
yaml file radious , height 
function add_one_cup --> should take input name , position
add function add_all_cups() --> uses add one cup 
    how many cups 
    random position
    inline position 

create_scene_random_cups
create_scene_inline_cups
attach_box/detach should take name(cup#) input 
function moves one cups
remove stuff like scene = self.scene

get cups that are not in order 


"""


class Scene():
    def __init__(self,myscene):
        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:

        rospy.loginfo("INIT")
        # Misc variables
        # self.scene = moveit_commander.PlanningSceneInterface()
        self.scene = myscene


        # self.add_table()
        # self.add_cup("cup1")
        # self.create_scene_one_cup()
        rospy.loginfo("added scene")

    def add_table(self):
        # add ctable surface 
        box_name = "table"
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = 'world'
        box_pose.pose.orientation.w = 1.0
        
        box_pose.pose.position.x = 1.5
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.0
        self.scene.add_box(box_name, box_pose, size=(1.5, 2.5, 0.2))


        self.wait_for_state_update(object_name= "table", box_is_known=True, timeout=5)
        

    def add_cup(self,name, timeout=4):
        '''
        Adds one cup
        Input:
        timeout (int)
        '''
        height  = 0.1 
        radious = 0.05
        cylinder_pose = geometry_msgs.msg.PoseStamped()
        cylinder_pose.header.frame_id = 'world'
        cylinder_pose.pose.orientation.w = 1.0
        cylinder_pose.pose.position.x = 1.0
        cylinder_pose.pose.position.y = 0.0
        cylinder_pose.pose.position.z = height/2.0

        self.scene.add_cylinder(name,cylinder_pose,height,radious)
        return self.wait_for_state_update(object_name=name,box_is_known=True, timeout=5)


    def create_scene_one_cup(self):
        self.add_table()
        self.add_cup("cup1")


    def wait_for_state_update(self, object_name ,box_is_known=False, box_is_attached=False, timeout=4):
        """Copied from tutorial
        """

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([object_name])
            is_attached = len(attached_objects.keys()) > 0
            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = object_name in self.scene.get_known_object_names()
            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL



    def attach_box(self, timeout=4):
        """Copied from tutorial
        """
    
        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Arm's wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Interbotix
        ## robot, we set ``grasping_group = 'interbotix_gripper'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = 'interbotix_gripper'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        """Copied from tutorial
        """
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        """Copied from tutorial
        """
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
