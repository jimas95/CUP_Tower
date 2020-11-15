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
from gazebo_msgs.srv import GetModelState,SetModelState
from geometry_msgs.msg import Pose,Point,Twist
from gazebo_msgs.msg import ModelState



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

        rospy.loginfo("INIT Scene")
        self.scene = myscene
        self.gms = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)
        self.sms = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)


        rospy.loginfo("added scene")

        # OBJECT VARIABLES
        self.cup_radius = rospy.get_param("radius") # cup size
        self.cup_height = rospy.get_param("length")
        self.table_x = 1.1 # table size
        self.table_y = 2.1
        self.table_z = 0.05

        # rospy.logerr(self.cup_radius)




    # Functions that add objects to scene
    def add_table(self,name,position, timeout=4):
        """Adds table object to planning scene
        """
        # add ctable surface 
        box_name = name
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = 'world'
        box_pose.pose.orientation.w = 1.0
        
        box_pose.pose.position.x = position.x
        box_pose.pose.position.y = position.y
        box_pose.pose.position.z = position.z
        self.scene.add_box(box_name, box_pose, size=(self.table_x, self.table_y, self.table_z))


        self.wait_for_state_update(object_name= "table", box_is_known=True, timeout=5)
        

    def add_cup(self,name, position, timeout=4):
        """Adds a cup object at a specified position
        Args:
            name (str): name of cup to place in scene (ex. cup3)
            posistion (????): postion of specified cup in planning scene
            timeout (int): we wait until updates have been made or "timeout" seconds have passed
        Returns: 
            True or False (bool): pending on result from wait_for_state_update
        """
        cylinder_pose = geometry_msgs.msg.PoseStamped()
        cylinder_pose.header.frame_id = 'world'
        cylinder_pose.pose.orientation.w = 1.0
        cylinder_pose.pose.position.x = position.x
        cylinder_pose.pose.position.y = position.y
        cylinder_pose.pose.position.z = position.z # if on first row height should be: self.cup_height/2.0 + self.table_z/2

        self.scene.add_cylinder(name,cylinder_pose,self.cup_height,self.cup_radius)
        return self.wait_for_state_update(object_name=name,box_is_known=True, timeout=5)

    

    def restart_scene(self):
        "restarts gazebo scene"
        pose = Pose()
        twist = Twist()
        pose.position = Point(1,0.0,0.3)
        

        pose.position.y = 0
        self.sms(ModelState("Cup_1",pose,twist,"base"))
        pose.position.y = 0.5
        self.sms(ModelState("Cup_2",pose,twist,"base"))
        pose.position.y = -0.5
        self.sms(ModelState("Cup_3",pose,twist,"base"))
        
    def get_cup_position(self,name):
        """return the position of Cup
        """
        cup = self.gms(name,"base")
        return cup.pose

    def create_scene_one_cup(self):
        """Dimitris
        """
        cup1 = self.gms("Cup_1","base")
        cup2 = self.gms("Cup_2","base")
        cup3 = self.gms("Cup_3","base")
        self.add_cup("cup1",cup1.pose.position)
        self.add_cup("cup2",cup2.pose.position)
        self.add_cup("cup3",cup3.pose.position)
        table = self.gms("Table","base")
        self.add_table("Table",table.pose.position)





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



    # def attach_cup(self, ee_link, cup_name, robot,  timeout=4):
    def attach_cup(self, ee_link, robot,cup_name,  timeout=4):
        """Attaches objects to the robot.
        Adds link names to touch_links array. 
        This tells the planning scene to ignore collisons between the robot and 
        cups.
        Args:
             ee_link (str): name of end effector group name
             cup_name (str) : cup object to attach to robot
             robot (RobotComander object): provides info about robot
        """
        # get a list of known objects in scene
        known_object_list = self.scene.get_known_object_names()
        if cup_name not in known_object_list:
           rospy.logerr("Object %s does not exist in the scene", cup_name)
        rospy.logerr(known_object_list)

        # start attach object code from tutorial
        grasping_group = ee_link   # end effector group name
        rospy.logerr(grasping_group)
        touch_links = robot.get_link_names(group = grasping_group)
        
        #http://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html
        self.scene.attach_mesh(ee_link, known_object_list[1], touch_links = touch_links)   # attach mesh is a moveit commander function


        #wait for planning scene to update
        return self.wait_for_state_update(known_object_list[1], box_is_attached=True, box_is_known=False, timeout=timeout)
      
   



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
