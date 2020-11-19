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
    def __init__(self,myscene,REAL_ROBOT):
        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:

        rospy.loginfo("INIT Scene")
        self.scene = myscene
        if(REAL_ROBOT):
            self.gms = self.fake_gms()
            self.sms = self.fake_sms()
        else:
            self.gms = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)
            self.sms = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)


        rospy.loginfo("added scene")

        # OBJECT VARIABLES
        self.cup_radius = rospy.get_param("radius") # cup size
        self.cup_height = rospy.get_param("length")
        self.table_x = rospy.get_param("table_x")
        self.table_y = rospy.get_param("table_y")
        self.table_z = rospy.get_param("table_z")
        self.table_posx = rospy.get_param("t_x")    
        self.table_posy = rospy.get_param("t_y")
        self.table_posz = rospy.get_param("t_z")
        self.number_cups = 3

        # rospy.logerr(self.cup_radius)


        self.cup_n = 3

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

    

    def restart_scene_workStation(self):
        """restarts gazebo scene    """
        pose = Pose()
        twist = Twist()
        pose.position = Point(1,0.0,0.3)
        
        pose.position.y = 0
        self.sms(ModelState("Cup_1",pose,twist,"base"))
        pose.position.y = 0.4
        self.sms(ModelState("Cup_2",pose,twist,"base"))
        pose.position.y = -0.4
        self.sms(ModelState("Cup_3",pose,twist,"base"))

        self.create_scene()

    def restart_scene_inline(self):
        """restarts gazebo scene    """
        pose = Pose()
        twist = Twist()
        pose.position = Point(0.8,0.9,0.1)
        
        self.sms(ModelState("Cup_1",pose,twist,"base"))
        pose.position.x = 1.2
        self.sms(ModelState("Cup_2",pose,twist,"base"))
        pose.position.y = -pose.position.y
        pose.position.x = 0.8
        self.sms(ModelState("Cup_3",pose,twist,"base"))

        self.create_scene() 


    def create_scene(self):
        """Creates scene at moveIt with 3 cups at the table
        """
        cup1 = self.gms("Cup_1","base")
        cup2 = self.gms("Cup_2","base")
        cup3 = self.gms("Cup_3","base")
        self.add_cup("Cup_1",cup1.pose.position)
        self.add_cup("Cup_2",cup2.pose.position)
        self.add_cup("Cup_3",cup3.pose.position)
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
        rospy.logdebug("attach : "+str(cup_name))
        # get a list of known objects in scene
        known_object_list = self.scene.get_known_object_names()
        if cup_name not in known_object_list:
            rospy.logerr("Object %s does not exist in the scene", cup_name)
            rospy.logerr(known_object_list)

        grasping_group = ee_link   # end effector group name
        touch_links = robot.get_link_names(group = grasping_group)
        rospy.logerr(touch_links)
        
        self.scene.attach_mesh(ee_link, cup_name, touch_links = touch_links)   # attach mesh is a moveit commander function


        #wait for planning scene to update
        return self.wait_for_state_update(cup_name, box_is_attached=True, box_is_known=False, timeout=timeout)
      


    def detach_cup(self,cup_name,ee_link, timeout=4):
        """detach a cup from robot

        """
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        rospy.logdebug("detach :"+str(cup_name))
        self.scene.remove_attached_object(ee_link, name=cup_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(cup_name,box_is_known=True, box_is_attached=False, timeout=timeout)


    def fake_sms(self, ModelState):
        pass

    def fake_gms(self, name,base):
        pos = Pose()
        if(name=="Cup_1"):
            pos.position.x= 1.0
            pos.position.y= 0.5
            pos.position.z= -0.06
        elif(name=="Cup_2"):
            pos.position.x= 1.0
            pos.position.y= -0.5
            pos.position.z= 0.06
        elif(name=="Cup_3"):
            pos.position.x= 1.0
            pos.position.y= -0.4
            pos.position.z= 0.01
        elif(name=="Table"):
            pos.position.x= 1.0
            pos.position.y= 0.0
            pos.position.z= 0.0
        return ModelState(name,pos,Twist(), "base")

    def cups_sorted(self):
        """
        Returns True if all cups are inside inLine area 
        Returns False if any cup is still inside the workspace
        """
        cups_list = ["Cup_1", "Cup_2", "Cup_3"]
        for cup in cups_list:
            position = self.get_cup_position(cup)
            y_pos = position.position.y
            rospy.logerr(cup)
            rospy.logerr(position.position.y)
            # if the cup is in the middle two quadrants of the table
            if y_pos < self.table_y/4 and y_pos > -1*self.table_y/4:
                rospy.logerr("HI")
                rospy.logerr(self.table_y/4)
                return False
        return True

    def assing_cup_st1(self,hand):
        """
        this function is for state 1 
        Return the name of a cup that should be grabed from hand arm
        left_hand arm gets y>0 
        right_hand arm gets y<0
        priority is given to cup with min(x)
        """
        pass


    def get_cup_position(self,name):
        """return the position of Cup
        """
        cup = self.gms(name,"base")
        return cup.pose


    def get_next_sorting_position(self,hand):
        """
        Return the position that we should leave cup on inLine workstation
        return type: tuple (x,y)
        """
        #this is fake needs to implemented
        if(hand=="left_hand"):
            if(len(self.sorted_list_pos_left)==0):
                rospy.logerr("ERROR list of sorted points is empty!")
            return self.sorted_list_pos_left.pop()
        elif(hand=="right_hand"):
            return self.sorted_list_pos_right.pop()
        else:
            if(len(self.sorted_list_pos_right)==0):
                rospy.logerr("ERROR list of sorted points is empty!")
            rospy.logerr("ERROR IN get_next_sorting_position")

    def create_sorted_list_position(self,reverse):
        """
        create a list for each hand that has the position we should leave each cup at inLine workstation
        """
        self.sorted_list_pos_left = [] 
        self.sorted_list_pos_right = []


        radious = self.cup_radius

        for i in range(self.cup_n):

            L_pose = Pose()
            L_pose.position.x = 0.6
            L_pose.position.y = 0.8
            L_pose.position.z = -0.05
            L_pose.position.x = L_pose.position.x + 2*radious*i
            self.sorted_list_pos_left.append(L_pose)

            R_pose = Pose()
            R_pose.position.x = 0.6
            R_pose.position.y = -0.8
            R_pose.position.z = -0.05
            R_pose.position.x = R_pose.position.x + 2*radious*i
            rospy.logerr(L_pose)
            self.sorted_list_pos_right.append(R_pose)
        
        if(reverse):
            self.sorted_list_pos_left.reverse()
            self.sorted_list_pos_right.reverse()