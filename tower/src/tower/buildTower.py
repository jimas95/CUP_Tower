
import rospy

class BuildTower():
    def __init__(self):
        rospy.loginfo("position info of building")
        self.TOL = 0.03 # tolerance for cup
        self.TOL_table = 0.25 # tolerance for table
        #TODO: yaml
        self.radius = rospy.get_param("radius") + self.TOL # cup size
        self.height = rospy.get_param("length") + self.TOL
        self.tableHeight =  rospy.get_param("t_z") + self.TOL_table # or the minimum height that the gripper should be allowed to go
        self.centerY = 0.0
        self.towerX = 1.0   #34.5 cm
        self.POS_Z = -0.04

    def tower_3_cups(self):
        """ build tower with three cups

            Returns:
                placePos (list of (x,y,z)) - places to put cups.
                useHand (list of string) - "left_gripper" or "right_gripper". Order to place cups.
        """
        rospy.loginfo("building tower with 3 cups")
        placePos = []
        useHand = []
        
        # 1 cup
        useHand.append("left_gripper")
        y = self.centerY + self.radius
        z = self.POS_Z
        placePos.append((self.towerX, y, z))   

        # 2 cup
        useHand.append("right_gripper")
        y = self.centerY - self.radius
        z = self.POS_Z
        placePos.append((self.towerX, y, z))   
        
        # no cup
        useHand.append("left_gripper")
        placePos.append((0, 0, 0))   
        
        # 4 cup
        useHand.append("right_gripper")
        y = self.centerY
        z = self.POS_Z + self.height
        placePos.append((self.towerX, y, z))       

        return placePos, useHand 

    def tower_6_cups(self):
        """ build tower with six cups

            Returns:
                placePos (list of (x,y,z)) - places to put cups.
                useHand (list of string) - "left_gripper" or "right_gripper". Order to place cups.
        """
        rospy.loginfo("building tower with 3 cups")
        placePos = []
        useHand = []
        
        # 1 cup
        useHand.append("left_gripper")
        y = self.centerY
        z = self.tableHeight
        placePos.append((self.towerX, y, z))   

        # 2 cup
        useHand.append("right_gripper")
        y = self.centerY - self.radius*2
        z = self.tableHeight
        placePos.append((self.towerX, y, z))   
        
        # 3 cup
        useHand.append("left_gripper")
        y = self.centerY + self.radius*2
        z = self.tableHeight 
        placePos.append((self.towerX, y, z)) 
        
        # 4 cup
        useHand.append("right_gripper")
        y = self.centerY - self.radius
        z = self.tableHeight + self.height
        placePos.append((self.towerX, y, z))      
             
        # 5 cup
        useHand.append("left_gripper")
        y = self.centerY + self.radius
        z = self.tableHeight + self.height
        placePos.append((self.towerX, y, z))      

        # 6 cup
        useHand.append("right_gripper")
        y = self.centerY
        z = self.tableHeight + self.height*2
        placePos.append((self.towerX, y, z))       

        return placePos, useHand 
