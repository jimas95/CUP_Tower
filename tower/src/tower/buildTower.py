
import rospy

class BuildTower():
    def __init__(self):
        rospy.loginfo("position info of building")
        self.TOL = 0.03 # tolerance for cup
        self.radius = 0.025 + self.TOL
        self.height = 0.15 + self.TOL
        self.tableHeight = 0 # or the minimum height that the gripper should be allowed to go
        self.centerY = 0.0
        self.towerX = 1.0 #34.5 m

    def tower_3_cups(self):
        rospy.loginfo("building tower with 3 cups")
        placePos = []
        useHand = []
        
        # first cup
        useHand.append("right_gripper")
        y = self.centerY - self.radius
        z = self.tableHeight
        placePos.append((self.towerX, y, z))   
        
        # second cup
        useHand.append("left_gripper")
        y = self.centerY + self.radius
        z = self.tableHeight
        placePos.append((self.towerX, y, z))   
        
        # third cup
        useHand.append("right_gripper")
        x = self.centerY
        z = self.tableHeight + self.height
        placePos.append((self.towerX, y, z))       

        return placePos, useHand 
