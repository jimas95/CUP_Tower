
import rospy

class BuildTower():
    def __init__(self):
        rospy.loginfo("position info of building")
        self.TOL = 0.03 # tolerance for cup
        self.radius = 0.025 + self.TOL
        self.height = 0.15 + self.TOL
        self.tableHeight = -0.05 # or the minimum height that the gripper should be allowed to go
        self.centerX = 0.0
        self.towerY = -0.1

    def tower_3_cups(self):
        rospy.loginfo("building tower with 3 cups")
        placePos = []
        useHand = []
        
        # first cup
        useHand.append("right_gripper")
        x = self.centerX - self.radius
        z = self.tableHeight
        placePos.append((x, self.towerY, z))   
        
        # second cup
        useHand.append("left_gripper")
        x = self.centerX + self.radius
        z = self.tableHeight
        placePos.append((x, self.towerY, z))   
        
        # third cup
        useHand.append("right_gripper")
        x = self.centerX
        z = self.tableHeight + self.height
        placePos.append((x, self.towerY, z))       

        return placePos, useHand 
