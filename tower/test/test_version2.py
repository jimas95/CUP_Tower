#!/usr/bin/env python
""" Create a unittest node that tests out python pacakege calc.py"""
import unittest
import rospy
from tower.buildTower import BuildTower
import moveit_commander




class HardCaseNode(unittest.TestCase):

    def test_zero_position(self):
        building_tower = BuildTower()
        cup_pos,hand = building_tower.tower_3_cups()
        print(cup_pos)
        # position = myscene.get_cup_position("Cup_0")
        # xy = 0
        # print(position)
        # if position.x == 0 and position.y ==0 and position.z == 0:
        #     xy = 0
        test = True
        if cup_pos[0][0]!=1.0  : test = False
        if cup_pos[1][0]!=1.0  : test = False
        if cup_pos[2][0]!=0.0  : test = False
        if cup_pos[0][1]!=0.05 : test = False
        if hand[0]!="left_gripper" : test = False
        if hand[1]!="right_gripper" : test = False
        # (1.0, 0.05, -0.04), (1.0, -0.05, -0.04), (0, 0, 0), (1.0, 0.0, 0.04)], ['left_gripper', '
        self.assertEquals(test , True)
 
    

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun("tower", "Scene", HardCaseNode)  #class in simulator.py

 