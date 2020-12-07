#!/usr/bin/env python
""" Create a unittest node that tests out python pacakege calc.py"""
import unittest
import rospy
from tower.simulator import Scene
import moveit_commander




class HardCaseNode(unittest.TestCase):

    def test_zero_position(self):
        scene = moveit_commander.PlanningSceneInterface()
        myscene = Scene(scene, False)
        position = myscene.get_cup_position("Cup_0")
        xy = 1
        if position.x == 0 and position.y ==0 and position.z == 0:
            xy = 0

        self.assertEquals( xy , 0)

 
    

if __name__ == "__main__":
    import rosunit
    rosunit.rosrun("tower", "Scene", Scene)  #class in simulator.py
 