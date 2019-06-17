import sys, os
os.chdir('C:/Users/Guilherme/workspace/unicamp/mc906/proj3/vrep-robot-master/examples')
sys.path.insert(0, '../src')
from robot import Robot
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

leftDistance = ctrl.Antecedent(np.arange(0, 6, 1), 'left distance')
rightDistance = ctrl.Antecedent(np.arange(0, 6, 1), 'right distance')
rightMotor = ctrl.Consequent(np.arange(-10, 11, 1), 'right motor')
leftMotor = ctrl.Consequent(np.arange(-10, 11, 1), 'left motor')

leftDistance['near'] = fuzz.trimf(leftDistance.universe, [0, 0, 3])
leftDistance['far'] = fuzz.trimf(leftDistance.universe, [2, 5, 5])
#leftDistance.view()

rightDistance['near'] = fuzz.trimf(rightDistance.universe, [0, 0, 3])
rightDistance['far'] = fuzz.trimf(rightDistance.universe, [2, 5, 5])
#rightDistance.view()

rightMotor['reverse'] = fuzz.trimf(rightMotor.universe, [-10, -10, 1])
rightMotor['forward'] = fuzz.trimf(rightMotor.universe, [0, 10, 10])
#rightMotor.view()

leftMotor['reverse'] = fuzz.trimf(leftMotor.universe, [-10, -10, 1])
leftMotor['forward'] = fuzz.trimf(leftMotor.universe, [0, 10, 10])
#leftMotor.view()

rule1 = ctrl.Rule(rightDistance['near'], rightMotor['forward'])
rule2 = ctrl.Rule(rightDistance['near'], leftMotor['reverse'])
rule3 = ctrl.Rule(leftDistance['near'], rightMotor['reverse'])
rule4 = ctrl.Rule(leftDistance['near'], leftMotor['forward'])
rule5 = ctrl.Rule(rightDistance['far'] & leftDistance['far'], rightMotor['forward'])
rule6 = ctrl.Rule(rightDistance['far'] & leftDistance['far'], leftMotor['forward'])

avoidobs_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6])
avoidobs = ctrl.ControlSystemSimulation(avoidobs_ctrl)

robot = Robot()

while(1):
    ultrassonic = robot.read_ultrassonic_sensors()
    
    avoidobs.input['left distance'] = ultrassonic[2]
    avoidobs.input['right distance'] = ultrassonic[5]
    avoidobs.compute()
    robot.set_left_velocity(avoidobs.output['left motor'])
    robot.set_right_velocity(avoidobs.output['right motor'])