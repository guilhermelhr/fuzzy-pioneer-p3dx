import sys, os
os.chdir('C:/Users/Guilherme/workspace/unicamp/mc906/proj3/vrep-robot-master/examples')
sys.path.insert(0, '../src')
from robot import Robot
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

leftDistance = ctrl.Antecedent(np.arange(0, 5.5, .5), 'left distance')
leftDistance['vclose'] = fuzz.trimf(leftDistance.universe, [0, 0, 1])
leftDistance['close'] = fuzz.trimf(leftDistance.universe, [0, 1.5, 2.5])
leftDistance['avg'] = fuzz.trimf(leftDistance.universe, [1.5, 2.5, 3.5])
leftDistance['far'] = fuzz.trimf(leftDistance.universe, [2.5, 3.5, 5])
leftDistance['vfar'] = fuzz.trimf(leftDistance.universe, [4, 5, 5])

#leftDistance.view()

rightDistance = ctrl.Antecedent(np.arange(0, 5.5, .5), 'right distance')
rightDistance['vclose'] = fuzz.trimf(rightDistance.universe, [0, 0, 1])
rightDistance['close'] = fuzz.trimf(rightDistance.universe, [0, 1.5, 2.5])
rightDistance['avg'] = fuzz.trimf(rightDistance.universe, [1.5, 2.5, 3.5])
rightDistance['far'] = fuzz.trimf(rightDistance.universe, [2.5, 3.5, 5])
rightDistance['vfar'] = fuzz.trimf(rightDistance.universe, [4, 5, 5])

#rightDistance.view()

frontDistance = ctrl.Antecedent(np.arange(0, 5.5, .5), 'front distance')
frontDistance['vclose'] = fuzz.trimf(frontDistance.universe, [0, 0, .5])
frontDistance['close'] = fuzz.trimf(frontDistance.universe, [0, 0, 2.5])
frontDistance['avg'] = fuzz.trimf(frontDistance.universe, [0, 2.5, 5])
frontDistance['far'] = fuzz.trimf(frontDistance.universe, [2.5, 5, 5])
#frontDistance.view()

angularVelocity = ctrl.Consequent(np.arange(-4, 5, 1), 'angular velocity')
angularVelocity['right'] = fuzz.trimf(angularVelocity.universe, [-2, -2, 0])
angularVelocity['forward'] = fuzz.trimf(angularVelocity.universe, [-2, 0, 2])
angularVelocity['left'] = fuzz.trimf(angularVelocity.universe, [0, 2, 2])
#angularVelocity.view()

linearVelocity = ctrl.Consequent(np.arange(.1, .6, .1), 'linear velocity')
linearVelocity['slow'] = fuzz.trimf(linearVelocity.universe, [.1, .1, .2])
linearVelocity['avg'] = fuzz.trimf(linearVelocity.universe, [.1, .2, .5])
linearVelocity['fast'] = fuzz.trimf(linearVelocity.universe, [.3, .5, .5])
#linearVelocity.view()

rules = [
    #rules for turning
    ctrl.Rule(rightDistance['vclose'] & leftDistance['vclose'], angularVelocity['left']),
    ctrl.Rule(rightDistance['vclose'] & leftDistance['close'], angularVelocity['left']),
    ctrl.Rule(rightDistance['vclose'] & leftDistance['avg'], angularVelocity['left']),
    ctrl.Rule(rightDistance['vclose'] & leftDistance['far'], angularVelocity['left']),
    ctrl.Rule(rightDistance['vclose'] & leftDistance['vfar'], angularVelocity['left']),
    
    ctrl.Rule(rightDistance['close'] & leftDistance['vclose'], angularVelocity['right']),
    ctrl.Rule(rightDistance['close'] & leftDistance['close'], angularVelocity['left']),
    ctrl.Rule(rightDistance['close'] & leftDistance['avg'], angularVelocity['left']),
    ctrl.Rule(rightDistance['close'] & leftDistance['far'], angularVelocity['left']),
    ctrl.Rule(rightDistance['close'] & leftDistance['vfar'], angularVelocity['left']),
    
    ctrl.Rule(rightDistance['avg'] & leftDistance['vclose'], angularVelocity['right']),
    ctrl.Rule(rightDistance['avg'] & leftDistance['close'], angularVelocity['right']),
    ctrl.Rule(rightDistance['avg'] & leftDistance['avg'], angularVelocity['forward']),
    ctrl.Rule(rightDistance['avg'] & leftDistance['far'], angularVelocity['left']),
    ctrl.Rule(rightDistance['avg'] & leftDistance['vfar'], angularVelocity['left']),
    
    ctrl.Rule(rightDistance['far'] & leftDistance['vclose'], angularVelocity['right']),
    ctrl.Rule(rightDistance['far'] & leftDistance['close'], angularVelocity['right']),
    ctrl.Rule(rightDistance['far'] & leftDistance['avg'], angularVelocity['right']),
    ctrl.Rule(rightDistance['far'] & leftDistance['far'], angularVelocity['forward']),
    ctrl.Rule(rightDistance['far'] & leftDistance['vfar'], angularVelocity['left']),
    
    ctrl.Rule(rightDistance['vfar'] & leftDistance['vclose'], angularVelocity['right']),
    ctrl.Rule(rightDistance['vfar'] & leftDistance['close'], angularVelocity['right']),
    ctrl.Rule(rightDistance['vfar'] & leftDistance['avg'], angularVelocity['right']),
    ctrl.Rule(rightDistance['vfar'] & leftDistance['far'], angularVelocity['right']),
    ctrl.Rule(rightDistance['vfar'] & leftDistance['vfar'], angularVelocity['forward']),
    
    #rules for speed
    ctrl.Rule(frontDistance['vclose'], linearVelocity['slow']),
    ctrl.Rule(frontDistance['close'] | leftDistance['close'] | rightDistance['close'], linearVelocity['slow']),
    ctrl.Rule(frontDistance['avg'], linearVelocity['avg']),
    ctrl.Rule(frontDistance['far'], linearVelocity['fast']),
]

avoidobs_ctrl = ctrl.ControlSystem(rules)
avoidobs = ctrl.ControlSystemSimulation(avoidobs_ctrl)

robot = Robot()

while(1):
    ultrassonic = robot.read_ultrassonic_sensors()    
    avoidobs.input['left distance'] = min(ultrassonic[2], ultrassonic[1])
    avoidobs.input['right distance'] = min(ultrassonic[5], ultrassonic[6])
    avoidobs.input['front distance'] = min(ultrassonic[3], ultrassonic[4])
    avoidobs.compute()
    robot.set_velocity(avoidobs.output['linear velocity'], avoidobs.output['angular velocity'])