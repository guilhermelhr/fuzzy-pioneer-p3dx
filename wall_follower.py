import sys, os
os.chdir('C:/Users/Guilherme/workspace/unicamp/mc906/proj3/vrep-robot-master/examples')
sys.path.insert(0, '../src')
from robot import Robot
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

frontDistance = ctrl.Antecedent(np.arange(0, 5.1, .1), 'front distance')
frontDistance['close'] = fuzz.trimf(frontDistance.universe, [0, 0, .3])
frontDistance['good'] = fuzz.trimf(frontDistance.universe, [0, .3, .4])
frontDistance['far'] = fuzz.trimf(frontDistance.universe, [.3, 5, 5])
frontDistance.view()

rearDistance = ctrl.Antecedent(np.arange(0, 5.1, .1), 'rear distance')
rearDistance['close'] = fuzz.trimf(rearDistance.universe, [0, 0, .3])
rearDistance['good'] = fuzz.trimf(rearDistance.universe, [0, .3, .4])
rearDistance['far'] = fuzz.trimf(rearDistance.universe, [.3, 5, 5])
rearDistance.view()

collisionDistance = ctrl.Antecedent(np.arange(0, 5.1, .1), 'collision distance')
collisionDistance['close'] = fuzz.trimf(collisionDistance.universe, [0, 0, 1.5])
collisionDistance['far'] = fuzz.trimf(collisionDistance.universe, [1.5, 5, 5])
collisionDistance.view()

rightMotor = ctrl.Consequent(np.arange(0, 3, .1), 'right motor')
rightMotor['slow'] = fuzz.trimf(rightMotor.universe, [0, 0, .7])
rightMotor['avg'] = fuzz.trimf(rightMotor.universe, [.7, 1, 1.3])
rightMotor['fast'] = fuzz.trimf(rightMotor.universe, [1.3, 2, 2])
rightMotor.view()

leftMotor = ctrl.Consequent(np.arange(0, 3, .1), 'left motor')
leftMotor['slow'] = fuzz.trimf(leftMotor.universe, [0, 0, .7])
leftMotor['avg'] = fuzz.trimf(leftMotor.universe, [.7, 1, 1.3])
leftMotor['fast'] = fuzz.trimf(leftMotor.universe, [1.3, 2, 2])
leftMotor.view()


rules = [
    ctrl.Rule(frontDistance['close'] & rearDistance['close'], rightMotor['fast']),
    ctrl.Rule(frontDistance['close'] & rearDistance['close'], leftMotor['slow']),

    ctrl.Rule(frontDistance['close'] & rearDistance['good'], rightMotor['fast']),
    ctrl.Rule(frontDistance['close'] & rearDistance['good'], leftMotor['slow']),

    ctrl.Rule(frontDistance['close'] & rearDistance['far'], rightMotor['fast']),
    ctrl.Rule(frontDistance['close'] & rearDistance['far'], leftMotor['slow']),

    ctrl.Rule(frontDistance['good'] & rearDistance['close'], rightMotor['slow']),
    ctrl.Rule(frontDistance['good'] & rearDistance['close'], leftMotor['fast']),

    ctrl.Rule(frontDistance['good'] & rearDistance['good'], rightMotor['avg']),
    ctrl.Rule(frontDistance['good'] & rearDistance['good'], leftMotor['avg']),

    ctrl.Rule(frontDistance['good'] & rearDistance['far'], rightMotor['fast']),
    ctrl.Rule(frontDistance['good'] & rearDistance['far'], leftMotor['slow']),

    ctrl.Rule(frontDistance['far'] & rearDistance['close'], rightMotor['slow']),
    ctrl.Rule(frontDistance['far'] & rearDistance['close'], leftMotor['fast']),

    ctrl.Rule(frontDistance['far'] & rearDistance['good'], rightMotor['slow']),
    ctrl.Rule(frontDistance['far'] & rearDistance['good'], leftMotor['fast']),

    ctrl.Rule(frontDistance['far'] & rearDistance['far'], rightMotor['slow']),
    ctrl.Rule(frontDistance['far'] & rearDistance['far'], leftMotor['avg']),

    ctrl.Rule(collisionDistance['close'], leftMotor['slow']),
    ctrl.Rule(collisionDistance['close'], rightMotor['fast']),
]

wallfollower_ctrl = ctrl.ControlSystem(rules)
wallfollower = ctrl.ControlSystemSimulation(wallfollower_ctrl)

robot = Robot()
while(1):
    ultrassonic = robot.read_ultrassonic_sensors()    
    wallfollower.input['rear distance'] = ultrassonic[8]
    wallfollower.input['front distance'] = ultrassonic[7]
    wallfollower.input['collision distance'] = ultrassonic[4]
    wallfollower.compute()
    robot.set_left_velocity(wallfollower.output['left motor'])
    robot.set_right_velocity(wallfollower.output['right motor'])