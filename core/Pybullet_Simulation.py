from cmath import pi
from logging import raiseExceptions
from multiprocessing.dummy import JoinableQueue
from ntpath import join
from turtle import position
from scipy.spatial.transform import Rotation as npRotation
from scipy.special import comb
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import numpy as np
import math
import re
import time
import yaml

from Pybullet_Simulation_base import Simulation_base

class Simulation(Simulation_base):
    """A Bullet simulation involving Nextage robot"""

    def __init__(self, pybulletConfigs, robotConfigs, refVect=None):
        """Constructor
        Creates a simulation instance with Nextage robot.
        For the keyword arguments, please see in the Pybullet_Simulation_base.py
        """
        super().__init__(pybulletConfigs, robotConfigs)
        if refVect:
            self.refVector = np.array(refVect)
        else:
            self.refVector = np.array([1,0,0])

    ########## Task 1: Kinematics ##########
    # Task 1.1 Forward Kinematics
    jointRotationAxis = {
        'base_to_dummy': np.zeros(3),  # Virtual joint
        'base_to_waist': np.zeros(3),  # Fixed joint
        # COMPLETE: modify from here
        'CHEST_JOINT0': np.array([0, 0, 1]),
        'HEAD_JOINT0': np.array([0, 0, 1]),
        'HEAD_JOINT1': np.array([0, 1, 0]),
        'LARM_JOINT0': np.array([0, 0, 1]),
        'LARM_JOINT1': np.array([0, 1, 0]),
        'LARM_JOINT2': np.array([0, 1, 0]),
        'LARM_JOINT3': np.array([1, 0, 0]),
        'LARM_JOINT4': np.array([0, 1, 0]),
        'LARM_JOINT5': np.array([0, 0, 1]),
        'RARM_JOINT0': np.array([0, 0, 1]),
        'RARM_JOINT1': np.array([0, 1, 0]),
        'RARM_JOINT2': np.array([0, 1, 0]),
        'RARM_JOINT3': np.array([1, 0, 0]),
        'RARM_JOINT4': np.array([0, 1, 0]),
        'RARM_JOINT5': np.array([0, 0, 1]),
        'RHAND'      : np.array([0, 0, 1]),
        'LHAND'      : np.array([0, 0, 1])
    }

    frameTranslationFromParent = {
        'base_to_dummy': np.zeros(3),  # Virtual joint
        'base_to_waist': np.zeros(3),  # Fixed joint
        # COMPLETE: modify from here
        'CHEST_JOINT0': np.array([0,0,1.117]),
        'HEAD_JOINT0': np.array([0,0,0.302]),
        'HEAD_JOINT1': np.array([0,0,0.066]),
        'LARM_JOINT0': np.array([0.04,0.135,0.1015]),
        'LARM_JOINT1': np.array([0,0,0.066]),
        'LARM_JOINT2': np.array([0,0.095,-0.25]),
        'LARM_JOINT3': np.array([0.1805,0,-0.03]),
        'LARM_JOINT4': np.array([0.1495,0,0]),
        'LARM_JOINT5': np.array([0,0,-0.1335]),
        'RARM_JOINT0': np.array([0.04,-0.135,0.1015]),
        'RARM_JOINT1': np.array([0,0,0.066]),
        'RARM_JOINT2': np.array([0,-0.095,-0.25]),
        'RARM_JOINT3': np.array([0.1805,0,-0.03]),
        'RARM_JOINT4': np.array([0.1495,0,0]),
        'RARM_JOINT5': np.array([0,0,-0.1335]),
        'RHAND'      : np.array([0,0,0]), # optional
        'LHAND'      : np.array([0,0,0]) # optional
    }

    def getJointRotationalMatrix(self, jointName=None, theta=None):
        """
            Returns the 3x3 rotation matrix for a joint from the axis-angle representation,
            where the axis is given by the revolution axis of the joint and the angle is theta.
        """
        if jointName == None or jointName not in list(self.jointRotationAxis):
            raise Exception("[getJointRotationalMatrix] \
                Must provide a valid joint in order to compute the rotational matrix!")

        # COMPLETE: modify from here
        # Hint: the output should be a 3x3 rotational matrix as a numpy array
        # return np.matrix()

        rm = {(1, 0, 0): np.matrix([[1,0,0],[0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]]),
              (0, 1, 0): np.matrix([[np.cos(theta), 0, np.sin(theta)], [0,1,0], [-np.sin(theta), 0, np.cos(theta)]]),
              (0, 0, 1): np.matrix([[np.cos(theta),-np.sin(theta),0],[np.sin(theta), np.cos(theta),0],[0,0,1]]),
              (0, 0, 0): np.matrix([[1,0,0],[0,1,0],[0,0,1]])}
       
        return rm[tuple(self.jointRotationAxis[jointName])]
        
    def getTransformationMatrices(self):
        """
            Returns the homogeneous transformation matrices for each joint as a dictionary of matrices.
        """
        transformationMatrices = {}
        # COMPLETE: modify from here
        # Hint: the output should be a dictionary with joint names as keys and
        # their corresponding homogeneous transformation matrices as values 
        moveableJoints = list(self.jointRotationAxis)[2:len(list(self.jointRotationAxis))-2]
        for jointName in moveableJoints:
            RotMat = self.getJointRotationalMatrix(jointName, theta=self.getJointPos(jointName))
            Trans = self.frameTranslationFromParent[jointName]

            TransMat = np.array([[RotMat[0,0],RotMat[0,1],RotMat[0,2],Trans[0]],
                                [RotMat[1,0],RotMat[1,1],RotMat[1,2],Trans[1]],
                                [RotMat[2,0],RotMat[2,1],RotMat[2,2],Trans[2]],
                                [0,0,0,1]])
            
            transformationMatrices.update({jointName: TransMat})

        return transformationMatrices

    def getEndEffPath(self, endEffector):

        if endEffector not in list(self.jointRotationAxis):
            raise Exception("[getEndEffPath] \
                Must provide a joint in order to compute the rotational matrix!")

        path = ["CHEST_JOINT0"]
        if endEffector == "CHEST_JOINT0": return path
        
        fullpaths = {"R" : ["RARM_JOINT"+str(n) for n in range(0,int(endEffector[-1])+1)],
                     "L" : ["LARM_JOINT"+str(n) for n in range(0,int(endEffector[-1])+1)],
                     "H" : ["HEAD_JOINT"+str(n) for n in range(0,int(endEffector[-1])+1)],}
        
        path += fullpaths[endEffector[0]]
        return path

    def convertAngle(self, n):
        return np.arcsin(np.sin(n))

    def getJointLocationAndOrientation(self, jointName):
        """
            Returns the position and rotation matrix of a given joint using Forward Kinematics
            according to the topology of the Nextage robot.
        """
        # Remember to multiply the transformation matrices following the kinematic chain for each arm.
        # COMPLETE: modify from here
        # Hint: return two numpy arrays, a 3x1 array for the position vector,
        # and a 3x3 array for the rotation matrix
        #return pos, rotmat

        transMats = self.getTransformationMatrices()

        TransMat = np.identity(4)
        path = self.getEndEffPath(jointName)
        
        for j in path: TransMat=np.matmul(TransMat,transMats[j])

        pos = np.array([[TransMat[0,3]],[TransMat[1,3]],[TransMat[2,3]]])
        rotmat = np.array([[TransMat[0,0],TransMat[0,1],TransMat[0,2]],
                           [TransMat[1,0],TransMat[1,1],TransMat[1,2]],
                           [TransMat[2,0],TransMat[2,1],TransMat[2,2]]])

        return pos, rotmat

    def getJointPosition(self, jointName):
        """Get the position of a joint in the world frame, leave this unchanged please."""
        return self.getJointLocationAndOrientation(jointName)[0]

    def getJointOrientation(self, jointName, ref=None):
        """Get the orientation of a joint in the world frame, leave this unchanged please."""
        if ref is None:
            return np.array(self.getJointLocationAndOrientation(jointName)[1] @ self.refVector).squeeze()
        else:
            return np.array(self.getJointLocationAndOrientation(jointName)[1] @ ref).squeeze()

    def getJointAxis(self, jointName):
        """Get the orientation of a joint in the world frame, leave this unchanged please."""
        return np.array(self.getJointLocationAndOrientation(jointName)[1] @ self.jointRotationAxis[jointName]).squeeze()

    def jacobianMatrix(self, endEffector):
        """Calculate the Jacobian Matrix for the Nextage Robot."""
        # COMPLETE: modify from here
        # You can implement the cross product yourself or use calculateJacobian().
        # Hint: you should return a numpy array for your Jacobian matrix. The
        # size of the matrix will depend on your chosen convention. You can have
        # a 3xn or a 6xn Jacobian matrix, where 'n' is the number of joints in
        # your kinematic chain.
        #return np.array()

        #Jacobian
        #t1 . tn
        #(* * *) x
        #(* * *) y
        #(* * *) z

        joints = self.getEndEffPath(endEffector)
        PosEndEff = self.getJointPosition(endEffector)
        jacobian = np.array([[0],[0],[0]])

        for j in joints:
            RotAxis = self.getJointAxis(j)
            PosJon = self.getJointPosition(j)
            newcol = np.cross(RotAxis, (PosEndEff - PosJon).flatten())
            jacobian = np.c_[jacobian, newcol]
        
        return np.delete(jacobian,0,axis=1)

    # Task 1.2 Inverse Kinematicse

    def inverseKinematics(self, endEffector, targetPosition, orientation, interpolationSteps, threshold):
        """Your IK solver \\
        Arguments: \\
            endEffector: the jointName the end-effector \\
            targetPosition: final destination the the end-effector \\
            orientation: the desired orientation of the end-effector
                         together with its parent link \\
            interpolationSteps: number of interpolation steps
            maxIterPerStep: maximum iterations per step
            threshold: accuracy threshold
        Return: \\
            Vector of x_refs
        """
        # COMPLETE: add your code here
        # Hint: return a numpy array which includes the reference angular
        # positions for all joints after performing inverse kinematics.

        joints = self.getEndEffPath(endEffector)
        EFpos = self.getJointPosition(endEffector).flatten()
        TargetPositions = np.linspace(EFpos,targetPosition,interpolationSteps)

        traj = list(); traj.append([0]*len(joints))
    
        for n in range(1,interpolationSteps):
            dy = TargetPositions[n] - TargetPositions[n-1]
            jacobian = self.jacobianMatrix(endEffector)
            dq = np.matmul(np.linalg.pinv(jacobian), dy)
            angles = list(self.convertAngle(np.array(traj[n-1])+dq))
            traj.append(angles)
            EFpos = self.getJointPosition(endEffector).flatten()
            if np.linalg.norm((targetPosition - EFpos)) < threshold:
                break

        return traj

    def move_without_PD(self, endEffector, targetPosition, speed=0.01, orientation=None,
        threshold=1e-3, maxIter=3000, debug=False, verbose=False):
        """
        Move joints using Inverse Kinematics solver (without using PD control).
        This method should update joint states directly.
        Return:
            pltTime, pltDistance arrays used for plotting
        """
        # COMPLETE: add your code here
        # iterate through joints and update joint states based on IK solver

        pltTime = list()
        pltDistance = list()

        joints = self.getEndEffPath(endEffector)
        angles = self.inverseKinematics(endEffector, targetPosition, orientation, maxIter, threshold)
        
        for n in range(0, maxIter):
            jointStates = dict(zip(joints, angles[n]))
            for j in joints: self.jointTargetPos[j] = jointStates[j]
            self.tick_without_PD()
            pltTime.append(n*self.dt)
            tp = np.transpose(np.array([targetPosition]))
            efp = self.getJointPosition(endEffector)

            pltDistance.append(np.linalg.norm(efp-tp))

            if np.linalg.norm((tp - efp)) < threshold:
                break
        
        return pltTime, pltDistance

    def tick_without_PD(self):
        """Ticks one step of simulation without PD control. """
        # COMPLETE: modify from here
        # Iterate through all joints and update joint states.
        # For each joint, you can use the shared variable self.jointTargetPos.
        
        for j in list(self.jointTargetPos):
            self.p.resetJointState(self.robot, self.jointIds[j], self.jointTargetPos[j])
       
        self.p.stepSimulation()
        self.drawDebugLines()
        time.sleep(self.dt)


    ########## Task 2: Dynamics ##########
    # Task 2.1 PD Controller
    def calculateTorque(self, x_ref, x_real, dx_ref, dx_real, integral, kp, ki, kd):
        """ This method implements the closed-loop control \\
        Arguments: \\
            x_ref - the target position \\
            x_real - current position \\
            dx_ref - target velocity \\
            dx_real - current velocity \\
            integral - integral term (set to 0 for PD control) \\
            kp - proportional gain \\
            kd - derivetive gain \\
            ki - integral gain \\
        Returns: \\
            u(t) - the manipulation signal
        """
        # COMPLETE: Add your code here
        u = kp*(x_ref - x_real) - kd*(dx_real) + ki*integral
        return u

    # Task 2.2 Joint Manipulation
    def moveJoint(self, joint, targetPosition, targetVelocity, verbose=False):
        """ This method moves a joint with your PD controller. \\
        Arguments: \\
            joint - the name of the joint \\
            targetPos - target joint position \\
            targetVel - target joint velocity
        """
        def toy_tick(x_ref, x_real, dx_ref, dx_real, integral):
            # loads your PID gains
            jointController = self.jointControllers[joint]
            kp = self.ctrlConfig[jointController]['pid']['p']
            ki = self.ctrlConfig[jointController]['pid']['i']
            kd = self.ctrlConfig[jointController]['pid']['d']

            ### Start your code here: ###
            # Calculate the torque with the above method you've made
            torque = self.calculateTorque(x_ref, x_real, dx_ref, dx_real, integral, kp, ki, kd)
            ### To here ###

            # send the manipulation signal to the joint
            self.p.setJointMotorControl2(
                bodyIndex=self.robot,
                jointIndex=self.jointIds[joint],
                controlMode=self.p.TORQUE_CONTROL,
                force=torque
            )
            # calculate the physics and update the world
            self.p.stepSimulation()
            time.sleep(self.dt)

            return torque
    
        targetPosition, targetVelocity = float(targetPosition), float(targetVelocity)
        # disable joint velocity controller before apply a torque
        self.disableVelocityController(joint)
        # logging for the graph

        pltTime, pltTarget, pltTorque, pltTorqueTime, pltPosition, pltVelocity = [], [], [], [], [], []

        n=0
        while n < 3000:
            torque = toy_tick(targetPosition, self.getJointPos(joint), targetVelocity, self.getJointVel(joint),0)
            pltTorque.append(torque); pltTorqueTime.append(self.dt*n)
            pltTime.append(self.dt*n); pltTarget.append(targetPosition)
            pltPosition.append(self.getJointPos(joint)); pltVelocity.append(self.getJointVel(joint))
            n += 1

        return pltTime, pltTarget, pltTorque, pltTorqueTime, pltPosition, pltVelocity

    def move_with_PD(self, endEffector, targetPosition, speed=0.01, orientation=None,
        threshold=1e-3, maxIter=3000, debug=False, verbose=False):
        """
        Move joints using inverse kinematics solver and using PD control.
        This method should update joint states using the torque output from the PD controller.
        Return:
            pltTime, pltDistance arrays used for plotting
        """
        #TODO add your code here
        # Iterate through joints and use states from IK solver as reference states in PD controller.
        # Perform iterations to track reference states using PD controller until reaching
        # max iterations or position threshold.

        # Hint: here you can add extra steps if you want to allow your PD
        # controller to converge to the final target position after performing
        # all IK iterations (optional).

        # return pltTime, pltDistance

        pltTime = list()
        pltDistance = list()

        joints = self.getEndEffPath(endEffector)
        angles = self.inverseKinematics(endEffector, targetPosition, orientation, maxIter, threshold)
        
        for n in range(0, maxIter):
            jointStates = dict(zip(joints, angles[n]))
            for j in joints: self.jointTargetPos[j] = jointStates[j]
            self.tick()
            pltTime.append(n*self.dt)
            tp = np.transpose(np.array([targetPosition]))
            efp = self.getJointPosition(endEffector)

            pltDistance.append(np.linalg.norm(efp-tp))

            if np.linalg.norm((tp - efp)) < threshold:
                break
        
        return pltTime, pltDistance
        

    def tick(self):
        """Ticks one step of simulation using PD control."""
        # Iterate through all joints and update joint states using PD control.
        for joint in self.joints:
            # skip dummy joints (world to base joint)
            jointController = self.jointControllers[joint]
            if jointController == 'SKIP_THIS_JOINT':
                continue

            # disable joint velocity controller before apply a torque
            self.disableVelocityController(joint)

            # loads your PID gains
            kp = self.ctrlConfig[jointController]['pid']['p']
            ki = self.ctrlConfig[jointController]['pid']['i']
            kd = self.ctrlConfig[jointController]['pid']['d']

            ### Implement your code from here ... ###
            # TODO: obtain torque from PD controller

            torque = self.calculateTorque(self.jointTargetPos[joint], self.getJointPos(joint), 0, self.getJointVel(joint), 0, kp, ki, kd)
            ### ... to here ###

            self.p.setJointMotorControl2(
                bodyIndex=self.robot,
                jointIndex=self.jointIds[joint],
                controlMode=self.p.TORQUE_CONTROL,
                force=torque
            )

            # Gravity compensation
            # A naive gravitiy compensation is provided for you
            # If you have embeded a better compensation, feel free to modify
            compensation = self.jointGravCompensation[joint]
            self.p.applyExternalForce(
                objectUniqueId=self.robot,
                linkIndex=self.jointIds[joint],
                forceObj=[0, 0, -compensation],
                posObj=self.getLinkCoM(joint),
                flags=self.p.WORLD_FRAME
            )
            # Gravity compensation ends here

        self.p.stepSimulation()
        self.drawDebugLines()
        time.sleep(self.dt)

    ########## Task 3: Robot Manipulation ##########
    def cubic_interpolation(self, points, nTimes=100):
        """
        Given a set of control points, return the
        cubic spline defined by the control points,
        sampled nTimes along the curve.
        """
        #TODO add your code here
        # Return 'nTimes' points per dimension in 'points' (typically a 2xN array),
        # sampled from a cubic spline defined by 'points' and a boundary condition.
        # You may use methods found in scipy.interpolate

        #return xpoints, ypoints
        pass

    # Task 3.1 Pushing
    def dockingToPosition(self, leftTargetAngle, rightTargetAngle, angularSpeed=0.005,
            threshold=1e-1, maxIter=300, verbose=False):
        """A template function for you, you are free to use anything else"""
        # TODO: Append your code here
        pass

    # Task 3.2 Grasping & Docking
    def clamp(self, leftTargetAngle, rightTargetAngle, angularSpeed=0.005, threshold=1e-1, maxIter=300, verbose=False):
        """A template function for you, you are free to use anything else"""
        # TODO: Append your code here
        pass

 ### END
