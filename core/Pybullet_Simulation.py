from logging import raiseExceptions
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
        'CHEST_JOINT0': np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.267],[0,0,0,1]]),
        'HEAD_JOINT0': np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.302],[0,0,0,1]]),
        'HEAD_JOINT1': np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.066],[0,0,0,1]]),
        'LARM_JOINT0': np.array([[1,0,0,0.04],[0,1,0,0.135],[0,0,1,0.1015],[0,0,0,1]]),
        'LARM_JOINT1': np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.066],[0,0,0,1]]),
        'LARM_JOINT2': np.array([[1,0,0,0],[0,1,0,0.095],[0,0,1,-0.25],[0,0,0,1]]),
        'LARM_JOINT3': np.array([[1,0,0,0.1805],[0,1,0,0],[0,0,1,-0.03],[0,0,0,1]]),
        'LARM_JOINT4': np.array([[1,0,0,0.1495],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),
        'LARM_JOINT5': np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-0.1335],[0,0,0,1]]),
        'RARM_JOINT0': np.array([[1,0,0,0.04],[0,1,0,-0.135],[0,0,1,0.1015],[0,0,0,1]]),
        'RARM_JOINT1': np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.066],[0,0,0,1]]),
        'RARM_JOINT2': np.array([[1,0,0,0],[0,1,0,-0.095],[0,0,1,-0.25],[0,0,0,1]]),
        'RARM_JOINT3': np.array([[1,0,0,0.1805],[0,1,0,0],[0,0,1,-0.03],[0,0,0,1]]),
        'RARM_JOINT4': np.array([[1,0,0,0.1495],[0,1,0,0],[0,0,1,0],[0,0,0,1]]),
        'RARM_JOINT5': np.array([[1,0,0,0],[0,1,0,0],[0,0,1,-0.1335],[0,0,0,1]]),
        'RHAND'      : np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]), # optional
        'LHAND'      : np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]) # optional
    }

    def getJointRotationalMatrix(self, jointName=None, theta=None):
        """
            Returns the 3x3 rotation matrix for a joint from the axis-angle representation,
            where the axis is given by the revolution axis of the joint and the angle is theta.
        """
        if jointName == None:
            raise Exception("[getJointRotationalMatrix] \
                Must provide a joint in order to compute the rotational matrix!")
        # COMPLETE
        # Hint: the output should be a 3x3 rotational matrix as a numpy array
        #return np.matrix()

        #define rotaion matrices for rotation around x, y, & z
        rx = np.matrix([[1,0,0],[0, (np.cos(theta)), -(np.sin(theta))], [0, (np.sin(theta)), (np.cos(theta))]])
        ry = np.matrix([[(np.cos(theta)), 0, (np.sin(theta))], [0,1,0], [-(np.sin(theta)), 0, (np.cos(theta))]])
        rz = np.matrix([[np.cos(theta),-np.sin(theta),0],[np.sin(theta), np.cos(theta),0],[0,0,1]])
        
        #instantiate zero rotation matrix to replace
        j_rotmat = np.matrix([[0,0,0],[0,0,0],[0,0,0]])

        # if axis of rotation is x, use x rotation matrix
        if (self.jointRotationAxis[jointName])[0] == 1:
            j_rotmat = rx

        # if axis of rotation is y, use y rotation matrix
        elif (self.jointRotationAxis[jointName])[1] == 1:
            j_rotmat = ry

        # if axis of rotation is z, use z rotation matrix
        elif (self.jointRotationAxis[jointName])[2] == 1:
            j_rotmat = rz

        else:
            raise Exception("[getJointRotationalMatrix] \
                Joint rotation axis is an invalid format.")   

        return j_rotmat
        

    def getTransformationMatrices(self):
        """
            Returns the homogeneous transformation matrices for each joint as a dictionary of matrices.
        """
        transformationMatrices = {}
        # COMPLETE modify from here
        # Hint: the output should be a dictionary with joint names as keys and
        # their corresponding homogeneous transformation matrices as values.

        for jointName in self.jointRotationAxis:
            rotation_matrix = self.getJointRotationalMatrix(jointName, theta=0)
            whole_position_matrix = self.frameTranslationFromParent[jointName]
            required_position_matrix = []
            
            for n in range(0,2):
                required_position_matrix.append(whole_position_matrix[n][3])

            np.transpose(np.asarray(required_position_matrix))

            tranform_no_padding = np.concatenate(rotation_matrix, required_position_matrix, axis=1)
            padding = np.array([0,0,0,1])

            transformation_matrix = np.concatenate(tranform_no_padding, padding)

            transformationMatrices.update({jointName: transformation_matrix})

        return transformationMatrices

    def getJointLocationAndOrientation(self, jointName):
        """
            Returns the position and rotation matrix of a given joint using Forward Kinematics
            according to the topology of the Nextage robot.
        """
        # Remember to multiply the transformation matrices following the kinematic chain for each arm.
        # COMPLETE modify from here
        # Hint: return two numpy arrays, a 3x1 array for the position vector,
        # and a 3x3 array for the rotation matrix
        #return pos, rotmat

        path = []
        
        if jointName[0] == "R": path += ["RARM_JOINT"+str(n) for n in range(0,6)]
        elif jointName[0] == "L": path += ["LARM_JOINT"+str(n) for n in range(0,6)]
        elif jointName[0] == "H": path += ["HEAD_JOINT"+str(n) for n in range(0,2)]

        HTM = self.getTransformationMatrices()["CHEST_JOINT0"]

        if jointName != "CHEST_JOINT0":
            for joint in path:
                HTM *= self.getTransformationMatrices()[joint] 
                if joint == jointName: break

        pos = np.array([[HTM[0][3]],[HTM[1][3]],[HTM[2][3]]])
        rotmat = np.array([[HTM[0][0],HTM[0][1],HTM[0][2]],
                           [HTM[1][0],HTM[1][1],HTM[1][2]],
                           [HTM[2][0],HTM[2][1],HTM[2][2]]])

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
        # COMPLETE modify from here
        # You can implement the cross product yourself or use calculateJacobian().
        # Hint: you should return a numpy array for your Jacobian matrix. The
        # size of the matrix will depend on your chosen convention. You can have
        # a 3xn or a 6xn Jacobian matrix, where 'n' is the number of joints in
        # your kinematic chain.
        #return np.array()

        # default end effector is Left arm

        peff = self.getJointPosition(endEffector)
        joints = ['CHEST_JOINT0'] 

        if endEffector == 'RARM_JOINT5': joints += ['RARM_JOINT' + str(n) for n in range(0,6)]
        else: joints += ['LARM_JOINT' + str(n) for n in range(0,6)]
        
        # x y z
        #( . . .) n joints downwards |
        #( . . .)                    v
        #( . . .)
        
        #currently only correspodns to position and not orientation

        J = np.array([ (np.cross(self.getJointAxis(k).T, np.subtract(peff,self.getJointPosition(k)))).T for k in joints])
        return J

    # Task 1.2 Inverse Kinematicse

    def inverseKinematics(self, endEffector, targetPosition, orientation, interpolationSteps, maxIterPerStep, threshold):
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
        # TODO add your code here
        # Hint: return a numpy array which includes the reference angular
        # positions for all joints after performing inverse kinematics.
        joints = ['CHEST_JOINT0'] 
        if endEffector == 'RARM_JOINT5': joints += ['RARM_JOINT' + str(n) for n in range(0,5)]
        else: joints += ['LARM_JOINT' + str(n) for n in range(0,5)]
        
        EFpos = self.getJointPosition(endEffector)
        step_positions = np.linspace(EFpos, targetPosition, interpolationSteps)
        traj = [[0]*5]

        for i in range(1,interpolationSteps):
            curr_target = step_positions[i, :]
            dy = curr_target - EFpos
            jacobian = self.jacobianMatrix(endEffector)
            dtheta = np.linalg.pinv(jacobian) * dy
            EFpos += dy
            
            traj.append(traj[i-1]+dtheta)

            if np.abs(EFpos - curr_target) < threshold:
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
        #TODO add your code here
        # iterate through joints and update joint states based on IK solver
        traj = self.inverseKinematics(endEffector, targetPosition, orientation, maxIter, threshold)
        
        joints = ['CHEST_JOINT0'] 
        if endEffector == 'RARM_JOINT5': joints += ['RARM_JOINT' + str(n) for n in range(0,5)]
        else: joints += ['LARM_JOINT' + str(n) for n in range(0,5)]

        for n in range(0,len(joints)): Simulation.p.resetJointState(self, joints[n], traj[n])
        
        #return pltTime, pltDistance
        pass

    def tick_without_PD(self):
        """Ticks one step of simulation without PD control. """
        # TODO modify from here
        # Iterate through all joints and update joint states.
            # For each joint, you can use the shared variable self.jointTargetPos.

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
        u = kp*(x_ref - x_real) + kd*(dx_ref - dx_real) + ki*integral
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

            pltTorque.append(torque)

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

        targetPosition, targetVelocity = float(targetPosition), float(targetVelocity)

        # disable joint velocity controller before apply a torque
        self.disableVelocityController(joint)
        # logging for the graph
        pltTime, pltTarget, pltTorque, pltTorqueTime, pltPosition, pltVelocity = [], [], [], [], [], []

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

        #return pltTime, pltDistance
        pass

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
            torque = 0.0  # TODO: fix me
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
