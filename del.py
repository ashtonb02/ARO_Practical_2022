import numpy as np

def test_HTM():
    p = np.array([0,0,0,1])
    rotmat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
    trans = np.array([2,3,4])

    TransMat4x3 = np.c_[rotmat, trans]
    TransMat4x4 = np.r_[TransMat4x3, [p]]

    print(TransMat4x4)


def test_JLANDO(jointName):

    paths = {"R" : ["RARM_JOINT"+str(n) for n in range(0,6)] + ["RHAND"],
             "L" : ["LARM_JOINT"+str(n) for n in range(0,6)] + ["LHAND"],
             "H" : ["HEAD_JOINT"+str(n) for n in range(0,2)],}

    path = paths[jointName[0]]
    print(path)


def jacobianMatrix():

    paths = {"RHAND" : ['CHEST_JOINT0'] + ["RARM_JOINT"+str(n) for n in range(0,6)],
                 "LHAND" : ['CHEST_JOINT0'] + ["LARM_JOINT"+str(n) for n in range(0,6)],}

    joints = paths["RHAND"]
        
    PosEndEff = np.array([5,3,2])
    jacobian = np.array([[0,0,0]])

    for n in joints:
        RotAxis = np.array([1,0,0])
        PosJon = np.array([4,3,1])
        newrow = np.cross(RotAxis, PosEndEff - PosJon)
        jacobian = np.r_[jacobian, [newrow]]

    print(jacobian[1:])

def IK():
    start = np.array([0,0,0])
    end = np.array([10,10,20])
    traj = np.linspace(start,end,100)

    d = {"a":9,"b":8,"c":0}
    c = {"1":0,"2":0,"3":0}

    c = d
    print(c)

def getJointRotationalMatrix(jointName=None, theta=None):
        """
            Returns the 3x3 rotation matrix for a joint from the axis-angle representation,
            where the axis is given by the revolution axis of the joint and the angle is theta.
        """
        if jointName == None:
            raise Exception("[getJointRotationalMatrix] \
                Must provide a joint in order to compute the rotational matrix!")
        # COMPLETE: modify from here
        # Hint: the output should be a 3x3 rotational matrix as a numpy array
        # return np.matrix()

        rm = {(1, 0, 0): np.matrix([[1,0,0],[0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]]),
              (0, 1, 0): np.matrix([[np.cos(theta), 0, np.sin(theta)], [0,1,0], [-np.sin(theta), 0, np.cos(theta)]]),
              (0, 0, 1): np.matrix([[np.cos(theta),-np.sin(theta),0],[np.sin(theta), np.cos(theta),0],[0,0,1]]),
              (0, 0, 0): np.matrix([[1,0,0],[0,1,0],[0,0,1]])}
       
        print( np.around(rm[tuple((1,0,0))], 3) ) 

def angles(t):
    print( np.around(np.arcsin(np.sin(t)),3) )

angles(np.array([np.pi, 3*np.pi, 5*np.pi/2]))
