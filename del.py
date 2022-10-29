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

def RECARR():

    a = []
    a.append([0]*3)
    for n in range(0,10):
        b = list(np.array([n,n+1,n+2]) + np.array([n,n+1,n+2]))
        a.append(b)

    print(a)


RECARR()


