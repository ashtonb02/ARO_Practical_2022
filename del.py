import numpy as np

def test_HTM():
    p = np.array([0,0,0,1])
    rotmat = np.matrix([[1,0,0],[0,1,0],[0,0,1]])
    trans = np.array([2,3,4])

    TransMat4x3 = np.c_[rotmat, trans]
    TransMat4x4 = np.r_[TransMat4x3, [p]]

    print(TransMat4x4)

test_HTM()