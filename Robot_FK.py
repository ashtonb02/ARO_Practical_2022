import numpy as np

#Joint angles
#############################################
tc0 = tc1 = tc2 = tc3 = 0
tl0 = tl1 = tl2 = tl3 = tl4 = 0
tr0 = tr1 = tr2 = tr3 = tr4 = 0

#Core of robot
#############################################

COR_WB = np.matrix([[1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,0.85],
                    [0,0,0,1]])

COR_BC0 = np.matrix([[np.cos(tc0),-np.sin(tc0),0,0],
                     [np.sin(tc0),np.cos(tc0),0,0],
                     [0,0,1,0.267],
                     [0,0,0,1]])

COR_C0H0 = np.matrix([[np.cos(tc1),-np.sin(tc1),0,0],
                      [np.sin(tc1),np.cos(tc1),0,0],
                      [0,0,1,0.302],
                      [0,0,0,1]])

COR_H0H1a = np.matrix([[np.cos(tc2),-np.sin(tc2),0,0],
                       [np.sin(tc2),np.cos(tc2),0,0],
                       [0,0,1,0.066],
                       [0,0,0,1]])

COR_H0H1b = np.matrix([[np.cos(tc3),0,np.sin(tc3),0],
                       [0,1,0,0],
                       [-np.sin(tc3),0,np.cos(tc3),0],
                       [0,0,0,1]])

COR_H0H1 = COR_H0H1a * COR_H0H1b

COR_C0R0 = np.matrix([[1,0,0,0.04],
                      [0,1,0,-0.135],
                      [0,0,1,0.1015],
                      [0,0,0,1]])

COR_C0L0 = np.matrix([[1,0,0,0.04],
                      [0,1,0,0.135],
                      [0,0,1,0.1015],
                      [0,0,0,1]])

#Left arm
#############################################

LARM_01 = np.matrix([[np.cos(tl0),-np.sin(tl0),0,0],
                     [np.sin(tl0),np.cos(tl0),0,0],
                     [0,0,1,0.066],
                     [0,0,0,1]])

LARM_12 = np.matrix([[np.cos(tl1),0,np.sin(tl1),0],
                     [0,1,0,0.095],
                     [-np.sin(tl1),0,np.cos(tl1),-0.25],
                     [0,0,0,1]])

LARM_23 = np.matrix([[np.cos(tl2),0,np.sin(tl2),0.1805],
                     [0,1,0,0],
                     [-np.sin(tl2),0,np.cos(tl2),-0.03],
                     [0,0,0,1]])

LARM_34 = np.matrix([[1,0,0,0.1495],
                     [0,np.cos(tl3),-np.sin(tl3),0],
                     [0,np.sin(tl3),np.cos(tl3),0],
                     [0,0,0,1]])

LARM_45 = np.matrix([[np.cos(tl4),0,np.sin(tl4),0],
                     [0,1,0,0],
                     [-np.sin(tl4),0,np.cos(tl4),-0.1335],
                     [0,0,0,1]])
#Right arm
#############################################

RARM_01 = np.matrix([[np.cos(tr0),-np.sin(tr0),0,0],
                     [np.sin(tr0),np.cos(tr0),0,0],
                     [0,0,1,0.066],
                     [0,0,0,1]])

RARM_12 = np.matrix([[np.cos(tr1),0,np.sin(tr1),0],
                     [0,1,0,-0.095],
                     [-np.sin(tr1),0,np.cos(tr1),-0.25],
                     [0,0,0,1]])

RARM_23 = np.matrix([[np.cos(tr2),0,np.sin(tr2),0.1805],
                     [0,1,0,0],
                     [-np.sin(tr2),0,np.cos(tr2),-0.03],
                     [0,0,0,1]])

RARM_34 = np.matrix([[1,0,0,0.1495],
                     [0,np.cos(tr3),-np.sin(tr3),0],
                     [0,np.sin(tr3),np.cos(tr3),0],
                     [0,0,0,1]])

RARM_45 = np.matrix([[np.cos(tr4),0,np.sin(tr4),0],
                     [0,1,0,0],
                     [-np.sin(tr4),0,np.cos(tr4),-0.1335],
                     [0,0,0,1]])


CHEST_FK = np.around(COR_WB * COR_BC0,4)
HEAD_FK = np.around(CHEST_FK * COR_C0H0 * COR_H0H1, 4)
RHAND_FK = np.around(CHEST_FK * COR_C0R0 * RARM_01 * RARM_12 * RARM_23 * RARM_34 * RARM_45, 4)
LHAND_FK = np.around(CHEST_FK * COR_C0L0 * LARM_01 * LARM_12 * LARM_23 * LARM_34 * LARM_45, 4)

print(LHAND_FK)