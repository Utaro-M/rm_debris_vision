#!/usr/bin/env python

import rospy
import numpy as np
import numpy.linalg as LA
import math
import copy
import sympy as sym
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from rm_debris_vision.srv import PredK
from rm_debris_vision.srv import PredKResponse
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool

eign_instance=Float64MultiArray()
eign_instance.data=[]
eign_vector_instance=Polygon()
def get_trigger (req):
    global eign_instance
    global eign_vector_instance
    flag = Bool()
    print("get_trigger")
    mat_tmp=np.zeros([1,3])
    # print((req.mat.points[0]).x)    
    for v in req.mat.points:
        # print(np.array([v.x,v.y,v.z]))
        mat_tmp=copy.deepcopy(np.append(mat_tmp,np.array([[v.x,v.y,v.z]]),axis=0))
    
    mat=mat_tmp[1:]
    # print (mat)
    eign_list=LA.eig(mat)
    eign=(eign_list[0]/LA.norm(eign_list[0])).tolist()
    eignvector_list=eign_list[1]
    # if (eign[0] == eign[1]) or (eign[1] == eign[2]) or (eign[2] == eign[0]):
    if (eign[0].imag!=0) or (eign[1].imag!=0) or (eign[2].imag!=0):
        print ("cannot Diagonalization")
        flag.data = False
        # return PredKResponse(eign_instance,eign_vector_instance,flag)
        return PredKResponse(None,None,flag)
        # mat_sym=sym.Matrix(mat.tolist())
        # jordan_eignvector , jordan_eign=mat_sym.jordan_form()
        # eign=[jordan_eign[0,0],jordan_eign[1,1],jordan_eign[2,2]]
        # print ("eign jordon={}".format(eign))
        # # eignvector_list=[jordan_eignvector.row(0),jordan_eignvector.row(1),jordan_eignvector.row(2)]
        # eignvector_list=[jordan_eignvector[0,:].tolist()[0],jordan_eignvector[1,:].tolist()[0],jordan_eignvector[2,:].tolist()[0]]
        # print ("eignvector_list jordon={}".format(eignvector_list))
    else:
        point_list=[Point32(),Point32(),Point32()]
        for i in range(3):
            point_list[i].x=eignvector_list[i][0]
            point_list[i].y=eignvector_list[i][1]
            point_list[i].z=eignvector_list[i][2]
        # eignvector=eign_list[1].tolist()
        eignvector=point_list
        # print ("eign_list={}".format(eign_list))
        eign_instance.data=eign
        eign_vector_instance.points=eignvector
        print ("eign={}".format(eign_instance.data))
        print ("eign_vector={}".format(eign_vector_instance.points))
        flag.data = True
        return PredKResponse(eign_instance,eign_vector_instance,flag)

    # return PredKResponse(eign_instance)
    # return PredKResponse(eign_instance)    
    # eignvector_instance=Polygon()
    # res.eign.data=eign
    # res.eignvector.points=eignvector
    # # for v in eignvector:        
    # #     res.eignvector.points
    # # print ("\n\nset centroid {}\n".format(centroid))
    # return PredKResponse(res)

if __name__ == '__main__':
    print "OK"
    rospy.init_node('pred_k_node')

    pub_info_eign = rospy.Publisher('~eign', Float64MultiArray, queue_size=1)
    pub_info_eign_vector = rospy.Publisher('~eign_vector', Polygon, queue_size=1)
    s = rospy.Service('pred_k', PredK, get_trigger)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
        if eign_instance.data != []:
            pub_info_eign.publish(eign_instance)
            pub_info_eign_vector.publish(eign_vector_instance)
        else:
            None
        # print("Nothing")

