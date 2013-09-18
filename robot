#!/usr/bin/env python

PKG = 'manifold_mapping'
import roslib
roslib.load_manifest(PKG)
import rospy
from sys import argv,exit
from time import time
from math import acos,atan2,hypot
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry
from manifold_mapping.msg import PatchList,Patch,Relation


class Robot(object):
    def __init__(self,rname):
        rospy.init_node('manifold_mapping_robot_%s' % rname)
        self.name = rname
        self.patches = {}  # dictionary: uid => points
        self.last_odom_msg = Odometry()
        self.last_uid = '<start>'
        self.relations = []
        self.waiting = False
        self.patch_pub = rospy.Publisher('/align_patches', PatchList)
        rospy.Subscriber('/patch_cloud', PointCloud, self.laser)
        rospy.Subscriber('/odom', Odometry, self.odom)
        rospy.Subscriber('/landmark_%s' % rname, String, self.landmark)
        rospy.Subscriber('/merged', PatchList, self.reset_patches)

    def reset_patches(self,pl_msg):
        if pl_msg.sender_id != self.name:
            return  # not for us
        print "%s: got patches from server" % self.name
        self.patches.update((p.uid,p.points) for p in pl_msg.patches)
        self.relations.extend(pl_msg.relations)
        self.waiting = False

    def odom(self,msg):
        # decide if we should call new_patch
        dOdom = diff_odom(self.last_odom_msg,msg)
        if hypot(dOdom[0],dOdom[2]) > 0.7 and hasattr(self,"scan"):  # TODO: fix this decision
            while self.scan.header.stamp < msg.header.stamp:
                pass  # align the message times
            self.__make_patch(dOdom)
            self.last_odom_msg = msg
        self.last_dOdom = dOdom

    def __make_patch(self,dOdom):
            print "%s: making patch" % self.name
            uid = str(time())
            self.patches[uid] = self.scan
            # make a relation
            rel = Relation(p1_uid=self.last_uid, p2_uid=uid)
            rel.d,rel.dThetaMove,rel.dThetaPose = dOdom
            self.relations.append(rel)
            # reset caches
            self.last_uid = uid

    def laser(self,cloud):
        self.scan = cloud  # just cache it

    def landmark(self,obs_id):
        if self.waiting:
            print "%s: Can't send patches to the server, still waiting for a response" % self.name
            return
        self.__make_patch(self.last_dOdom)
        self.patch_pub.publish(self.__make_pl_msg(str(obs_id)))
        self.patches.clear()
        self.relations = []
        self.waiting = True

    def __make_pl_msg(self,obs_id):
        pats = [Patch(uid=id,points=ps) for id,ps in self.patches.iteritems()]
        return PatchList(sender_id=self.name,
                         observed_id=obs_id,
                         patches=pats,
                         relations=self.relations)
# end class Robot


def diff_odom(msg1,msg2):
    p1,p2 = msg1.pose.pose, msg2.pose.pose
    a1 = trans_angle(p1.orientation)
    a2 = trans_angle(p2.orientation)
    dtp = a2 - a1
    dx = p2.position.x - p1.position.x
    dy = p2.position.y - p1.position.y
    return (hypot(dx,dy),atan2(dy,dx)-a1,dtp)


def trans_angle(q):
    return 2*acos(q.w)*cmp(q.z,0)

if __name__ == '__main__':
    if len(argv) < 2:
        exit("Usage: %s robot_name" % argv[0])
    r = Robot(argv[1])
    print "%s: started" % argv[1]
    rospy.spin()
