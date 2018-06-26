#!/usr/bin/python
import matplotlib.pyplot as plt
import matplotlib
import rospy
from std_msgs.msg import String
import time
import sys

print "Waiting for data..."
def renard(x):
    return ((((x-1)%3)**2)+1)*10**((x-1)/3)


global x
global y
x = [renard(i)*9.81 for i in xrange(1, 11)]
y = []
global val
val = 0.0
global changed
def callback(data):
    global val
    val = float(data.data)

def listener():
    global y
    global x
    rospy.init_node('calibration', anonymous=True)
    rospy.Subscriber("GripperForceSensor", String, callback)
    for i in xrange(1,11):
        print "Place", renard(i), "g on sensor"
        raw_input("Press Enter when ready.")
        values = []
        s = '>'
        sys.stdout.write( 'Gathering values' )
        for i in xrange(2):
            sys.stdout.write(s)
            sys.stdout.flush()
            values.append(val)
            time.sleep(0.1)
        print "\n"
        y.append(sum(values)/len(values))
    font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 22}
    matplotlib.rc('font', **font)
    plt.title('Force vs Voltage', fontsize=22)
    plt.ylabel('Voltage/V', fontsize=16, linespacing=2)
    plt.xlabel('Force/N', fontsize=16, linespacing=2)
    plt.rc('xtick',labelsize=18)
    plt.rc('ytick',labelsize=18)
    plt.plot(x,y)
    plt.show()
#plt.show()
listener()
