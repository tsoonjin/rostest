#!/usr/bin/env python
import rosbag
import sys

inbag = sys.argv[1]
outbag = sys.argv[2]
lowTime = int(sys.argv[3])
hiTime = int(sys.argv[4])
epoch = True
initTime = 0

print("IN: " + inbag + " OUT: " + outbag)
with rosbag.Bag(outbag, 'w') as output:
    for topic, msg, t in rosbag.Bag(inbag).read_messages():
        if epoch:
            initTime = t.to_sec()
            epoch = False
        timePassed = t.to_sec() - initTime
        if(lowTime <= timePassed <= hiTime):
            output.write(topic,msg,t)
        if(timePassed > hiTime):
            break
        print("Duration passed: " + repr(timePassed))


