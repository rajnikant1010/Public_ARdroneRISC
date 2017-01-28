#!/usr/bin/env python

import roslib
roslib.load_manifest('data_extraction')
import rospy
import rosbag
import os 		#used to get directory for image topics
import sys 		#used for errors and exiting
import getopt 	#used to parse arguments
import subprocess, yaml #required to check bag file contents

def usage():
    print " -----------------------------------  USAGE:  -------------------------------------------------"
    print " rosrun data_extraction extract_all.py -b rosbag_file_name -o output_dir_name"
    print ""
    print " This code will extract all supported message types from the specified rosbag file. "
    print ""
    print "  rosbag_file_name - path to ros .bag file containing ROS records"
    print "  output_dir_name  - full path directory to save record to. Images are saved to this directory "
    print "                     also. Directory must exist prior to running code.  "
    print "  "
    print "  Currently supported message types are:"
    print "                   - sensor_msgs/Image "
    print "                   - sensor_msgs/Imu "
    print "                   - sensor_msgs/LaserScan "
    print "                   - sensor_msgs/NavSatFix "
    print "                   - gps_common/gpsVel  "
    print "                   - umrr_driver/radar_msg  "
    print "                   - ardrone_autonomy/Navdata "
    print "                   - risc_msgs/Controls "
    print "                   - risc_msgs/Waypoints "
    print "                   - risc_msgs/Cortex "
    print "                   - risc_msgs/Landmarks "
    print "                   - sensor_msgs/Joy "
    print "                   - risc_msgs/Observed_angles "
    print "                   - geometry_msgs/TransformStamped "
    print "  Rosbag Extraction Script v1 - Shane Lynn - 7th May 2012"
    print "  Modified by D. Spencer Maughan for RISC LAb June 2014"
    print "  Modified by D. Spencer Maughan for RISC LAb Sept 2014"
    print "  Modified by D. Spencer Maughan for RISC LAb Oct 2014"
    print "  Modified by D. Spencer Maughan for RISC LAb Feb 2015"
    print "  Modified by D. Spencer Maughan for RISC LAb May 2015"
    print ""
    print " ---------------------------------------------------------------------------------------------"

allowedTopics = ['ardrone_autonomy/Navdata','sensor_msgs/Image', 'sensor_msgs/Imu', 'sensor_msgs/LaserScan',\
        'sensor_msgs/NavSatFix', 'risc_msgs/Trajectories','risc_msgs/Controls', 'risc_msgs/Mocap_data', \
        'risc_msgs/Waypoints','risc_msgs/Landmarks', 'risc_msgs/Cortex','risc_msgs/Observed_angles', 'tf2_msgs/TFMessage',\
        'sensor_msgs/Joy', 'gps_common/GPSFix', 'umrr_driver/radar_msg', 'roscopter/Status','geometry_msgs/TwistStamped',\
        'std_msgs/Bool','geometry_msgs/Point32','three_pi_control/motor_command','three_pi_control/data_log']

def main():
    rospy.loginfo("Processing input arguments:")
    try:
        opts, extraparams = getopt.getopt(sys.argv[1:], "o:b:t:f:") #start at the second argument.
    except getopt.GetoptError, err:
        #print error info and exit
        print str(err)
        usage()
        sys.exit(2)

    #default values
    outDir = "output"
    rosbagFile = "bagfile.bag"
    fn = "Default"
    for o,a in opts:
        if o == "-o":
            outDir = a
        elif o == "-b":
            rosbagFile = a
        elif o == "-f":
            fn = a
        else:
            assert False, "unhandled option"
            usage()
    rospy.loginfo("Opening bag file: " + rosbagFile)
    try:
        bag = rosbag.Bag(rosbagFile)
    except:
        rospy.logerr("Error opening specified bag file : %s"%rosbagFile)
        usage()
        sys.exit(2)

    print "Bag file opened."

    print "Scanning topics..."
    for topic1, msg1, t1 in bag.read_messages(topics='/cortex_raw'):
         SIZE = len(msg1.Obj)
    infoDict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', rosbagFile], stdout=subprocess.PIPE).communicate()[0])

    #we now have all of the topics contained in infoDict. We need to run through these and run the data extract script.
    
    for ii in range(len(infoDict['topics'])):
        topicType = infoDict['topics'][ii]['type']
        print topicType
        topicName = infoDict['topics'][ii]['topic']
        #print topicName
        #is this an processable topic?
        rospy.loginfo("Found topic " + topicName + " of type " + topicType)
        process = False

        for object in allowedTopics:
            if object == topicType:
                process = True
                #print process
                break

        if process == True:
            rospy.loginfo("Processing topic...")
            rospy.loginfo("----------------------- Starting TOPIC_EXTRACT.PY for %s --------------------------"%topicName)
            fileName =  topicName.replace("/", "_")
            if topicName == "/cortex_raw" or topicName == "/data":
               for xr in xrange(1,SIZE+1):
                  commandLine = "rosrun data_extraction extract_topic.py -b " + rosbagFile + " -o " + outDir + "/" + fn + fileName + str(xr) + ".csv" + " -t " + topicName + " -n " + str(xr)
                  os.system(commandLine)
                  print "Finished Extracting " + topicName + str(xr)
            else:
               commandLine = "rosrun data_extraction extract_topic.py -b " + rosbagFile + " -o " + outDir + "/" + fn + fileName + ".csv" + " -t " + topicName + " -n 1"
               os.system(commandLine)
               print "Finished Extracting " + topicName
            
            rospy.loginfo("----------------------- Finished TOPIC_EXTRACT.PY for %s --------------------------"%topicName)
        else:
            rospy.loginfo("Topic Type not supported.")

    #Summarise for user
    bag.close()

    rospy.loginfo("Completed for all compatible topics.")

main()


#

# Need to define

#os.system(identify Image_0001.jpg) outputs widthxheight

#from files_per_second = number of msgs/duration, bitrate= high number usually 2400, topic_name = name of topic of type sensor_msgs/Image

#then run

#os.system(mencoder -nosound mf://*.jpg -mf w=width:h=height:type=jpg:fps=f_p_s \ -ovc lavc -lavcopts vcodec=mpeg4:vbitrate=bitrt \ :mbd=2:keyint=132:v4mv:vqmin=3:lumi_mask=0.07:dark_mask=0.2:mpeg_quant:scplx_mask=0.1:tcplx_mask=0.1:naq -o topic_name.avi -ovc lavc)

