#!/usr/bin/env python

import roslib
roslib.load_manifest('data_extraction')
import rospy
import rosbag
import cv
from cv_bridge import CvBridge, CvBridgeError
#from sensor_msgs.msg import Image
import csv		#writing CV files.
import os 		#used to get directory for image topics
import sys 		#used for errors and exiting
import getopt 	#used to parse arguments

def usage():
    print " -----------------------------------  USAGE:  -------------------------------------------------"
    print " rosrun data_extraction extract_topic.py -b bag_file_name -o output_file_name -t topic_name"
    print ""
    print "  bag_file_name - path to ros .bag file containing ROS records"
    print "  output_file_name - full path to csv file to save records to. If an image topic is specified"
    print "                     then the images will be saved to the same directory as the output file "
    print "                     and named Image_0001.jpg, Image_0002.jpg, etc."
    print "  topic_name       - topic in bag file that contains ROS messages."
    print "  "
    print "  Currently supported message types are:"
    print "                   - sensor_msgs/Image "
    print "                   - sensor_msgs/Imu "
    print "                   - sensor_msgs/LaserScan "
    print "                   - sensor_msgs/NavSatFix "
    print "                   - gps_common/gpsVel  "
    print "                   - umrr_driver/radar_msg  "
    print "                   - ardrone_autonomy/Navdata  "
    print "                   - risc_msgs/Controls  "
    print "                   - risc_msgs/Landmarks  "
    print "                   - risc_msgs/Waypoints  "
    print "                   - risc_msgs/Cortex  "
    print "                   - sensor_msgs/Joy  "
    print "                   - risc_msgs/Angles  "
    print ""
    print "  Rosbag Extraction Script v1 - Shane Lynn - 3rd May 2012"
    print "                   - D. Spencer Maughan - June 2014"
    print "  Modified to include: Navdata,Controls, Waypoints and Cortex and Joy(for ps controller and one object only)"
    print "                   - D. Spencer Maughan - September 2014"
    print "  Modified to include: Tf,Observed_angles"
    print "                   - D. Spencer Maughan - October 2014"
    print "  Modified to include: Mocap_data"
    print "  Modified to include: templates for Dr. Maguire's Students, and Suspension Senior project"
    print "                   - D. Spencer Maughan - Novemember 2014"
    print "  Modified to include: Trajectories"
    print "                   - D. Spencer Maughan - February 2015"
    print "  Modified to include: roscopter/Status, geometry_msgs/TwistStamped, and std_msgs/Bool"
    print "                   - D. Spencer Maughan - May 2014"
    print ""
    print ""
    print " ---------------------------------------------------------------------------------------------"


def getHeader(msg,topicName):
    #this function makes up the top of the csv file
    msgType = str(type(msg))
    msgType = msgType[msgType.index('.')+1:]
    msgType = msgType[:msgType.index('\'')]
    
    rospy.loginfo("Messages are of type %s"%msgType)
    rospy.loginfo("Processing records:")
    isImage = False

    #Add handlers as necessary for each message type:

    if (msgType == '_sensor_msgs__LaserScan'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
                "angle_min", "angle_max", "angle_increment","time_increment", "scan_time", "range_min", "range_max" ]
        #set up the header row:
        for i in range(len(msg.ranges)):
            headerRow.append("Range%s"%(i+1))
        if len(msg.intensities) >= 1:
            for i in range(len(msg.intensities)):
                headerRow.append("Intensity%s"%(i+1))

    elif (msgType == '_sensor_msgs__Imu'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
                "Orientation_x", "Orientation_y", "Orientation_z", "Orientation_w", \
                "AngularVelocity_x", "AngularVelocity_y", "AngularVelocity_z", \
                "LinearAcceleration_x", "LinearAcceleration_y", "LinearAcceleration_z"]

    elif (msgType == '_sensor_msgs__NavSatFix'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
                "Latitude", "Longitude", "Altitude", "PositionCovariance"]
        
    elif (msgType == '_std_msgs__Bool'):
        headerRow = ["Time", "Boolean"]

    elif (msgType == '_roscopter__Status'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
                "Battery Voltage", "Battery Current", "Battery Remaining", "Sensors Enabled"]

    elif (msgType == '_geometry_msgs__TwistStamped'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
                "X", "Y", "Z", "CovX", "CovY", "CovZ"]

    elif (msgType == '_gps_common__GPSFix'):
        headerRow =["Time", "Header_sequence", "Header_secs", "Header_nsecs", "GPS_status", \
                "Latitude", "Longitude", "Altitude", "PositionCovariance", "PositionCovariance_type", \
                "Track", "Speed", "Vert_Speed", "GPS_time", "Pitch", "Roll", "Dip" \
                ]
    elif (msgType == '_sensor_msgs__Image'):
        headerRow = ["Time", "Header_ sequence", "Header_secs", "Header_nsecs", "Filename"]
        isImage = True

    elif (msgType == '_umrr_driver__radar_msg'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
                "SensorID", "NumTargets", "Mode", "Submode", "Status", \
                "Target_Range", "Target_Angle", "Target_RadialSpeed", "Target_Signal2Threshold", "Target_Type"]

    elif (msgType == '_risc_msgs__Controls'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
                "Name", "Phi", "Theta", "Psidot", "Thrust"]

    elif (msgType == '_risc_msgs__Waypoints'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
                "Name", "X", "Y", "Z", "heading"]
    elif (msgType == '_risc_msgs__Landmarks'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
                "Landmark1", "X", "Y", "Z", "heading",
                "Landmark2", "X", "Y", "Z", "heading",
                "Landmark3", "X", "Y", "Z", "heading",
                "Landmark4", "X", "Y", "Z", "heading"]
    elif (msgType == '_risc_msgs__Trajectories'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
                "Traj1", "x", "y", "z", "psi", "xdot", "ydot", "zdot", "psidot", "xddot", "yddot", "zddot", "psiddot"]
    elif (msgType == '_risc_msgs__Cortex'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
                "Quad1","visible", "x", "y", "z", "u", "v", "w", "phi", "theta", "psi", "p", "q", "r"]
# for one object with three markers
#    elif (msgType == '_risc_msgs__Mocap_data'):
#        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
#                "Object1","residual", "x1", "y1", "z1", "x2", "y2", "z2", "x3", "y3", "z3"]
    elif (msgType == '_risc_msgs__Mocap_data'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
                "Object1","residual", "x1", "y1", "z1", "x2", "y2", "z2", "x3", "y3", "z3", "x4", "y4", "z4", "x5", "y5", "z5", "x6", "y6", "z6", \
                "Object2","residual", "x1", "y1", "z1", "x2", "y2", "z2", "x3", "y3", "z3", "x4", "y4", "z4"]
    elif (msgType == '_risc_msgs__Observed_angles'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
                "quad1","Lm1", "visible", "azim", "elev", "Lm2", "visible", "azim", "elev","Lm3", "visible", "azim", "elev","Lm4", "visible", "azim", "elev","Q2", "visible", "azim", "elev", "quad2","Lm1", "visible", "azim", "elev", "Lm2", "visible", "azim", "elev","Lm3", "visible", "azim", "elev","Lm4", "visible", "azim", "elev","Q1", "visible", "azim", "elev"]

    elif (msgType == '_tf2_msgs__TFMessage'):
        headerRow = ["tf1Time", "tf1Header_sequence", "tf1Header_secs", "tf1Header_nsecs", \
                "tf1parent_frame", "tf1child_frame", "tf1TranslationX", "tf1TranslationY","tf1TranslationZ","tf1Rotationx","tf1Rotationy","tf1Rotationz","tf1Rotationw","tf2Time", "tf2Header_sequence", "tf2Header_secs", "tf2Header_nsecs", \
                "tf2parent_frame", "tf2child_frame", "tf2TranslationX", "tf2TranslationY","tf2TranslationZ","tf2Rotationx","tf2Rotationy","tf2Rotationz","tf2Rotationw","tf3Time", "tf3Header_sequence", "tf3Header_secs", "tf3Header_nsecs", \
                "tf3parent_frame", "tf3child_frame", "tf3TranslationX", "tf3TranslationY","tf3TranslationZ","tf3Rotationx","tf3Rotationy","tf3Rotationz","tf3Rotationw","tf4Time", "tf4Header_sequence", "tf4Header_secs", "tf4Header_nsecs", \
                "tf4parent_frame", "tf4child_frame", "tf4TranslationX", "tf4TranslationY","tf4TranslationZ","tf4Rotationx","tf4Rotationy","tf4Rotationz","tf4Rotationw"]
    elif (msgType == '_sensor_msgs__Joy'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs", \
                "Roll", "Pitch", "A3", "Yaw", "A5", "A6","A7","A8", \
                "B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8", "B9", "Up", "Down"]

    elif (msgType == '_ardrone_autonomy__Navdata'):
        headerRow = ["Time", "Header_sequence", "Header_secs", "Header_nsecs" \
                "batteryPercent",
"state",
"magX",
"magY",
"magZ",
"pressure",
"temp",
"wind_speed",
"wind_angle",
"wind_comp_angle",
"rotX",
"rotY",
"rotZ",
"altd",
"vx",
"vy",
"vz",
"ax",
"ay",
"az",
"motor1",
"motor2",
"motor3",
"motor4",
"tags_count",
"tags_type",
"tags_xc",
"tags_yc",
"tags_width",
"tags_height",
"tags_orientation",
"tags_distance",
"tm"
]
    elif (msgType == '_geometry_msgs__Point32'):
        headerRow = ["Time", "xt", "yt", "vt"]
    elif (msgType == '_three_pi_control__motor_command'):
        headerRow = ["Time", "left", "right"]
    elif (msgType == '_three_pi_control__data_log'):
        headerRow = ["Time", "Distance_Error", "Velocity_Error","Path_Error","Distance", "Velocity"]
    else:
        rospy.logerr("Unsupport Message type %s"%msgType)
        usage()
        sys.exit(2)

    return headerRow, isImage


def getColumns(t, objnum, msg, imageFile = ""):
    t = t.to_sec()
    #this function gets the data that is necessary for the csv file - one row at a time from the current msg.
    msgType = str(type(msg))
    msgType = msgType[msgType.index('.')+1:]
    msgType = msgType[:msgType.index('\'')]

    if (msgType == '_sensor_msgs__LaserScan'):
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, \
                msg.angle_min, msg.angle_max, msg.angle_increment, msg.time_increment, msg.scan_time, msg.range_min, msg.range_max ]
        for i in range(len(msg.ranges)):
            columns.append(msg.ranges[i])

        if len(msg.intensities) >= 1:
            for i in range(len(msg.intensities)):
                columns.append(msg.intensities[i])

    elif (msgType == '_sensor_msgs__Imu'):
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, \
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w, \
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, \
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

    elif (msgType == '_sensor_msgs__Joy'):
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, \
                msg.axes[0], msg.axes[1], msg.axes[2], msg.axes[3], msg.axes[4], msg.axes[5],msg.axes[6],msg.axes[7], \
                msg.buttons[0], msg.buttons[1], msg.buttons[2], msg.buttons[3], msg.buttons[4],\
                msg.buttons[5], msg.buttons[6], msg.buttons[7], msg.buttons[8], msg.buttons[9],\
                msg.buttons[10]]

    elif (msgType == '_sensor_msgs__NavSatFix'):
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, \
                msg.latitude, msg.longitude, msg.altitude, msg.position_covariance]

    elif (msgType == '_gps_common__GPSFix'):
        columns =[t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, msg.status.status, \
                msg.latitude, msg.longitude, msg.altitude, msg.position_covariance, msg.position_covariance_type, \
                msg.track, msg.speed, msg.climb, msg.time, msg.pitch, msg.roll, msg.dip ]
    elif (msgType == '_sensor_msgs__Image'):
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, imageFile]

    elif (msgType == '_umrr_driver__radar_msg'):
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, \
                msg.sensorID, msg.numTargets, msg.mode, msg.submode, msg.status, \
                msg.targetRange, msg.targetAngle, msg.targetRadialSpeed, msg.targetSig2Threhold, msg.targetType ]
    elif (msgType == '_risc_msgs__Observed_angles'):
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, \
                msg.Obj[0].name, msg.Obj[0].landmarks[0].name, msg.Obj[0].landmarks[0].visible,msg.Obj[0].landmarks[0].azim,msg.Obj[0].landmarks[0].elev,msg.Obj[0].landmarks[1].name, msg.Obj[0].landmarks[1].visible,msg.Obj[0].landmarks[1].azim,msg.Obj[0].landmarks[1].elev,msg.Obj[0].landmarks[2].name, msg.Obj[0].landmarks[2].visible,msg.Obj[0].landmarks[2].azim,msg.Obj[0].landmarks[2].elev,msg.Obj[0].landmarks[3].name, msg.Obj[0].landmarks[3].visible,msg.Obj[0].landmarks[3].azim,msg.Obj[0].landmarks[3].elev]

    elif (msgType == '_tf2_msgs__TFMessage'):
        columns = [t, msg.transforms[0].header.seq, msg.transforms[0].header.stamp.secs, msg.transforms[0].header.stamp.nsecs, \
                msg.transforms[0].header.frame_id, msg.transforms[0].child_frame_id, msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z, msg.transforms[0].transform.rotation.x,msg.transforms[0].transform.rotation.y,msg.transforms[0].transform.rotation.z,msg.transforms[0].transform.rotation.w]

    elif (msgType == '_ardrone_autonomy__Navdata'):
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs, msg.batteryPercent, msg.state, msg.magX, msg.magY, msg.magZ, msg.pressure, msg.temp, msg.wind_speed, msg.wind_angle, msg.wind_comp_angle, msg.rotX, msg.rotY, msg.rotZ, msg.altd, msg.vx, msg.vy, msg.vz, msg.ax, msg.ay, msg.az, msg.motor1, msg.motor2, msg.motor3, msg.motor4, msg.tags_count, msg.tags_type, msg.tags_xc, msg.tags_yc, msg.tags_width, msg.tags_height, msg.tags_orientation, msg.tags_distance, msg.tm]
 
    elif (msgType == '_std_msgs__Bool'):
        rospy.loginfo("recognizes Boolean")
        columns = [t, msg.data]

    elif (msgType == '_roscopter__Status'):
        rospy.loginfo("recognizes roscopter Status")
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs,  msg.battery_voltage, msg.battery_current,  msg.battery_remaining, msg.sensors_enabled]

    elif (msgType == '_geometry_msgs__TwistStamped'):
        rospy.loginfo("recognizes TwistStamped")
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs,  msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z]

    elif (msgType == '_risc_msgs__Controls'):
        rospy.loginfo("recognizes Controls")
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs,  msg.Obj[0].name, msg.Obj[0].phi, msg.Obj[0].theta,msg.Obj[0].psi,msg.Obj[0].T]

    elif (msgType == '_risc_msgs__Trajectories'):
        rospy.loginfo("recognizes trajectories")
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs,  msg.Obj[0].name, msg.Obj[0].x,  msg.Obj[0].y, msg.Obj[0].z, msg.Obj[0].psi, msg.Obj[0].xdot, msg.Obj[0].ydot, msg.Obj[0].zdot, msg.Obj[0].psidot,msg.Obj[0].xddot,msg.Obj[0].yddot, msg.Obj[0].zddot, msg.Obj[0].psiddot]
## two quads
    elif (msgType == '_risc_msgs__Cortex'):
        num = int(float(objnum)) - 1
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs,  msg.Obj[num].name, msg.Obj[num].visible,msg.Obj[num].x,  msg.Obj[num].y, msg.Obj[num].z, msg.Obj[num].u, msg.Obj[num].v, msg.Obj[num].w, msg.Obj[num].phi, msg.Obj[num].theta,msg.Obj[num].psi,msg.Obj[num].p, msg.Obj[num].q, msg.Obj[num].r]

# for one object three markers
#    elif (msgType == '_risc_msgs__Mocap_data'):
#        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs,  msg.Obj[0].name, msg.Obj[0].residual,msg.Obj[0].marker[0].x, msg.Obj[0].marker[0].y, msg.Obj[0].marker[0].z,msg.Obj[0].marker[1].x,msg.Obj[0].marker[1].y,msg.Obj[0].marker[1].z,msg.Obj[0].marker[2].x,msg.Obj[0].marker[2].y,msg.Obj[0].marker[2].z]
# for two objects first with 6 markers and the second with 4
    elif (msgType == '_risc_msgs__Mocap_data'):
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs,  msg.Obj[0].name, msg.Obj[0].residual,msg.Obj[0].marker[0].x, msg.Obj[0].marker[0].y, msg.Obj[0].marker[0].z,msg.Obj[0].marker[1].x,msg.Obj[0].marker[1].y,msg.Obj[0].marker[1].z,msg.Obj[0].marker[2].x,msg.Obj[0].marker[2].y,msg.Obj[0].marker[2].z,msg.Obj[0].marker[3].x,msg.Obj[0].marker[3].y,msg.Obj[0].marker[3].z,msg.Obj[0].marker[4].x,msg.Obj[0].marker[4].y,msg.Obj[0].marker[4].z,msg.Obj[0].marker[5].x,msg.Obj[0].marker[5].y,msg.Obj[0].marker[5].z,msg.Obj[1].name, msg.Obj[1].residual,msg.Obj[1].marker[0].x, msg.Obj[1].marker[0].y, msg.Obj[1].marker[0].z,msg.Obj[1].marker[1].x,msg.Obj[1].marker[1].y,msg.Obj[1].marker[1].z,msg.Obj[1].marker[2].x,msg.Obj[1].marker[2].y,msg.Obj[1].marker[2].z,msg.Obj[1].marker[3].x,msg.Obj[1].marker[3].y,msg.Obj[1].marker[3].z]

    elif (msgType == '_risc_msgs__Waypoints'):
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs,  msg.Obj[0].name, msg.Obj[0].x,  msg.Obj[0].y, msg.Obj[0].z,msg.Obj[0].heading]

    elif (msgType == '_risc_msgs__Landmarks'):
        columns = [t, msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs,  msg.Obj[0].name, msg.Obj[0].x,  msg.Obj[0].y, msg.Obj[0].z,msg.Obj[1].name, msg.Obj[1].x,  msg.Obj[1].y, msg.Obj[1].z,msg.Obj[2].name, msg.Obj[2].x,  msg.Obj[2].y, msg.Obj[2].z,msg.Obj[3].name, msg.Obj[3].x,  msg.Obj[3].y, msg.Obj[3].z]

    elif (msgType == '_geometry_msgs__Point32'):
        columns = [t, msg.x, msg.y, msg.z]

    elif (msgType == '_three_pi_control__motor_command'):
        columns = [t, msg.left, msg.right]
    elif (msgType == '_three_pi_control__data_log'):
        num = int(float(objnum)) - 1
        columns = [t, msg.distance_error[num],  msg.velocity_error[num], msg.path_error[num], msg.distance[num],  msg.velocity[num],]
    else:
        rospy.logerror("Unexpected error - AGH!")
        usage()
        sys.exit(2)

    return columns


def main():
    
    rospy.loginfo("Processing input arguments:")
    try:
        opts, extraparams = getopt.getopt(sys.argv[1:], "o:b:t:n:") #start at the second argument.
    except getopt.GetoptError, err:
        #print error info and exit
        print str(err)
        usage()
        sys.exit(2)

    #default values
    outFile = "output.csv"
    rosbagFile = "bagfile.bag"
    topicName = "/crossbow_imu/data"
    objnum = "1"
    for o,a in opts:
        if o == "-o":
            outFile = a
        elif o == "-b":
            rosbagFile = a
        elif o == "-t":
            topicName = a
        elif o == "-n":
            objnum = a
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
    rospy.loginfo ("Bag file opened.")
    rospy.loginfo("Opening " + outFile + " for writing..")
    try:
        fileH = open(outFile, 'wt')
        fileWriter = csv.writer(fileH)
    except:
        rospy.logerr("Error opening specified output file : %s"%outFile)
        usage()
        sys.exit(2)

    rospy.loginfo ("Output file opened.")
    #get the directory if we need it
    outDir = os.path.dirname(outFile)
    rospy.loginfo("Getting topic " + topicName + " from bag file.")
    count = 1
    isImage = False
    imageFile = ""
    #cvBridge set up in case we have images to deal with
    bridge = CvBridge()

    #fileWriter.writerow(["Time", "Header sequence", "Header secs", "Header nsecs", \
            #"angle_min", "angle_max", "angle_increment","time_increment", "scan_time", "range_min", "range_max",\
            #						 "ranges", "intensities" ])
    for topic, msg, t in bag.read_messages(topics=topicName):
        #on the first message, we need to set the header...
        if count == 1:
            #use this type to process the headers:
            headerRow, isImage = getHeader(msg,topicName)
            if headerRow != None:
                fileWriter.writerow(headerRow)

        #if we are dealing with image data - we also need to write the image file.
        if isImage:
            try:
                cvImage = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError, err:
                print err
	    timestr = str(msg.header.stamp.to_sec())
            imageFile = 'Image_%.4d'%count
            saveFileName = str(outDir)+"/"+imageFile+"-"+timestr+".jpg"
            cv.SaveImage(saveFileName, cv.fromarray(cvImage))

        #get the columns for the csv file.
        columns = getColumns(t, objnum, msg, imageFile)
        #write the columns or image to the file/folder.
        fileWriter.writerow(columns)

        #keep track of the number of records processed
        count = count + 1
        if (count % 100 == 0):
            rospy.loginfo("Processed %s records..."%count)


    #Summarise for user
    rospy.loginfo("GRAND TOTAL of %s records."%count)
    if count < 5:
        rospy.logwarn("WARNING: Very few records processed - is your topic name correctly specified?")
    rospy.loginfo("Closing files and cleaning up.")
    fileH.close()
    bag.close()
    rospy.loginfo("Done!")

main()
