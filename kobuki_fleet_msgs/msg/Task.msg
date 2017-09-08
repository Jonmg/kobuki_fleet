Header header
string machineName
string robotName
string srcName
uint16 tid
uint16 rid1
uint16 rid2
geometry_msgs/Pose robotPose
geometry_msgs/Pose machinePose
geometry_msgs/Pose srcPose

uint8 OPEN=0
uint8 WORKING=1
uint8 FINISHED=2
uint8 ERROR=3
uint8 CANCELLED=4
uint8 taskStatus

uint8 typeA=1
uint8 typeB=2
uint8 materialType
