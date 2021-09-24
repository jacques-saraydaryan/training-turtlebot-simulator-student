#!/bin/bash
#ptA="{'header':{'frame_id':'map'},'pose':{'position':{'x':1.978,'y':3.017},'orientation':{'z':-0.6978,'w':0.71628}}}"
#ptB="{'header':{'frame_id':'map'},'pose':{'position':{'x':1.978,'y':-1.3422},'orientation':{'z':0.70,'w':0.70}}}"


#ptA="{'header':{'frame_id':'map'},'pose':{'position':{'x':-0.1222,'y':-5.526},'orientation':{'z':0,'w':1}}}"
#ptB="{'header':{'frame_id':'map'},'pose':{'position':{'x':6.45,'y':-5.526},'orientation':{'z':1,'w':0}}}"

#ptA="{'header':{'frame_id':'map'},'pose':{'position':{'x':-14.666,'y':-5.168},'orientation':{'z':0,'w':1}}}"
#ptB="{'header':{'frame_id':'map'},'pose':{'position':{'x':-9.4343,'y':-5.526},'orientation':{'z':1,'w':0}}}"

#ptA="{'header':{'frame_id':'map'},'pose':{'position':{'x':-2.72,'y':-5.23},'orientation':{'z':1,'w':0}}}"
#ptB="{'header':{'frame_id':'map'},'pose':{'position':{'x':-6.5,'y':-5.526},'orientation':{'z':0,'w':1}}}"

ptA="{'header':{'frame_id':'map'},'pose':{'position':{'x':-15.513,'y':1.743},'orientation':{'z':0,'w':1}}}"
ptB="{'header':{'frame_id':'map'},'pose':{'position':{'x':-11.982,'y':1.743},'orientation':{'z':1,'w':0}}}"


sleepTime=20
isA=true
for (( c=1; c<=100; c++ ))
do  
   echo "Iteration $c "
   if $isA ;
	then
		echo "$ptA"
		rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped $ptA
		isA=false
	else
		echo "$ptB"
		rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped $ptB
		isA=true
   fi
	#rostopic pub /move_base_simple/goal geometry_msgs $ptA
   sleep $sleepTime
done
