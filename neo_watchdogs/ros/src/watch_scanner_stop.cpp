/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** 
 *  move_base usually gets confused by scanner stops...
 *  ->reset move_base/goal if scanner stop occures.
*/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <neo_msgs/EmergencyStopState.h>

class ScanStopNode
{
	public:
	ScanStopNode(){};
	virtual ~ScanStopNode(){};
	ros::NodeHandle n;
	ros::Publisher pub_navGoal;
	ros::Subscriber subs_navGoal;
	ros::Subscriber subs_errorStop;
	
	int init();
	void handleNavGoal(const move_base_msgs::MoveBaseActionGoal& cg);
	void handleErrorStop(const neo_msgs::EmergencyStopState& es);
	private:
	move_base_msgs::MoveBaseActionGoal currentGoal;
	bool hasError, hasGoal;
	
};

int ScanStopNode::init()
{
	hasError = false;
	hasGoal = false;
	pub_navGoal = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
	subs_navGoal = n.subscribe("/move_base/goal", 1, &ScanStopNode::handleNavGoal, this);
	subs_errorStop = n.subscribe("/emergency_stop_state", 1, &ScanStopNode::handleErrorStop, this);
	return 0;
}

void ScanStopNode::handleNavGoal(const move_base_msgs::MoveBaseActionGoal& cg)
{
	currentGoal = cg;
	hasGoal = true;
}

void ScanStopNode::handleErrorStop(const neo_msgs::EmergencyStopState& es)
{
	if(es.scanner_stop)
	{
		if(hasGoal && !hasError)
		{
			hasError = true;
			move_base_msgs::MoveBaseActionGoal sendGoal;
			sendGoal = currentGoal;
			sendGoal.header.seq = currentGoal.header.seq+1;
			sendGoal.goal.target_pose.header.seq = currentGoal.goal.target_pose.header.seq+1;
			sendGoal.header.stamp = ros::Time::now();
			sendGoal.goal.target_pose.header.stamp = sendGoal.header.stamp;
			pub_navGoal.publish(sendGoal);
		}
	}
	else
	{
		hasError = false;
	}
	
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "scanner_stop_watcher");
	ScanStopNode node;
	node.init();
	ros::spin();
}

