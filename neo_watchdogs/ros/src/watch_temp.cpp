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


#include <ros/ros.h>
#include <neo_msgs/Temperatur.h>

class WatchTempNode
{
	public:
	WatchTempNode(){};
	virtual ~WatchTempNode(){};
	ros::NodeHandle n;
	ros::Subscriber subs_temp;
	
	int init();
	void handletemp(const neo_msgs::Temperatur& es);
	private:
	
};

int WatchTempNode::init()
{
	subs_temp = n.subscribe("/temperature", 1, &WatchTempNode::handletemp, this);
	return 0;
}

void WatchTempNode::handletemp(const neo_msgs::Temperatur& es)
{
	if(es.temperatur > 70)
	{
		ROS_ERROR("the temperatur is to high, halt the system, please");
	}
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "temperatur_watcher");
	WatchTempNode node;
	node.init();
	ros::spin();
}
