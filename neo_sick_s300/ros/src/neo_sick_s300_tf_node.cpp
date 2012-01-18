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
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

class TF_SICK_S300 
{
   public:
	TF_SICK_S300();
	~TF_SICK_S300();
	int init();
	ros::NodeHandle n;
	ros::Subscriber sub;
	tf::TransformBroadcaster br;
	void mesSubs(const sensor_msgs::LaserScan::ConstPtr& r);
   private:
	tf::Transform transform;
	std::string frame_name;
	std::vector<double> position, orientation;
	
};

int TF_SICK_S300::init()
{
	//get parameters from config file:
	XmlRpc::XmlRpcValue e;
	n.param<std::string>("frame_id",frame_name,"/base_laser_link");
	n.getParam("translation",e);
	for(int i=0; i<e.size(); i++)
	{
		position.push_back((double) e[i]);
	}
	n.getParam("orientation",e);
	for(int i=0; i<e.size(); i++)
	{
		orientation.push_back((double) e[i]);
	}
	//set up transformations:
	transform.setOrigin( tf::Vector3(position[0], position[1], position[2]) );
	transform.setRotation( tf::Quaternion(orientation[0], orientation[1], orientation[2]) );

	//setup broadcaster:
	sub = n.subscribe("/scan",1,&TF_SICK_S300::mesSubs, this);
	ROS_INFO("started neo_sick_s300 transformation broadcaster ");


	return 0;	
}

TF_SICK_S300::TF_SICK_S300()
{
}


TF_SICK_S300::~TF_SICK_S300()
{

}

void TF_SICK_S300::mesSubs(const sensor_msgs::LaserScan::ConstPtr& r)
{
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", frame_name));
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "TF_SICK_S300");
	TF_SICK_S300 tf;
	if(tf.init() != 0) return 1;
	ros::spin();	
	return 0;
};
