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
#include <RelaisBoardNode.h>



int RelaisBoardNode::init() 
{
	if (n.hasParam("ComPort"))
	{
		n.getParam("ComPort", sComPort);
		ROS_INFO("Loaded ComPort parameter from parameter server: %s",sComPort.c_str());
	}

	n.param("message_timeout", relayboard_timeout_, 2.0);
	n.param("requestRate", requestRate, 25.0);

	n.param("protocol_version", protocol_version_, 1);
    
	m_SerRelayBoard = new SerRelayBoard(protocol_version_, &n);
	m_SerRelayBoard->init();
	ROS_INFO("Opened Relayboard at ComPort = %s", sComPort.c_str());

	n.getParam("drive1/CANId", motorCanIdent[0]);
	n.getParam("drive2/CANId", motorCanIdent[1]);
	n.getParam("drive1/joint_name", joint_names[0]);
	n.getParam("drive2/joint_name", joint_names[1]);
	//topics, which get published if the module is available
	n.getParam("hasMotorRight",activeModule[DRIVE1]);
	n.getParam("hasMotorLeft",activeModule[DRIVE2]);
	if(activeModule[DRIVE1] == 1 || activeModule[DRIVE2] == 1)
	{
		topicPub_drives = n.advertise<neo_serrelayboard::DriveStates>("/drive_states",1);
		topicSub_drives = n.subscribe("/cmd_drives",1,&RelaisBoardNode::getNewDriveStates, this);
	}
	n.getParam("hasIOBoard", activeModule[IO_BOARD]);
	n.getParam("hasLCDOut", hasLCDOut);
	if(hasLCDOut == 1) topicSub_lcdDisplay = n.subscribe("/srb_lcd_display",1,&RelaisBoardNode::getNewLCDOutput, this);

	if(activeModule[IO_BOARD] == 1)
	{
		topicSub_setDigOut = n.subscribe("/srb_io_set_dig_out",1,&RelaisBoardNode::getIOBoardDigOut, this);
		topicPub_ioDigIn = n.advertise<std_msgs::Int16>("/srb_io_dig_in",1);
		topicPub_ioDigOut = n.advertise<std_msgs::Int16>("/srb_io_dig_out",1);
		topicPub_analogIn = n.advertise<neo_serrelayboard::IOAnalogIn>("/srb_io_analog_in",1);

	}
	n.getParam("hasUSBoard", activeModule[US_BOARD]);
	if(activeModule[US_BOARD] == 1)
	{
		topicPub_usBoard = n.advertise<neo_serrelayboard::USBoard>("/srb_us_measurements",1);
		topicSub_startUSBoard = n.subscribe("/srb_start_us_board",1,&RelaisBoardNode::startUSBoard, this);
		topicSub_stopUSBoard = n.subscribe("/srb_stop_us_board",1,&RelaisBoardNode::stopUSBoard, this);

	}
	n.getParam("hasRadarBoard", activeModule[RADAR_BOARD]);
	if(activeModule[RADAR_BOARD] == 1) topicPub_radarBoard = n.advertise<neo_serrelayboard::RadarBoard>("/srb_radar_measurements",1);

	n.getParam("hasGyroBoard", activeModule[GYRO_BOARD]);
	if(activeModule[GYRO_BOARD] == 1)
	{
		topicPub_gyroBoard = n.advertise<neo_serrelayboard::GyroBoard>("/srb_gyro_measurements",1);
		topicSub_zeroGyro = n.subscribe("/srb_zero_gyro",1,&RelaisBoardNode::zeroGyro, this);
	}

	n.getParam("hasKeyPad", hasKeyPad);
	if(hasKeyPad == 1) topicPub_keypad = n.advertise<neo_serrelayboard::Keypad>("/srb_keypad",1);
	n.getParam("hasIRSensors", hasIRSensors);
	if(hasIRSensors == 1) topicPub_IRSensor = n.advertise<neo_serrelayboard::IRSensors>("/srb_ir_measurements",1);






	// Init member variable for EM State
	EM_stop_status_ = ST_EM_ACTIVE;
	duration_for_EM_free_ = ros::Duration(1);
	return 0;
}

int RelaisBoardNode::requestBoardStatus() {
	int ret;	
	
	// Request Status of RelayBoard 
	ret = m_SerRelayBoard->sendRequest();

	if(ret != SerRelayBoard::NO_ERROR) {
		ROS_ERROR("Error in sending message to Relayboard over SerialIO, lost bytes during writing");
	}

	ret = m_SerRelayBoard->evalRxBuffer();
	if(ret==SerRelayBoard::NOT_INITIALIZED) {
		ROS_ERROR("Failed to read relayboard data over Serial, the device is not initialized");
		relayboard_online = false;
	} else if(ret==SerRelayBoard::NO_MESSAGES) {
		ROS_ERROR("For a long time, no messages from RelayBoard have been received, check com port!");
		if(time_last_message_received_.toSec() - ros::Time::now().toSec() > relayboard_timeout_) {relayboard_online = false;}
	} else if(ret==SerRelayBoard::TOO_LESS_BYTES_IN_QUEUE) {
		//ROS_ERROR("Relayboard: Too less bytes in queue");
	} else if(ret==SerRelayBoard::CHECKSUM_ERROR) {
		ROS_ERROR("A checksum error occurred while reading from relayboard data");
	} else if(ret==SerRelayBoard::NO_ERROR) {
		relayboard_online = true;
		relayboard_available = true;
		time_last_message_received_ = ros::Time::now();
	}

	return 0;
}

double RelaisBoardNode::getRequestRate()
{
	return requestRate;
}

//////////////
// RelaisBoard

void RelaisBoardNode::sendEmergencyStopStates()
{

	if(!relayboard_available) return;
	
	
	bool EM_signal;
	ros::Duration duration_since_EM_confirmed;
	neo_serrelayboard::EmergencyStopState EM_msg;

	// assign input (laser, button) specific EM state
	EM_msg.emergency_button_stop = m_SerRelayBoard->isEMStop();
	EM_msg.scanner_stop = m_SerRelayBoard->isScannerStop();

	// determine current EMStopState
	EM_signal = (EM_msg.emergency_button_stop || EM_msg.scanner_stop);

	switch (EM_stop_status_)
	{
		case ST_EM_FREE:
		{
			if (EM_signal == true)
			{
				ROS_INFO("Emergency stop was issued");
				EM_stop_status_ = EM_msg.EMSTOP;
			}
			break;
		}
		case ST_EM_ACTIVE:
		{
			if (EM_signal == false)
			{
				ROS_INFO("Emergency stop was confirmed");
				EM_stop_status_ = EM_msg.EMCONFIRMED;
				time_of_EM_confirmed_ = ros::Time::now();
			}
			break;
		}
		case ST_EM_CONFIRMED:
		{
			if (EM_signal == true)
			{
				ROS_INFO("Emergency stop was issued");
				EM_stop_status_ = EM_msg.EMSTOP;
			}
			else
			{
				duration_since_EM_confirmed = ros::Time::now() - time_of_EM_confirmed_;
				if( duration_since_EM_confirmed.toSec() > duration_for_EM_free_.toSec() )
				{
					ROS_INFO("Emergency stop released");
					EM_stop_status_ = EM_msg.EMFREE;
				}
			}
			break;
		}
	};

	
	EM_msg.emergency_state = EM_stop_status_;

	//publish EM-Stop-Active-messages, when connection to relayboard got cut
	if(relayboard_online == false) {
		EM_msg.emergency_state = EM_msg.EMSTOP;
	}
	topicPub_isEmergencyStop.publish(EM_msg);
}


void RelaisBoardNode::sendAnalogIn()
{
	if(!relayboard_available) return;
	int analogIn[8];
	m_SerRelayBoard->getRelayBoardAnalogIn(analogIn);
	//temperatur
	neo_serrelayboard::Temperatur temp;
	temp.temperatur = analogIn[2];
	topicPub_temperatur.publish(temp);
	//battery
	neo_serrelayboard::BatState bat;
	bat.volts = analogIn[1];
	bat.chargingCurrent = analogIn[0];
	topicPub_batVoltage.publish(bat);
	//keypad
	if(hasKeyPad == 1)
	{
		neo_serrelayboard::Keypad pad;
		int mask = 1;
		for(int i = 0; i<4; i++)
		{
			if((analogIn[3] & mask) != 0)
			{
				pad.button[i] = true;
			} else {
				pad.button[i] = false;
			} 
			mask = mask << 1;
		}
		topicPub_keypad.publish(pad);
	}
	if(hasIRSensors == 1)
	{
		neo_serrelayboard::IRSensors irmsg;
		for(int i=0; i<4; i++) irmsg.measurement[i] = analogIn[4+i];
		topicPub_IRSensor.publish(irmsg);
	}
}

//////////////
// motorCtrl

void RelaisBoardNode::sendDriveStates()
{
	if(!relayboard_available) return;
	neo_serrelayboard::DriveStates state;
	for(int i = 0; i<2; i++)  state.joint_names[i] = joint_names[i];
	int temp;
	if(activeModule[DRIVE1] == 1 && activeModule[DRIVE2] == 1)
	{	
		m_SerRelayBoard->getWheelPosVel(motorCanIdent[0],&(state.angularPosition[0]), &(state.angularVelocity[0]));
		m_SerRelayBoard->getWheelPosVel(motorCanIdent[1],&(state.angularPosition[1]), &(state.angularVelocity[1]));
		m_SerRelayBoard->getStatus(motorCanIdent[0], &(state.motorState[0]), &temp);
		m_SerRelayBoard->getStatus(motorCanIdent[1], &(state.motorState[1]), &temp);
		topicPub_drives.publish(state);
	} 
	else if (activeModule[DRIVE1] == 1)
	{
		m_SerRelayBoard->getWheelPosVel(motorCanIdent[0],&(state.angularPosition[0]), &(state.angularVelocity[0]));
		m_SerRelayBoard->getStatus(motorCanIdent[0], &(state.motorState[0]), &temp);
		topicPub_drives.publish(state);
	} 
	else if (activeModule[DRIVE2] == 1)
	{
		m_SerRelayBoard->getWheelPosVel(motorCanIdent[1],&(state.angularPosition[1]), &(state.angularVelocity[1]));
		m_SerRelayBoard->getStatus(motorCanIdent[1], &(state.motorState[1]), &temp);
		topicPub_drives.publish(state);
	}
}

void RelaisBoardNode::getNewDriveStates(const neo_serrelayboard::DriveCommands& driveCommands)
{
	ROS_INFO("received drive command: %f   %f",driveCommands.angularVelocity[0],driveCommands.angularVelocity[1]);
	if(!relayboard_available) return;
	if(driveCommands.driveActive[0]){
		//TODO: test disableBrake:
		m_SerRelayBoard->disableBrake(motorCanIdent[0],driveCommands.disableBrake[0]);
		m_SerRelayBoard->setWheelVel(motorCanIdent[0], driveCommands.angularVelocity[0], driveCommands.quickStop[0]);
	}
	if(driveCommands.driveActive[1]){
		//TODO: test disableBrake:
		m_SerRelayBoard->disableBrake(motorCanIdent[1],driveCommands.disableBrake[1]);
		m_SerRelayBoard->setWheelVel(motorCanIdent[1], driveCommands.angularVelocity[1], driveCommands.quickStop[1]);
	}
}

//////////////
// GyroBoard

void RelaisBoardNode::sendGyroBoard()
{
	if(!relayboard_available || activeModule[GYRO_BOARD] != 1) return;
	neo_serrelayboard::GyroBoard gyro;
	m_SerRelayBoard->getGyroBoardAngBoost(&(gyro.orientation),gyro.acceleration);
	topicPub_gyroBoard.publish(gyro);
}

void RelaisBoardNode::zeroGyro(const std_msgs::Bool& b)
{
	if(!relayboard_available || activeModule[GYRO_BOARD] != 1) return;
	m_SerRelayBoard->zeroGyro(b.data);
}

//////////////
// radarBoard

void RelaisBoardNode::sendRadarBoard()
{
	if(!relayboard_available || activeModule[RADAR_BOARD] != 1) return;
	neo_serrelayboard::RadarBoard radar;
	double radarState[4];
	m_SerRelayBoard->getRadarBoardData(radarState);
	for(int i=0; i<3;i++)
	{
		radar.velocity[i] = radarState[i];
	}
	radar.state = radarState[3];
	topicPub_radarBoard.publish(radar);
}

//////////////
// usBoard

void RelaisBoardNode::sendUSBoard()
{
	if(!relayboard_available || activeModule[US_BOARD] != 1) return;
	int usSensors[8];
	int usAnalog[4];
	neo_serrelayboard::USBoard usBoard;
	m_SerRelayBoard->getUSBoardData1To8(usSensors);
	for(int i=0; i<8; i++) usBoard.sensor[i] = usSensors[i];
	m_SerRelayBoard->getUSBoardData9To16(usSensors);
	for(int i=0; i<8; i++) usBoard.sensor[i+8] = usSensors[i];
	m_SerRelayBoard->getUSBoardAnalogIn(usAnalog);
	for(int i=0; i<4; i++) usBoard.analog[i] = usAnalog[i];	
	topicPub_usBoard.publish(usBoard);
}

void RelaisBoardNode::startUSBoard(const std_msgs::Int16& configuration)
{
	if(!relayboard_available || activeModule[US_BOARD] != 1) return;
	m_SerRelayBoard->startUS(configuration.data);
}

void RelaisBoardNode::stopUSBoard(const std_msgs::Empty& empty)
{
	if(!relayboard_available || activeModule[US_BOARD] != 1) return;
	m_SerRelayBoard->stopUS();
}

//////////////
// ioBoard

void RelaisBoardNode::getNewLCDOutput(const neo_serrelayboard::LCDOutput& msg) 
{
	if(!relayboard_available || hasLCDOut != 1) return;
	m_SerRelayBoard->writeIOBoardLCD(0,0,msg.msg_line);

}

void RelaisBoardNode::getIOBoardDigOut(const neo_serrelayboard::IOOut& setOut)
{
	if(!relayboard_available || activeModule[IO_BOARD] != 1) return;
	m_SerRelayBoard->setIOBoardDigOut(setOut.channel, setOut.active);
}

void RelaisBoardNode::sendIOBoardDigIn()
{
	if(!relayboard_available || activeModule[IO_BOARD] != 1) return;
 	std_msgs::Int16 i;
	i.data = m_SerRelayBoard->getIOBoardDigIn();
	topicPub_ioDigIn.publish(i);

}

void RelaisBoardNode::sendIOBoardDigOut()
{
	if(!relayboard_available || activeModule[IO_BOARD] != 1) return;
 	std_msgs::Int16 i;
	i.data = m_SerRelayBoard->getIOBoardDigOut();
	topicPub_ioDigOut.publish(i);
}

void RelaisBoardNode::sendIOBoardAnalogIn()
{
	if(!relayboard_available || activeModule[IO_BOARD] != 1) return;
	int analogIn[8];
	neo_serrelayboard::IOAnalogIn in;
	m_SerRelayBoard->getIOBoardAnalogIn(analogIn);
	for(int i=0;i <8; i++) in.input[i] = analogIn[i];
	topicPub_analogIn.publish(in);
}