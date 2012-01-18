// ROS includes
#include <ros/ros.h>
#include <iostream>
#include <neo_SerRelayBoard/SerRelayBoard.h>

// ROS message includes
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <neo_serrelayboard/EmergencyStopState.h>
#include <neo_serrelayboard/BatState.h>
#include <neo_serrelayboard/Temperatur.h>
#include <neo_serrelayboard/DriveStates.h>
#include <neo_serrelayboard/DriveCommands.h>
#include <neo_serrelayboard/Keypad.h>
#include <neo_serrelayboard/LCDOutput.h>
#include <neo_serrelayboard/IRSensors.h>
#include <neo_serrelayboard/GyroBoard.h>
#include <neo_serrelayboard/RadarBoard.h>
#include <neo_serrelayboard/USBoard.h>
#include <neo_serrelayboard/IOOut.h>
#include <neo_serrelayboard/IOAnalogIn.h>

// ROS service includes
//--

// external includes
//--

//####################
//#### node class ####
class RelaisBoardNode
{
	//
	public:
		// create a handle for this node, initialize node
		ros::NodeHandle n;
                
		// topics:
		//basic topics:
		ros::Publisher topicPub_isEmergencyStop;
		ros::Publisher topicPub_batVoltage;
		ros::Publisher topicPub_temperatur;
		ros::Publisher topicPub_keypad;
		ros::Publisher topicPub_IRSensor;

		//optional topics:
		enum Modules {
			DRIVE1=0,
			DRIVE2=1,
			US_BOARD=2,
			RADAR_BOARD=3,
			IO_BOARD=4,
			GYRO_BOARD=5
		};
		//drives:
		ros::Publisher topicPub_drives;			//both motors are connected: motor callback
		ros::Subscriber topicSub_drives;		//set velocity of both drives
		//ultrasonic board:
		ros::Publisher topicPub_usBoard;
		ros::Subscriber topicSub_startUSBoard;
		ros::Subscriber topicSub_stopUSBoard;		
		//radar board:
		ros::Publisher topicPub_radarBoard;
		//io board:
		ros::Publisher topicPub_ioDigIn;
		ros::Publisher topicPub_ioDigOut;
		ros::Publisher topicPub_analogIn;
		ros::Subscriber topicSub_lcdDisplay;
		ros::Subscriber topicSub_setDigOut;
		//gyro board:
		ros::Publisher topicPub_gyroBoard;
		ros::Subscriber topicSub_zeroGyro;

		// Constructor
		RelaisBoardNode()
		{
			//topics which allways get published
			topicPub_isEmergencyStop = n.advertise<neo_serrelayboard::EmergencyStopState>("/srb_emergency_stop_state", 1);
			topicPub_batVoltage = n.advertise<neo_serrelayboard::BatState>("/srb_battery_state", 1);
			topicPub_temperatur = n.advertise<neo_serrelayboard::Temperatur>("/srb_temperatur", 1);

			// Make sure member variables have a defined state at the beginning
			EM_stop_status_ = ST_EM_FREE;
			relayboard_available = false;
			relayboard_online = false;
			relayboard_timeout_ = 2.0;
			protocol_version_ = 1;
			duration_for_EM_free_ = ros::Duration(1);
		}
        
		// Destructor
		~RelaisBoardNode() 
		{
			delete m_SerRelayBoard;
		}
    		//basic:
		void sendEmergencyStopStates();
		void sendAnalogIn();
		//IOBoard
		void getNewLCDOutput(const neo_serrelayboard::LCDOutput&); //output on a 20 char lcd display
		void sendIOBoardDigIn();
		void sendIOBoardDigOut();
		void getIOBoardDigOut(const neo_serrelayboard::IOOut&);
		void sendIOBoardAnalogIn();
		//motor:
		void sendDriveStates();
		void getNewDriveStates(const neo_serrelayboard::DriveCommands& driveCommands);
		//gyroBoard:
		void sendGyroBoard();
		void zeroGyro(const std_msgs::Bool& b);
		//radarBoard:
		void sendRadarBoard();
		//usBoard:
		void sendUSBoard();
		void startUSBoard(const std_msgs::Int16& configuration);
		void stopUSBoard(const std_msgs::Empty& empty);
		
		int init();
		int requestBoardStatus();
		double getRequestRate();

	private:
		int activeModule[6]; //are the modules available (else ther won't be any datastreaming);
		std::string sComPort;
		SerRelayBoard * m_SerRelayBoard;

		int EM_stop_status_;
		ros::Duration duration_for_EM_free_;
		ros::Time time_of_EM_confirmed_;
		double relayboard_timeout_;
		int protocol_version_;
		double requestRate;

		ros::Time time_last_message_received_;
		bool relayboard_online; //the relayboard is sending messages at regular time
		bool relayboard_available; //the relayboard has sent at least one message -> publish topic

		// possible states of emergency stop
		enum
		{
			ST_EM_FREE = 0,
			ST_EM_ACTIVE = 1,
			ST_EM_CONFIRMED = 2
		};
		int motorCanIdent[2];
		std::string joint_names[2];
		int hasKeyPad, hasIRSensors, hasLCDOut;

};
