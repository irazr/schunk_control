#ifndef __POWER_CUBE_CHAIN_H_
#define __POWER_CUBE_CHAIN_H_


// ROS includes
#include <ros/ros.h>
#include <urdf/model.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

// own includes
#include <PowerCubeCtrl.h>
//#include <PowerCubeCtrlParams.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include <actionlib/goal_id_generator.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryResult.h>

/*!
 * \brief Implementation of ROS node for powercube_chain.
 *
 * Offers velocity and position interface.
 */
class PowerCubeChainNode
{
public:
    /// Constructor
	PowerCubeChainNode();

	/// Destructor
	~PowerCubeChainNode();
	
	/*!
	* \brief Gets parameters from the ROS parameter server and configures the powercube_chain.
	*/
	void getROSParametersFromParamSrv();
	
	/*!
	* \brief Gets parameters from the robot_description and configures the powercube_chain.
	*/
	void getRobotDescriptionFromParamSrv();
	
	/*!
	* \brief Executes the callback from the command_pos topic.
	*
	* Set the current position target.
	* \param msg JointPositions
	*/
	void topicCallback_CommandPos(const brics_actuator::JointPositions::ConstPtr& msg);
	
	/*!
	* \brief Executes the callback from the command_vel topic.
	*
	* Set the current velocity target.
	* \param msg JointVelocities
	*/
	void topicCallback_CommandVel(const brics_actuator::JointVelocities::ConstPtr& msg);
	
	/*!
	* \brief Executes the service callback for init.
	*
	* Connects to the hardware and initialized it.
	* \param req Service request
	* \param res Service response
	*/
	bool srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);
	
	/*!
	* \brief Executes the service callback for stop.
	*
	* Stops all hardware movements.
	* \param req Service request
	* \param res Service response
	*/
	bool srvCallback_Stop(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);
	
	/*!
	* \brief Executes the service callback for recover.
	*
	* Recovers the driver after an emergency stop.
	* \param req Service request
	* \param res Service response
	*/
	bool srvCallback_Recover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res);
	
	/*!
	* \brief Executes the service callback for SetOperationMode.
	*
	* Sets the driver to different operation modes. Currently only operation_mode=velocity is supported.
	* \param req Service request
	* \param res Service response
	*/
	bool srvCallback_SetOperationMode(cob_srvs::SetOperationMode::Request &req, cob_srvs::SetOperationMode::Response &res);
	
	/*!
	* \brief Publishes the state of the powercube_chain as ros messages.
	*
	* Published to "/joint_states" as "sensor_msgs/JointState"
	* Published to "state" as "control_msgs/JointTrajectoryControllerState"
	*/
	void publishState(bool update=true);

	void processTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);
	
public:
	/// create a handle for this node, initialize node
	ros::NodeHandle m_nh;

	/// declaration of topics to publish
	ros::Publisher m_topicPub_JointState;
	ros::Publisher m_topicPub_ControllerState;
	ros::Publisher m_topicPub_Diagnostic;

	/// declaration of topics to subscribe, callback is called for new messages arriving
	ros::Subscriber m_topicSub_CommandPos;
	ros::Subscriber m_topicSub_CommandVel;

	/// declaration of service servers
	ros::ServiceServer m_srvServer_Init;
	ros::ServiceServer m_srvServer_Stop;
	ros::ServiceServer m_srvServer_Recover;

	/// handle for powercube_chain
	PowerCubeCtrl* m_pcCtrl;

	/// handle for powercube_chain parameters
	PowerCubeCtrlParams* m_pcParams;

	/// member variables
	bool m_bInitialized;
	bool m_bStopped;
	bool m_bError;
	std::string m_sErrorMsg;
	ros::Time m_rtLastPublishTime;

	//actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> m_FJTA_Srv;

	typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> FJTAS;
	boost::scoped_ptr<FJTAS> m_FJTA_Srv;
}; //PowerCubeChainNode

#endif
