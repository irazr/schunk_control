#include <PowerCubeChain.h>


PowerCubeChainNode::PowerCubeChainNode()
{
	// Create a node handle with a private namespace.
	//m_nh = ros::NodeHandle("powercube_chain");

	// Create the parameter and the control objects.
	m_pcParams = new PowerCubeCtrlParams();
	m_pcCtrl = new PowerCubeCtrl(m_pcParams);

	// Implementation of topics to publish.
    m_topicPub_JointState = m_nh.advertise<sensor_msgs::JointState> ("/joint_states", 1);
	m_topicPub_ControllerState = m_nh.advertise<control_msgs::JointTrajectoryControllerState> ("state", 1);
	m_topicPub_Diagnostic = m_nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);

	// Implementation of topics to subscribe.
	m_topicSub_CommandPos = m_nh.subscribe("command_pos", 1, 
										   &PowerCubeChainNode::topicCallback_CommandPos, this);
    m_topicSub_CommandVel = m_nh.subscribe("/command_vel", 1,
										   &PowerCubeChainNode::topicCallback_CommandVel, this);

	// Implementation of service servers.
	m_srvServer_Init = m_nh.advertiseService("init", &PowerCubeChainNode::srvCallback_Init, this);
	m_srvServer_Stop = m_nh.advertiseService("stop", &PowerCubeChainNode::srvCallback_Stop, this);
	m_srvServer_Recover = m_nh.advertiseService("recover", &PowerCubeChainNode::srvCallback_Recover, this);

	// Set default values.
	m_bInitialized = false;
	m_bStopped = true;
	m_bError = false;
	m_rtLastPublishTime = ros::Time::now();

	m_FJTA_Srv.reset(new FJTAS(m_nh, "follow_joint_trajectory",
									  boost::bind(&PowerCubeChainNode::processTrajectory, this, _1),
									  false));
	m_FJTA_Srv->start();
}


PowerCubeChainNode::~PowerCubeChainNode()
{
	// Terminate the controller.
	bool closed = m_pcCtrl->Close();
	if (closed)
		ROS_INFO("PowerCube Device closed!");
}


void PowerCubeChainNode::processTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
	// helper variables
	std::string sErrorMsg;
	int iNumPoints, iDOF;
	std::vector<std::string> vsJointNames;
	std::vector<double> vdPositions, vdVelocities, vdAccelerations;
	std::vector<double> vdTargetPositions, vdTargetVelocities, vdTargetAccelerations;
	control_msgs::FollowJointTrajectoryFeedback FJTA_Fbk;
	control_msgs::FollowJointTrajectoryResult FJTA_Res;
	double dPos, dVel, dAcc;
	bool bSuccess;

	// Publish info to the console for the user.
	ROS_INFO("Process Trajectory.");

	// Get some parameters.
	iDOF = m_pcParams->GetDOF();
	iNumPoints = goal->trajectory.points.size();
	vsJointNames = m_pcParams->GetJointNames();

	// Check the amount of points.
	if (iNumPoints == 0)
	{
		FJTA_Res.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
		sErrorMsg = "First point of trajectory has no positions";
		ROS_ERROR("%s", sErrorMsg.c_str());
		m_FJTA_Srv->setAborted(FJTA_Res, sErrorMsg);

		return;
	}

	// Check the dimensions.
	if (goal->trajectory.joint_names.size() != iDOF)
	{
		std::ostringstream ssErrorMsg;
		ssErrorMsg << "Skipping command: Commanded positions " << goal->trajectory.joint_names.size()
				   << " and DOF " << iDOF << " are not same dimension." << std::endl;
		FJTA_Res.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
		sErrorMsg = ssErrorMsg.str();
		ROS_ERROR("%s", sErrorMsg.c_str());
		m_FJTA_Srv->setAborted(FJTA_Res, sErrorMsg);

		return;
	}

	// Check the joint names.
	for (int i = 0; i < iDOF; i++)
	{
		if (goal->trajectory.joint_names[i] != vsJointNames[i])
		{
			std::ostringstream ssErrorMsg;
			ssErrorMsg << "Skipping command: Received joint name " << goal->trajectory.joint_names[i].c_str()
					   << " doesn't match expected joint name " << vsJointNames[i].c_str()
					   << " for joint " << i << std::endl;
			FJTA_Res.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
			sErrorMsg = ssErrorMsg.str();
			ROS_ERROR("%s", sErrorMsg.c_str());
			m_FJTA_Srv->setAborted(FJTA_Res, sErrorMsg);

			return;
		}
	}

	// Get the target values.
	for (int i = 0; i < iDOF; i++)
	{
		dPos = goal->trajectory.points[iNumPoints-1].positions[i];
		dVel = goal->trajectory.points[iNumPoints-1].velocities[i];
		dAcc = goal->trajectory.points[iNumPoints-1].accelerations[i];

		vdTargetPositions.push_back(dPos);
		vdTargetVelocities.push_back(dVel);
		vdTargetAccelerations.push_back(dAcc);
	}

	// Start executing the action.
	for (int point = 0; point < iNumPoints; point++)
	{
		// Check that preempt has not been requested by the client.
		if (m_FJTA_Srv->isPreemptRequested() || !ros::ok())
		{
			sErrorMsg = "Process Trajectory: Preempted";
			ROS_ERROR("%s", sErrorMsg.c_str());
			// set the action state to preempted
			m_FJTA_Srv->setPreempted();

			return;
		}

		// Clear the vectors with the desired values.
		vdPositions.clear();
		vdVelocities.clear();
		vdAccelerations.clear();

		// Save the desired values.
		for (int i = 0; i < iDOF; i++)
		{
			dPos = goal->trajectory.points[point].positions[i];
			dVel = goal->trajectory.points[point].velocities[i];
			dAcc = goal->trajectory.points[point].accelerations[i];

			vdPositions.push_back(dPos);
			vdVelocities.push_back(dVel);
			vdAccelerations.push_back(dAcc);
		}

		// Move the arm with the desired values.
        bSuccess = m_pcCtrl->MovePos(vdPositions);
        bSuccess = m_pcCtrl->updateStates();
		bSuccess = true;

		// Check, if the arm could be moved.
		if (!bSuccess)
		{
			FJTA_Res.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
			sErrorMsg = "Error during movement.";
			ROS_ERROR("%s", sErrorMsg.c_str());
			m_FJTA_Srv->setAborted(FJTA_Res, sErrorMsg);

			return;
		}

		// Set some feedback information.
		FJTA_Fbk.header.stamp = ros::Time::now();

		FJTA_Fbk.joint_names.clear();
		FJTA_Fbk.actual.positions.clear();
		FJTA_Fbk.actual.velocities.clear();
		FJTA_Fbk.actual.accelerations.clear();
		FJTA_Fbk.desired.positions.clear();
		FJTA_Fbk.desired.velocities.clear();
		FJTA_Fbk.desired.accelerations.clear();
		FJTA_Fbk.error.positions.clear();
		FJTA_Fbk.error.velocities.clear();
		FJTA_Fbk.error.accelerations.clear();

		for (int j = 0; j < iDOF; j++)
		{
			FJTA_Fbk.joint_names.push_back(vsJointNames[j]);
			//FJTA_Fbk.actual.positions.push_back(m_pcCtrl->getPositions().at(j));
			//FJTA_Fbk.actual.velocities.push_back(m_pcCtrl->getVelocities().at(j));
			//FJTA_Fbk.actual.accelerations.push_back(m_pcCtrl->getAccelerations().at(j));
			FJTA_Fbk.actual.positions.push_back(vdPositions[j]);
			FJTA_Fbk.actual.velocities.push_back(vdVelocities[j]);
			FJTA_Fbk.actual.accelerations.push_back(vdAccelerations[j]);
			FJTA_Fbk.desired.positions.push_back(vdTargetPositions[j]);
			FJTA_Fbk.desired.velocities.push_back(vdTargetVelocities[j]);
			FJTA_Fbk.desired.accelerations.push_back(vdTargetAccelerations[j]);
			//FJTA_Fbk.error.positions.push_back(vdTargetPositions[j] - m_pcCtrl->getPositions().at(j));
			//FJTA_Fbk.error.velocities.push_back(vdTargetVelocities[j] - m_pcCtrl->getVelocities().at(j));
			//FJTA_Fbk.error.accelerations.push_back(vdTargetAccelerations[j] - m_pcCtrl->getAccelerations().at(j));
			FJTA_Fbk.error.positions.push_back(vdTargetPositions[j] - vdPositions[j]);
			FJTA_Fbk.error.velocities.push_back(vdTargetVelocities[j] - vdVelocities[j]);
			FJTA_Fbk.error.accelerations.push_back(vdTargetAccelerations[j] - vdAccelerations[j]);
		}

		// Publish the feedback.
		m_FJTA_Srv->publishFeedback(FJTA_Fbk);
	}

	// Set some result information.
	FJTA_Res.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;

	sErrorMsg = "Trajectory execution successfully completed";
	ROS_INFO("%s", sErrorMsg.c_str());

	// Set the action state to succeeded.
	m_FJTA_Srv->setSucceeded(FJTA_Res, sErrorMsg);
}


void PowerCubeChainNode::getROSParametersFromParamSrv()
{
	bool bSuccess;
	
	bSuccess = m_pcParams->getROSParametersFromParamSrv(m_nh);

	// Shutdown the node, if not all parameters are available.
	if (!bSuccess) m_nh.shutdown();
}


void PowerCubeChainNode::getRobotDescriptionFromParamSrv()
{
	bool bSuccess;
	
	bSuccess = m_pcParams->getRobotDescriptionFromParamSrv(m_nh);

	// Shutdown the node, if not all parameters are available.
	if (!bSuccess) m_nh.shutdown();
}
	

void PowerCubeChainNode::topicCallback_CommandPos(const brics_actuator::JointPositions::ConstPtr& msg)
{
	unsigned int uiDOF;
	std::vector<std::string> vsJointNames;
	std::vector<double> vdCmdPos;
	std::string sUnit;
	
	ROS_DEBUG("Received new position command");
	
	// Check, if the power cube is initialized.
	if (!m_bInitialized)
	{	
		ROS_WARN("Skipping command: powercubes not initialized");
		publishState(false);
		
		return; 
	}
	
	// Check, if the status of the controller is ok.
	if (m_pcCtrl->getPC_Status() != PowerCubeCtrl::PC_CTRL_OK)
	{
		publishState(false);
		return; 
	}

	// Get the degrees of freedom and the joint names from the parameter object.
    uiDOF = m_pcParams->GetDOF();
	vsJointNames = m_pcParams->GetJointNames();
	
	// Resize the position vector and set the unit of the values.
	vdCmdPos.resize(uiDOF);
    sUnit = "degree";

	// Check the dimension of the position vector.
	if (msg->positions.size() != uiDOF)
	{
		ROS_ERROR("Skipping command: Commanded positions and DOF are not same dimension.");
		return;
	}

	// Parse the positions.
	for (unsigned int i = 0; i < uiDOF; i++)
	{
		// Check the joint name.
		if (msg->positions[i].joint_uri != vsJointNames[i])
		{
			ROS_ERROR("Skipping command: Received joint name %s doesn't match expected joint name %s for joint %d.", 
					  msg->positions[i].joint_uri.c_str(), vsJointNames[i].c_str(), i);
			return;
		}

		// Check the unit of the position.
		if (msg->positions[i].unit != sUnit)
		{
			ROS_ERROR("Skipping command: Received unit %s doesn't match expected unit %s.",
					  msg->positions[i].unit.c_str(), sUnit.c_str());
			return;
		}

		// If all checks are successful, parse the position value for this joint.
		ROS_DEBUG("Parsing position %f for joint %s", msg->positions[i].value, vsJointNames[i].c_str());
        vdCmdPos[i] = msg->positions[i].value/180.0*3.14;
	}

    for (int i = 0; i < vdCmdPos.size(); i++ )
        ROS_INFO("Pos: %f ", vdCmdPos.at(i));

	// Send the positions to the power cubes.
    if (!m_pcCtrl->MovePos(vdCmdPos))
	{
		m_bError = true;
		m_sErrorMsg = m_pcCtrl->getErrorMessage();
		ROS_ERROR("Skipping command: %s",m_pcCtrl->getErrorMessage().c_str());
		return;
    }

	ROS_DEBUG("Executed position command");
	
	publishState();
}
	
	
void PowerCubeChainNode::topicCallback_CommandVel(const brics_actuator::JointVelocities::ConstPtr& msg)
{
	unsigned int uiDOF;
	std::vector<std::string> vsJointNames;
	std::vector<double> vdCmdVel;
	std::string sUnit;
	
	ROS_DEBUG("Received new velocity command");
	
	// Check, if the power cube is initialized.
	if (!m_bInitialized)
	{	
		ROS_WARN("Skipping command: powercubes not initialized");
		publishState(false);
		
		return; 
	}
	
	// Check, if the status of the controller is ok.
	if (m_pcCtrl->getPC_Status() != PowerCubeCtrl::PC_CTRL_OK)
	{
		publishState(false);
		return; 
	}

	// Get the degrees of freedom and the joint names from the parameter object.
	uiDOF = m_pcParams->GetDOF();
	vsJointNames = m_pcParams->GetJointNames();
	
	// Resize the velocity vector and set the unit of the values.
	vdCmdVel.resize(uiDOF);
	sUnit = "rad";

	// Check the dimension of the velocity vector.
	if (msg->velocities.size() != uiDOF)
	{
		ROS_ERROR("Skipping command: Commanded velocities and DOF are not same dimension.");
		return;
	}

	// Parse the velocities.
	for (unsigned int i = 0; i < uiDOF; i++)
	{
		// Check the joint name.
		if (msg->velocities[i].joint_uri != vsJointNames[i])
		{
			ROS_ERROR("Skipping command: Received joint name %s doesn't match expected joint name %s for joint %d.", 
					  msg->velocities[i].joint_uri.c_str(), vsJointNames[i].c_str(), i);
			return;
		}

		// Check the unit of the velocity.
		if (msg->velocities[i].unit != sUnit)
		{
			ROS_ERROR("Skipping command: Received unit %s doesn't match expected unit %s.",
					  msg->velocities[i].unit.c_str(), sUnit.c_str());
			return;
		}

		// If all checks are successful, parse the velocity value for this joint.
		ROS_DEBUG("Parsing velocity %f for joint %s", msg->velocities[i].value, vsJointNames[i].c_str());
		vdCmdVel[i] = msg->velocities[i].value;
	}
	
	// Send the velocities to the power cubes.
	if (!m_pcCtrl->MoveVel(vdCmdVel))
	{
		m_bError = true;
		m_sErrorMsg = m_pcCtrl->getErrorMessage();
		ROS_ERROR("Skipping command: %s",m_pcCtrl->getErrorMessage().c_str());
		return;
	}

	ROS_DEBUG("Executed velocity command");
	
	publishState();
}
	
	
bool PowerCubeChainNode::srvCallback_Init(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
{
	if (!m_bInitialized)
	{
		ROS_INFO("Initializing powercubes...");

		// Initialize power cubes.
		if (m_pcCtrl->Init())
		{
			m_bInitialized = true;
			m_bError = false;
			m_sErrorMsg = "";
			res.success.data = true;
			ROS_INFO("...initializing powercubes successful");
		}
		else
		{
			m_bError = true;
			m_sErrorMsg = m_pcCtrl->getErrorMessage();
			res.success.data = false;
			res.error_message.data = m_pcCtrl->getErrorMessage();
			ROS_INFO("...initializing powercubes not successful. error: %s", res.error_message.data.c_str());
		}
	}
	else
	{
		res.success.data = true;
		res.error_message.data = "powercubes already initialized";
		ROS_WARN("...initializing powercubes not successful. error: %s",res.error_message.data.c_str());
	}

	return true;
}
	
	
bool PowerCubeChainNode::srvCallback_Stop(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
{
	ROS_INFO("Stopping powercubes...");

	// Stop power cubes.
	if (m_pcCtrl->Stop())
	{
		m_bStopped = true;
		m_bError = false;
		m_sErrorMsg = "";
		res.success.data = true;
		ROS_INFO("...stopping powercubes successful.");
	}
	else
	{
		m_bError = true;
		m_sErrorMsg = m_pcCtrl->getErrorMessage();
		res.success.data = false;
		res.error_message.data = m_pcCtrl->getErrorMessage();
		ROS_ERROR("...stopping powercubes not successful. error: %s", res.error_message.data.c_str());
	}
	
	return true;
}
	
	
bool PowerCubeChainNode::srvCallback_Recover(cob_srvs::Trigger::Request &req, cob_srvs::Trigger::Response &res)
{
	ROS_INFO("Recovering powercubes...");
	
	if (m_bInitialized)
	{
		// Stopping all arm movements.
		if (m_pcCtrl->Recover())
		{
			m_bError = false;
			m_sErrorMsg = "";
			res.success.data = true;
			ROS_INFO("...recovering powercubes successful.");
		}
		else
		{	
			m_bError = true;
			m_sErrorMsg = m_pcCtrl->getErrorMessage();
			res.success.data = false;
			res.error_message.data = m_pcCtrl->getErrorMessage();
			ROS_ERROR("...recovering powercubes not successful. error: %s", res.error_message.data.c_str());
		}
	}
	else
	{
		res.success.data = false;
		res.error_message.data = "powercubes not initialized";
		ROS_ERROR("...recovering powercubes not successful. error: %s",res.error_message.data.c_str());
	}

	return true;
}
	

void PowerCubeChainNode::publishState(bool update)
{
	// Publish only, if the power cube chain is
	// initialized.
	if (m_bInitialized)
	{
        //ROS_INFO("HELLO WORLD");
		ROS_DEBUG("publish state");

		// Update the state of the controller.
		if (update)
		{
			m_pcCtrl->updateStates();
		}

		// Create a joint state message.
		sensor_msgs::JointState joint_state_msg;
		joint_state_msg.header.stamp = ros::Time::now();
		joint_state_msg.name = m_pcParams->GetJointNames();
		joint_state_msg.position = m_pcCtrl->getPositions();
		joint_state_msg.velocity = m_pcCtrl->getVelocities();
		joint_state_msg.effort.resize(m_pcParams->GetDOF());

		// Create a joint trajectory controller state message.
		control_msgs::JointTrajectoryControllerState controller_state_msg;
		controller_state_msg.header.stamp = joint_state_msg.header.stamp;
		controller_state_msg.joint_names = m_pcParams->GetJointNames();
		controller_state_msg.actual.positions = m_pcCtrl->getPositions();
		controller_state_msg.actual.velocities = m_pcCtrl->getVelocities();
		controller_state_msg.actual.accelerations = m_pcCtrl->getAccelerations();

        for (int i = 0; i < m_pcCtrl->getPositions().size(); i++)
        {
            double dDegree = m_pcCtrl->getPositions().at(i)/3.14*180.0;
            controller_state_msg.actual.positions[i] = dDegree;
            //joint_state_msg.position[i] = dDegree;
        }

		// Publish on topics.
		m_topicPub_JointState.publish(joint_state_msg);
		m_topicPub_ControllerState.publish(controller_state_msg);

		// Save the last publish time.
		m_rtLastPublishTime = joint_state_msg.header.stamp;
	}

	// Check status of power cube chain.
	if (m_pcCtrl->getPC_Status() != PowerCubeCtrl::PC_CTRL_OK) 
	{ 	
		m_bError = true;
	} 
	else 
	{
		m_bError = false;
	} 

	// Create a diagnotic message.
	diagnostic_msgs::DiagnosticArray diagnostics;
	diagnostics.status.resize(1);

	// Set data to the diagnostic message.
	if(m_bError)
	{
		diagnostics.status[0].level = 2;
        diagnostics.status[0].name = m_nh.getNamespace();
		diagnostics.status[0].message =  m_pcCtrl->getErrorMessage();
	}
	else
	{
		if (m_bInitialized)
		{
			diagnostics.status[0].level = 0;
			diagnostics.status[0].name = m_nh.getNamespace();
			diagnostics.status[0].message = "powercubechain initialized and running";
		}
		else
		{
			diagnostics.status[0].level = 1;
			diagnostics.status[0].name = m_nh.getNamespace();
			diagnostics.status[0].message = "powercubechain not initialized";
		}
	}
	
	// Publish diagnostic message.
	m_topicPub_Diagnostic.publish(diagnostics);
}


/*!
 * \brief Main loop of ROS node.
 *
 * Running with a specific frequency defined by loop_rate.
 */
int main(int argc, char** argv)
{
	double frequency;
	ros::Duration min_publish_duration;
	
	/// Initialize ROS, specify name of node.
	ROS_INFO("Init node powercube_chain.");
	ros::init(argc, argv, "powercube_chain");

	/// Create PowerCubeChainNode.
	PowerCubeChainNode pc_node;
	
	/// Get parameters from parameter server.
	ROS_INFO("Get ros parameters.");
	pc_node.getROSParametersFromParamSrv();
	
	/// Get robot description parameters.
	ROS_INFO("Get robot description parameters.");
	pc_node.getRobotDescriptionFromParamSrv();

	/// Get the frequency of the main loop.
	frequency = pc_node.m_pcParams->GetFrequency();
	
	/// Get the min publish duration time.
	min_publish_duration = pc_node.m_pcParams->GetMinPublishDuration();
	
	/// Set the frequency of the main loop in Hz.
	ros::Rate loop_rate(frequency);
	
	/// Main loop.
	ROS_INFO("Start main loop of node powercube_chain.");
	while (pc_node.m_nh.ok())
	{
		if ((ros::Time::now() - pc_node.m_rtLastPublishTime) >= min_publish_duration)
		{
            pc_node.publishState();
		}

		/// Sleep and waiting for messages and callbacks.
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
