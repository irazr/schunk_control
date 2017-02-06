#include "PowerCubeCtrlParams.h"


PowerCubeCtrlParams::PowerCubeCtrlParams()
{
	m_iDOF = 0;
	m_bUseMoveVel = true;
}


PowerCubeCtrlParams::~PowerCubeCtrlParams()
{
}


void PowerCubeCtrlParams::Init(std::string sCanModule, std::string sCanDevice, 
							  int iBaudrate, std::vector<int> viModuleIDs)
{
	SetCanModule(sCanModule);
	SetCanDevice(sCanDevice);
	SetBaudrate(iBaudrate);
	SetDOF(viModuleIDs.size());
	
	for (int i = 0; i < m_iDOF; i++)
	{
		m_viModulIDs.push_back(viModuleIDs[i]);
	}
}


void PowerCubeCtrlParams::SetDOF(int iDOF)
{
	m_iDOF = iDOF;
}


int PowerCubeCtrlParams::GetDOF()
{
	return m_iDOF;
}


void PowerCubeCtrlParams::SetUseMoveVel(bool bUseMoveVel)
{
	m_bUseMoveVel = bUseMoveVel;
}


bool PowerCubeCtrlParams::GetUseMoveVel()
{
	return m_bUseMoveVel;
}


void PowerCubeCtrlParams::SetCanModule(std::string sCanModule)
{
	m_sCanModule = sCanModule;
}


std::string PowerCubeCtrlParams::GetCanModule()
{
	return m_sCanModule;
}


void PowerCubeCtrlParams::SetCanDevice(std::string sCanDevice)
{
	m_sCanDevice = sCanDevice;
}


std::string PowerCubeCtrlParams::GetCanDevice()
{
	return m_sCanDevice;
}


void PowerCubeCtrlParams::SetBaudrate(int iBaudrate)
{
	m_iBaudrate = iBaudrate;
}


int PowerCubeCtrlParams::GetBaudrate()
{
	return m_iBaudrate;
}


std::vector<int> PowerCubeCtrlParams::GetModuleIDs()
{
	return m_viModulIDs;
}


int PowerCubeCtrlParams::GetModuleID(int iNo)
{
	if (iNo < GetDOF())
		return m_viModulIDs[iNo];
	
	return -1;
}


bool PowerCubeCtrlParams::SetModuleID(int iNo, int iID)
{
	if (iNo < GetDOF())
	{
		m_viModulIDs[iNo] = iID;
		return true;
	}
	
	return false;
}


std::vector<std::string> PowerCubeCtrlParams::GetJointNames()
{
	return m_vsJointNames;
}


bool PowerCubeCtrlParams::SetJointNames(std::vector<std::string> vsJointNames)
{
	if ((int)vsJointNames.size() == GetDOF())
	{
		m_vsJointNames = vsJointNames;
		return true;
	}
	
	return false;
}


bool PowerCubeCtrlParams::SetUpperLimits(std::vector<double> vdUpperLimits)
{
	if ((int)vdUpperLimits.size() == GetDOF())
	{
		m_vdUpperLimits = vdUpperLimits;
		return true;
	}
	
	return false;
}


bool PowerCubeCtrlParams::SetLowerLimits(std::vector<double> vdLowerLimits)
{
	if ((int)vdLowerLimits.size() == GetDOF())
	{
		m_vdLowerLimits = vdLowerLimits;
		return true;
	}
	
	return false;
}


bool PowerCubeCtrlParams::SetOffsets(std::vector<double> vdOffsets)
{
	if ((int)vdOffsets.size() == GetDOF())
	{
		m_vdOffsets = vdOffsets;
		return true;
	}
	
	return false;
}


bool PowerCubeCtrlParams::SetMaxVel(std::vector<double> vdMaxVel)
{
	if ((int)vdMaxVel.size() == GetDOF())
	{
		m_vdMaxVel = vdMaxVel;
		return true;
	}
	
	return false;
}


bool PowerCubeCtrlParams::SetMaxAcc(std::vector<double> vdMaxAcc)
{
	if ((int)vdMaxAcc.size() == GetDOF())
	{
		m_vdMaxAcc = vdMaxAcc;
		return true;
	}
	
	return false;
}


std::vector<double> PowerCubeCtrlParams::GetUpperLimits()
{
	return m_vdUpperLimits;
}


std::vector<double> PowerCubeCtrlParams::GetLowerLimits()
{
	return m_vdLowerLimits;
}


std::vector<double> PowerCubeCtrlParams::GetOffsets()
{
	return m_vdOffsets;
}


std::vector<double> PowerCubeCtrlParams::GetMaxAcc()
{
	return m_vdMaxAcc;
}


std::vector<double> PowerCubeCtrlParams::GetMaxVel()
{
	return m_vdMaxVel;
}


void PowerCubeCtrlParams::SetFrequency(double dFrequency)
{
	m_dFrequency = dFrequency;
}


double PowerCubeCtrlParams::GetFrequency()
{
	return m_dFrequency;
}


void PowerCubeCtrlParams::SetMinPublishDuration(ros::Duration rdMinPublishDuration)
{
	m_rdMinPublishDuration = rdMinPublishDuration;
}


ros::Duration PowerCubeCtrlParams::GetMinPublishDuration()
{
	return m_rdMinPublishDuration;
}


bool PowerCubeCtrlParams::getCanModuleFromParamSrv(std::string& sCanModule, ros::NodeHandle nh)
{
	if (nh.hasParam("can_module"))
	{
		nh.getParam("can_module", sCanModule);
		return true;
	}
	else
	{
		ROS_ERROR("Parameter can_module not set, shutting down node...");
		return false;
	}
}


bool PowerCubeCtrlParams::getCanDeviceFromParamSrv(std::string& sCanDevice, ros::NodeHandle nh)
{
	if (nh.hasParam("can_device"))
	{
		nh.getParam("can_device", sCanDevice);
		return true;
	}
	else
	{
		ROS_ERROR("Parameter can_device not set, shutting down node...");
		return false;
	}
}


bool PowerCubeCtrlParams::getCanBaudrateFromParamSrv(int& iCanBaudrate, ros::NodeHandle nh)
{
	if (nh.hasParam("can_baudrate"))
	{
		nh.getParam("can_baudrate", iCanBaudrate);
		return true;
	}
	else
	{
		ROS_ERROR("Parameter can_baudrate not set, shutting down node...");
		return false;
	}
}


bool PowerCubeCtrlParams::getModulIDsFromParamSrv(std::vector<int>& viModulIDs, ros::NodeHandle nh)
{
	XmlRpc::XmlRpcValue xmlRpcModulIDs;
	
	if (nh.hasParam("modul_ids"))
	{
		nh.getParam("modul_ids", xmlRpcModulIDs);
		
		// Resize and assign of values to the ModulIDs.
		viModulIDs.resize(xmlRpcModulIDs.size());
		for (int i = 0; i < xmlRpcModulIDs.size(); i++)
		{
			viModulIDs[i] = (int)xmlRpcModulIDs[i];
		}
		
		return true;
	}
	else
	{
		ROS_ERROR("Parameter modul_ids not set, shutting down node...");
		return false;
	}
}


bool PowerCubeCtrlParams::getUseMoveVelFromParamSrv(bool& bUseMoveVel, ros::NodeHandle nh)
{
	if (nh.hasParam("force_use_movevel"))
	{
		nh.getParam("force_use_movevel", bUseMoveVel);
		ROS_INFO("Parameter force_use_movevel set, using moveVel");
	}
	else
	{
		ROS_INFO("Parameter force_use_movevel not set, using moveStep");
		bUseMoveVel = false;
	}
	
	return true;
}


bool PowerCubeCtrlParams::getJointNamesFromParamSrv(std::vector<std::string>& vsJointNames, 
													ros::NodeHandle nh)
{
	XmlRpc::XmlRpcValue xmlRpcJointNames;
	
	if (nh.hasParam("joint_names"))
	{
		nh.getParam("joint_names", xmlRpcJointNames);
		
		// Resize and assign of values to the JointNames.
		vsJointNames.resize(xmlRpcJointNames.size());
		for (int i = 0; i < xmlRpcJointNames.size(); i++)
		{
			vsJointNames[i] = (std::string)xmlRpcJointNames[i];
		}
		
		return true;
	}
	else
	{
		ROS_ERROR("Parameter joint_names not set, shutting down node...");
		return false;
	}
}


bool PowerCubeCtrlParams::getMaxAccelerationsFromParamSrv(std::vector<double>& vdMaxAccelerations, 
														  ros::NodeHandle nh)
{
	XmlRpc::XmlRpcValue xmlRpcMaxAccelerations;

	if (nh.hasParam("max_accelerations"))
	{
		nh.getParam("max_accelerations", xmlRpcMaxAccelerations);
		
		// Resize and assign of values to the MaxAccelerations.
		vdMaxAccelerations.resize(xmlRpcMaxAccelerations.size());
		for (int i = 0; i < xmlRpcMaxAccelerations.size(); i++)
		{
			vdMaxAccelerations[i] = (double)xmlRpcMaxAccelerations[i];
		}
		
		return true;
	}
	else
	{
		ROS_ERROR("Parameter max_accelerations not set, shutting down node...");
		
		return false;
	}
}


bool PowerCubeCtrlParams::getFrequencyFromParamSrv(double& dFrequency, ros::NodeHandle nh)
{
	// Frequency of driver has to be much higher then controller frequency.
	if (nh.hasParam("frequency"))
	{
		nh.getParam("frequency", dFrequency);	
	}
	else
	{
		dFrequency = 10; // Hz
		ROS_WARN("Parameter frequency not available, setting to default value: %f Hz", dFrequency);
	}
	
	return true;
}


bool PowerCubeCtrlParams::getMinPublishDurationFromParamSrv(ros::Duration& rdMinPublishDuration, 
															ros::NodeHandle nh)
{
	double dSec;
	
	if (nh.hasParam("min_publish_duration"))
	{
		nh.getParam("min_publish_duration", dSec);
		rdMinPublishDuration.fromSec(dSec);
		
		return true;
	}
	else
	{
		ROS_ERROR("Parameter min_publish_time not available");
		
		return false;
	}
}


bool PowerCubeCtrlParams::getROSParametersFromParamSrv(ros::NodeHandle nh)
{
	int iCanBaudrate;
	double dFrequency;
	bool bUseMoveVel, bSuccess;
	std::string sCanModule, sCanDevice;
	std::vector<int> viModulIDs;
	std::vector<double> vdMaxAccelerations;
	std::vector<std::string> vsJointNames;
	ros::Duration rdMinPublishDuration;
	
	// Get the parameters from the parameter server.
	bSuccess  = getCanModuleFromParamSrv(sCanModule, nh);
	bSuccess &= getCanDeviceFromParamSrv(sCanDevice, nh);
	bSuccess &= getCanBaudrateFromParamSrv(iCanBaudrate, nh);
	bSuccess &= getModulIDsFromParamSrv(viModulIDs, nh);
	bSuccess &= getUseMoveVelFromParamSrv(bUseMoveVel, nh);
	bSuccess &= getJointNamesFromParamSrv(vsJointNames, nh);
	bSuccess &= getMaxAccelerationsFromParamSrv(vdMaxAccelerations, nh);
	bSuccess &= getFrequencyFromParamSrv(dFrequency, nh);
	bSuccess &= getMinPublishDurationFromParamSrv(rdMinPublishDuration, nh);
	
	// Shutdown the node, if not all parameters are available.
	if (!bSuccess) return false;

	// Initialize parameters.
	Init(sCanModule, sCanDevice, iCanBaudrate, viModulIDs);
	
	// Check dimension of the joint names with with DOF.
	if ((int)vsJointNames.size() != GetDOF())
	{
		ROS_ERROR("Wrong dimensions of parameter joint_names, shutting down node...");
		return false;
	}
	
	// Check dimension of the max acceleration values with with DOF.
	if ((int)vdMaxAccelerations.size() != GetDOF())
	{
		ROS_ERROR("Wrong dimensions of parameter max_accelerations, shutting down node...");
		return false;
	}
	
	// Check the min publish duration.
	if((1.0/rdMinPublishDuration.toSec()) > dFrequency)
	{
		ROS_ERROR("min_publish_duration has to be longer then delta_t of controller frequency!");
		return false;
	}
	
	// Set the use move velocity flag.
	SetUseMoveVel(bUseMoveVel);

	// Set the joint names.
	SetJointNames(vsJointNames);
	
	// Set the max acceleration values.
	SetMaxAcc(vdMaxAccelerations);

	// Set the main loop frequency.
	SetFrequency(dFrequency);

	// Set the min publish duration.
	SetMinPublishDuration(rdMinPublishDuration);
	
	return true;
}


bool PowerCubeCtrlParams::getRobotModelNameFromParamSrv(ros::NodeHandle nh, std::string& sXml)
{
	std::string param_name, full_param_name;
	
	param_name = "robot_description";
	nh.searchParam(param_name, full_param_name);
	
	if (nh.hasParam(full_param_name))
	{
		nh.getParam(full_param_name.c_str(), sXml);
		
		if (sXml.size() == 0)
		{
			ROS_ERROR("Unable to load robot model from parameter %s",full_param_name.c_str());
			
			return false;
		}
		
		ROS_DEBUG("%s content\n%s", full_param_name.c_str(), sXml.c_str());
		
		return true;
	}
	else
	{
		ROS_ERROR("Parameter %s not set, shutting down node...", full_param_name.c_str());
		
		return false;
	}
}


bool PowerCubeCtrlParams::getRobotModelFromParamSrv(ros::NodeHandle nh, std::string sXml, 
													urdf::Model& urdfModel)
{
	if (urdfModel.initString(sXml))
	{
		ROS_DEBUG("Successfully parsed urdf file");
		
		return true;
	}
	else
	{
		ROS_ERROR("Failed to parse urdf file");
		
		return false;
	}
}


void PowerCubeCtrlParams::getMaxVelFromParamSrv(ros::NodeHandle nh, urdf::Model urdfModel,
												unsigned int uiDOF, std::vector<std::string> vsJointNames,
												std::vector<double>& vdMaxVelocities)
{
	vdMaxVelocities.resize(uiDOF);

	for (unsigned int i = 0; i < uiDOF; i++)
	{
		vdMaxVelocities[i] = urdfModel.getJoint(vsJointNames[i].c_str())->limits->velocity;
	}
}


void PowerCubeCtrlParams::getLowerLimitsFromParamSrv(ros::NodeHandle nh, urdf::Model urdfModel,
													 unsigned int uiDOF, std::vector<std::string> vsJointNames,
													 std::vector<double>& vdLowerLimits)
{
	vdLowerLimits.resize(uiDOF);
	
	for (unsigned int i = 0; i < uiDOF; i++)
	{
		vdLowerLimits[i] = urdfModel.getJoint(vsJointNames[i].c_str())->limits->lower;
	}
}


void PowerCubeCtrlParams::getUpperLimitsFromParamSrv(ros::NodeHandle nh, urdf::Model urdfModel,
													 unsigned int uiDOF, std::vector<std::string> vsJointNames,
													 std::vector<double>& vdUpperLimits)
{
	vdUpperLimits.resize(uiDOF);
	
	for (unsigned int i = 0; i < uiDOF; i++)
	{
		vdUpperLimits[i] = urdfModel.getJoint(vsJointNames[i].c_str())->limits->upper;
	}
}


void PowerCubeCtrlParams::getOffsetsFromParamSrv(ros::NodeHandle nh, urdf::Model urdfModel,
												 unsigned int uiDOF, std::vector<std::string> vsJointNames,
												 std::vector<double>& vdOffsets)
{
	vdOffsets.resize(uiDOF);
	
	for (unsigned int i = 0; i < uiDOF; i++)
	{
		vdOffsets[i] = urdfModel.getJoint(vsJointNames[i].c_str())->calibration->rising.get()[0];
	}
}


bool PowerCubeCtrlParams::getRobotDescriptionFromParamSrv(ros::NodeHandle nh)
{
	unsigned int uiDOF;
	std::vector<std::string> vsJointNames;
	std::string sXml;
	urdf::Model urdfModel;
	std::vector<double> vdMaxVelocities, vdLowerLimits, vdUpperLimits, vdOffsets;
	
	// Get the degrees of freedom and the joint names.
	uiDOF = GetDOF();
	vsJointNames = GetJointNames();
	
	// Get the name of the robot model.
	if (!getRobotModelNameFromParamSrv(nh, sXml)) return false;
		
	// Get the robot model.
	if (!getRobotModelFromParamSrv(nh, sXml, urdfModel)) return false;
	
	// Get the parameters from the robot model.
	getMaxVelFromParamSrv(nh, urdfModel, uiDOF, vsJointNames, vdMaxVelocities);
	getLowerLimitsFromParamSrv(nh, urdfModel, uiDOF, vsJointNames, vdLowerLimits);
	getUpperLimitsFromParamSrv(nh, urdfModel, uiDOF, vsJointNames, vdUpperLimits);
	getOffsetsFromParamSrv(nh, urdfModel, uiDOF, vsJointNames, vdOffsets);

	// Set the parameters.
	SetMaxVel(vdMaxVelocities);
	SetLowerLimits(vdLowerLimits);
	SetUpperLimits(vdUpperLimits);
	SetOffsets(vdOffsets);
	
	return true;
}