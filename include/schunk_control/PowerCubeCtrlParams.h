#ifndef __POWER_CUBE_CTRL_PARAMS_H_
#define __POWER_CUBE_CTRL_PARAMS_H_


// ROS includes
#include <ros/ros.h>
#include <urdf/model.h>


#include <string>
#include <vector>
#include <iostream>


/*!
 * \brief Parameters for powercube_chain
 *
 * Initializing and setting parameters for powercube_chain
 */
class PowerCubeCtrlParams
{
public:
	/// Constructor
	PowerCubeCtrlParams();

	/// Destructor
	~PowerCubeCtrlParams();

	/// Initializing
	void Init(std::string sCanModule, std::string sCanDevice, int iBaudrate, std::vector<int> viModuleIDs);

	/// Sets the DOF value
	void SetDOF(int DOF);

	/// Gets the DOF value
	int GetDOF();

	/// Sets UseMoveVel
	void SetUseMoveVel(bool bUseMoveVel);

	/// Gets UseMoveVel
	bool GetUseMoveVel();

	/// Sets the CAN Module
	void SetCanModule(std::string sCanModule);

	/// Gets the CAN Module
	std::string GetCanModule();

	/// Sets the CAN Device
	void SetCanDevice(std::string sCanDevice);

	/// Gets the CAN Device
	std::string GetCanDevice();

	/// Sets the Baudrate
	void SetBaudrate(int iBaudrate);

	/// Gets the Baudrate
	int GetBaudrate();

	/// Gets the Module IDs
	std::vector<int> GetModuleIDs();
	
	/// Gets the ModuleID
	int GetModuleID(int iNo);
	
	/// Sets the Module IDs
	bool SetModuleID(int iNo, int iID);

	/// Gets the joint names
	std::vector<std::string> GetJointNames();

	/// Sets the joint names
	bool SetJointNames(std::vector<std::string> vsJointNames);

	////////////////////////////////////////
	// Functions for angular constraints: //
	////////////////////////////////////////

	/// Sets the upper angular limits (rad) for the joints
	bool SetUpperLimits(std::vector<double> vdUpperLimits);

	/// Sets the lower angular limits (rad) for the joints
	bool SetLowerLimits(std::vector<double> vdLowerLimits);

	/// Sets the offset angulars (rad) for the joints
	bool SetOffsets(std::vector<double> vdAngleOffsets);

	/// Sets the max. angular velocities (rad/s) for the joints
	bool SetMaxVel(std::vector<double> vdMaxVel);

	/// Sets the max. angular accelerations (rad/s^2) for the joints
	bool SetMaxAcc(std::vector<double> vdMaxAcc);

	/// Gets the upper angular limits (rad) for the joints
	std::vector<double> GetUpperLimits();

	/// Gets the lower angular limits (rad) for the joints
	std::vector<double> GetLowerLimits();

	/// Gets the offset angulars (rad) for the joints
	std::vector<double> GetOffsets();

	/// Gets the max. angular accelerations (rad/s^2) for the joints
	std::vector<double> GetMaxAcc();

	/// Gets the max. angular velocities (rad/s) for the joints
	std::vector<double> GetMaxVel();
		
	/// Sets the frequency.
	void SetFrequency(double dFrequency);

	/// Gets the frequency.
	double GetFrequency();
	
	/// Sets the min publish duration.
	void SetMinPublishDuration(ros::Duration rdMinPublishDuration);

	/// Gets the min publish duration.
	ros::Duration GetMinPublishDuration();
	
	bool getCanModuleFromParamSrv(std::string& sCanModule, ros::NodeHandle nh);
	
	bool getCanDeviceFromParamSrv(std::string& sCanDevice, ros::NodeHandle nh);
	
	bool getCanBaudrateFromParamSrv(int& iCanBaudrate, ros::NodeHandle nh);

	bool getModulIDsFromParamSrv(std::vector<int>& viModulIDs, ros::NodeHandle nh);

	bool getUseMoveVelFromParamSrv(bool& bUseMoveVel, ros::NodeHandle nh);

	bool getJointNamesFromParamSrv(std::vector<std::string>& vsJointNames, 
														ros::NodeHandle nh);

	bool getMaxAccelerationsFromParamSrv(std::vector<double>& vdMaxAccelerations, 
															ros::NodeHandle nh);

	bool getFrequencyFromParamSrv(double& dFrequency, ros::NodeHandle nh);

	bool getMinPublishDurationFromParamSrv(ros::Duration& rdMinPublishDuration, 
																ros::NodeHandle nh);
	
	bool getROSParametersFromParamSrv(ros::NodeHandle nh);
	
	bool getRobotModelNameFromParamSrv(ros::NodeHandle nh, std::string& sXml);

	bool getRobotModelFromParamSrv(ros::NodeHandle nh, std::string sXml, 
														urdf::Model& urdfModel);

	void getMaxVelFromParamSrv(ros::NodeHandle nh, urdf::Model urdfModel,
													unsigned int uiDOF, std::vector<std::string> vsJointNames,
													std::vector<double>& vdMaxVelocities);

	void getLowerLimitsFromParamSrv(ros::NodeHandle nh, urdf::Model urdfModel,
														unsigned int uiDOF, std::vector<std::string> vsJointNames,
														std::vector<double>& vdLowerLimits);

	void getUpperLimitsFromParamSrv(ros::NodeHandle nh, urdf::Model urdfModel,
														unsigned int uiDOF, std::vector<std::string> vsJointNames,
														std::vector<double>& vdUpperLimits);
	void getOffsetsFromParamSrv(ros::NodeHandle nh, urdf::Model urdfModel,
													unsigned int uiDOF, std::vector<std::string> vsJointNames,
													std::vector<double>& vdOffsets);
	
	bool getRobotDescriptionFromParamSrv(ros::NodeHandle nh);

private:
	int m_iDOF;
	int m_iBaudrate;
	
	double m_dFrequency;

	std::string m_sCanModule;
	std::string m_sCanDevice;
	
	bool m_bUseMoveVel;
	
	std::vector<int> m_viModulIDs;
	
	std::vector<std::string> m_vsJointNames;
	
	std::vector<double> m_vdOffsets;
	std::vector<double> m_vdUpperLimits;
	std::vector<double> m_vdLowerLimits;
	std::vector<double> m_vdMaxVel;
	std::vector<double> m_vdMaxAcc;
	
	ros::Duration m_rdMinPublishDuration;
};

#endif
