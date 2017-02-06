#ifndef __POWER_CUBE_CTRL_H_
#define __POWER_CUBE_CTRL_H_


// ROS includes
#include <ros/ros.h>

// standard includes
#include <iostream>
#include <sstream>
#include <string>
#include <deque>
#include <pthread.h> 

// own includes
#include <schunk_libm5api/m5apiw32.h>
//#include <schunk_control/moveCommand.h>
#include <schunk_control/PowerCubeCtrlParams.h>


class PowerCubeCtrl
{
public:
	/// Constructor
	PowerCubeCtrl(PowerCubeCtrlParams * params);

	/// Destructor
	~PowerCubeCtrl();

	typedef enum
	{
		PC_CTRL_OK = 0, PC_CTRL_NOT_HOMED = -1, PC_CTRL_ERR = -2, PC_CTRL_POW_VOLT_ERR = -3
	} PC_CTRL_STATUS;

	/////////////////////////////////////////////
	// Functions for initialization and close: //
	/////////////////////////////////////////////

	/*!
	 * \brief Initializing
	 */
	bool Init();

	/*!
	 * \brief Checking if is initialized
	 */
	bool isInitialized() const;

	/*!
	 * \brief Get error message
	 */
	std::string getErrorMessage() const;
	
	/*!
	 * \brief Get PC_Status message
	 */
	PowerCubeCtrl::PC_CTRL_STATUS getPC_Status() const;

	/*!
	 * \brief Close
	 */
	bool Close();
	
	/*!
	 * \brief Checking if is initialized
	 */
	bool PCTRL_CHECK_INITIALIZED();

	////////////////////////////
	// Functions for control: //
	////////////////////////////

	/*!
	 * \brief Send position goals to powercubes, the final angles will be reached simultaneously
	 */
	bool MoveJointSpaceSync(const std::vector<double>& angles);

	/*!
	 * \brief Move joints with calculated velocities
	 * 
	 * Calculating positions and times by desired value of the cob_trajectory_controller
	 * 
	 * \param vdVel Velcocity vector.
	 */
	bool MoveVel(const std::vector<double>& vdVel);
	
	/*!
	 * \brief Move joints to the given positions.
	 * 
	 * 
	 * \param vdPos Position vector.
	 */
	bool MovePos(const std::vector<double>& vdPos);

	/*!
	 * \brief Move joints to the given positions with the given
	 * velocity and acceleration.
	 *
	 *
	 * \param vdPos Position vector.
	 * \param vdVel Velcocity vector.
	 * \param vdAcc Acceleration vector.
	 */
	bool MoveRamp(const std::vector<double>& vdPos,
				  const std::vector<double>& vdVel,
				  const std::vector<double>& vdAcc);

	/*!
	 * \brief Stops the Manipulator immediately
	 */
	bool Stop();

	/*!
	 * \brief Recovery after emergency stop or power supply failure
	 */
	bool Recover();

	//////////////////////////////////
	// functions to set parameters: //
	//////////////////////////////////

	/*!
	 * \brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
	 *
	 * A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
	 */
	bool setMaxVelocities();

	/*!
	 * \brief Sets the maximum angular velocity (rad/s) for the Joints, use with care!
	 *
	 * A Value of 0.5 is already pretty fast, you probably don't want anything more than one...
	 */
	bool setMaxAccelerations();

	/*!
	 * \brief Configure powercubes to start all movements synchronously
	 *
	 * Tells the Modules not to start moving until PCube_startMotionAll is called.
	 */
	bool setSyncMotion();

	/*!
	 * \brief Configure powercubes to start all movements asynchronously
	 *
	 * Tells the Modules to start immediately
	 */
	bool setASyncMotion();

	/////////////////////////////////////////////////
	// Functions for getting state and monitoring: //
	/////////////////////////////////////////////////

	/*!
	 * \brief Returns the state of all modules
	 */
	bool updateStates();

	/*!
	 * \brief Gets the status of the modules
	 */
	bool getStatus(PowerCubeCtrl::PC_CTRL_STATUS& status, std::vector<std::string>& errorMessages);
	
	/*!
	 * \brief Returns true if any of the Joints are still moving
	 *
	 * Should also return true if Joints are accelerating or decelerating
	 */
	bool statusMoving();

	/*!
	 * \brief Gets the current positions
	 */
	std::vector<double> getPositions();

	/*!
	 * \brief Gets the current velcities
	 */
	std::vector<double> getVelocities();

	/*!
	 * \brief Gets the current accelerations
	 */
	std::vector<double> getAccelerations();

	/*!
	 * \brief Waits until all Modules are homed.
	 *
	 * Homes only Schunk PW-Modules or PRL-Modules don't need to be homed.
	 */
	bool doHoming();

protected:
	pthread_mutex_t m_mutex;

	int m_iDeviceHandle;
	bool m_bInitialized;
	bool m_bCANDeviceOpened;

	PowerCubeCtrlParams* m_pPcParams;
	PC_CTRL_STATUS m_pcStatus;

	std::vector<unsigned long> m_vulStatus;
	std::vector<std::string> m_vsModuleTypes;
	std::vector<unsigned long> m_vulVersion;
	std::vector<unsigned char> m_vucDios;
	std::vector<double> m_vdPositions;
	std::deque< std::vector<double> > m_vdCachedPos;
	std::vector<double> m_vdVelocities;
	std::vector<double> m_vdAccelerations;

	ros::Time m_rtLastTimePub;

	std::string m_sErrorMessage;
};

#endif
