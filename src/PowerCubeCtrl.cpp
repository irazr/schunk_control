#include <PowerCubeCtrl.h>


PowerCubeCtrl::PowerCubeCtrl(PowerCubeCtrlParams * pPcParams)
{
    m_mutex = PTHREAD_MUTEX_INITIALIZER;

	m_bCANDeviceOpened = false;
	m_bInitialized = false;

	m_pPcParams = pPcParams;
	
	m_rtLastTimePub = ros::Time::now();

	m_pcStatus = PC_CTRL_OK;
}


PowerCubeCtrl::~PowerCubeCtrl()
{		
	// Stop all components.
	Stop(); 

	// Close CAN device.
	if (m_bCANDeviceOpened)
	{
		pthread_mutex_lock(&m_mutex);
		PCube_closeDevice(m_iDeviceHandle);
		pthread_mutex_unlock(&m_mutex);
	}
}


bool PowerCubeCtrl::Init()
{
	int iRet, iDOF, iCanBaudrate;
	std::string sCanModule, sCanDevice;
	std::vector<int> viModulIDs;
	std::vector<double> vdMaxVel, vdMaxAcc, vdOffsets, vdLowerLimits, vdUpperLimits;
	
	iRet = 0;
	
	// Get some parameters from the parameter object.
	iDOF 		  = m_pPcParams->GetDOF();
	sCanModule 	  = m_pPcParams->GetCanModule();
	sCanDevice 	  = m_pPcParams->GetCanDevice();
	viModulIDs 	  = m_pPcParams->GetModuleIDs();
	iCanBaudrate  = m_pPcParams->GetBaudrate();
	vdMaxVel 	  = m_pPcParams->GetMaxVel();
	vdMaxAcc 	  = m_pPcParams->GetMaxAcc();
	vdOffsets 	  = m_pPcParams->GetOffsets();
	vdLowerLimits = m_pPcParams->GetLowerLimits();
	vdUpperLimits = m_pPcParams->GetUpperLimits();

	// Resize the vectors  with the amount of degrees of freedom.
	m_vulStatus.resize(iDOF);
	m_vsModuleTypes.resize(iDOF);
	m_vulVersion.resize(iDOF); 
	m_vucDios.resize(iDOF);
	m_vdPositions.resize(iDOF);
	m_vdVelocities.resize(iDOF);
	m_vdAccelerations.resize(iDOF);

	// Output of current settings in the terminal.
	std::cout << "=========================================================================== " << std::endl;
	std::cout << "PowerCubeCtrl:Init: Trying to initialize with the following parameters: " << std::endl;
	std::cout << "DOF: " << iDOF << std::endl;
	std::cout << "CanModule: " << sCanModule << std::endl;
	std::cout << "CanDevice: " << sCanDevice << std::endl;
	std::cout << "CanBaudrate: " << iCanBaudrate << std::endl;
	
	std::cout << "ModulIDs: ";
	for (int i = 0; i < iDOF; i++)
	{
		std::cout << viModulIDs[i] << " ";
	}
	std::cout << std::endl;

	std::cout << std::endl << "maxVel: ";
	for (int i = 0; i < iDOF; i++)
	{
		std::cout << vdMaxVel[i] << " ";
	}

	std::cout << std::endl << "maxAcc: ";
	for (int i = 0; i < iDOF; i++)
	{
		std::cout << vdMaxAcc[i] << " ";
	}

	std::cout << std::endl << "upperLimits: ";
	for (int i = 0; i < iDOF; i++)
	{
		std::cout << vdUpperLimits[i] << " ";
	}

	std::cout << std::endl << "lowerLimits: ";
	for (int i = 0; i < iDOF; i++)
	{
		std::cout << vdLowerLimits[i] << " ";
	}

	std::cout << std::endl << "offsets: ";
	for (int i = 0; i < iDOF; i++)
	{
		std::cout << vdOffsets[i] << " ";
	}

	std::cout << std::endl << "=========================================================================== " << std::endl;
	std::ostringstream InitStr;
	InitStr << sCanModule << ":" << sCanDevice << "," << iCanBaudrate;
	std::cout << "initstring = " << InitStr.str().c_str() << std::endl;

	// Open the device.
	pthread_mutex_lock(&m_mutex);
    iRet = PCube_openDevice(&m_iDeviceHandle, InitStr.str().c_str());
    //iRet = PCube_openDevice(&m_iDeviceHandle, "RS232:1,9600");
	pthread_mutex_unlock(&m_mutex);
	
	// Check, if the device could be opened.
	if (iRet != 0)
	{
		std::ostringstream errorMsg;
		errorMsg << "Could not open device " << sCanDevice << 
			", m5api error code: " << iRet;
		m_sErrorMessage = errorMsg.str();
		
		return false;
	}
	m_bCANDeviceOpened = true;

	// Reset all modules of the chain.
	int max_tries = 3; 
	for (int i = 0; i < iDOF; i++)
	{  	
		for (int reset_try = 0; reset_try < max_tries; reset_try++)
		{	
			// Reset the module.
			pthread_mutex_lock(&m_mutex);
			iRet =  PCube_resetModule(m_iDeviceHandle, viModulIDs.at(i));
			pthread_mutex_unlock(&m_mutex);
			
			// If the module could be reseted, we can go the next module.
			// Else we try it one more time after a little break.
			// If the module couldn't be reseted after max_tries, the init
			// method stops.
			if (iRet == 0)
			{	
				// Go to the next module.
				break; 
			}
			else if ((iRet != 0) && (reset_try == (max_tries-1)))
			{
				// Output some error message and stop the init procedure.
				std::ostringstream errorMsg;
				errorMsg << "Could not reset module " << viModulIDs.at(i) << 
					" during init. Errorcode during reset: " << iRet << " Try to init once more.";
				m_sErrorMessage = errorMsg.str();
				
				return false;
			}
			else
			{
				// Make a little break and try it one more time.
				usleep(1500000); 
			}
		}
	}
	std::cout << "number of moduleIDs" << viModulIDs.size() << std::endl;

	// Check the number of modules connected to the bus.
	pthread_mutex_lock(&m_mutex);
	int number_of_modules = PCube_getModuleCount(m_iDeviceHandle);
	pthread_mutex_unlock(&m_mutex);
	std::cout << "found " << number_of_modules << " modules." << std::endl;

	// Check if the modules are connected.
	for (int i = 0; i < iDOF; i++)
	{
		unsigned long serNo;
		unsigned short verNo;
		unsigned long defConfig;
		std::vector<std::string> Module_Types; 
		
		// Retrieve serial number.
		pthread_mutex_lock(&m_mutex);
		iRet = PCube_getModuleSerialNo(m_iDeviceHandle, viModulIDs[i], &serNo);
		pthread_mutex_unlock(&m_mutex);
		if (iRet != 0)
		{
			std::ostringstream errorMsg;
			errorMsg << "Could not find Module with ID " << viModulIDs[i] << 
				", m5api error code: " << iRet;
			m_sErrorMessage = errorMsg.str();
			
			return false;
		}
		
		// Retrieve version number.	
		pthread_mutex_lock(&m_mutex);
		iRet = PCube_getModuleVersion(m_iDeviceHandle, viModulIDs[i], &verNo);
		pthread_mutex_unlock(&m_mutex);
		if (iRet != 0)
		{
			std::ostringstream errorMsg;
			errorMsg << "Could not find Module with ID " << viModulIDs[i] << 
				", m5api error code: " << iRet;
			m_sErrorMessage = errorMsg.str();
			
			return false;
		}
		else
		{
			m_vulVersion[i] = verNo; 
		}	
			
		// Retrieve defined gear ratio.
		float gear_ratio; 	
		pthread_mutex_lock(&m_mutex);
		iRet = PCube_getDefGearRatio(m_iDeviceHandle, viModulIDs[i], &gear_ratio);
		pthread_mutex_unlock(&m_mutex);
		if (iRet != 0)
		{
			std::ostringstream errorMsg;
			errorMsg << "Could not get Module type with ID " << viModulIDs[i] << 
				", m5api error code: " << iRet;
			m_sErrorMessage = errorMsg.str();
			
			return false;
		}
		else
		{
			std::cout << "gear ratio: " << gear_ratio << std::endl; 
		}	
			
		// Retrieve axis type (linear or rotational). 
		unsigned char type; 	
		pthread_mutex_lock(&m_mutex);
		iRet = PCube_getModuleType(m_iDeviceHandle, viModulIDs[i], &type);
		pthread_mutex_unlock(&m_mutex);
		if (iRet != 0)
		{
			std::ostringstream errorMsg;
			errorMsg << "Could not get Module type with ID " << viModulIDs[i] << 
				", m5api error code: " << iRet;
			m_sErrorMessage = errorMsg.str();
			
			return false;
		}
		else
		{
			if (type != TYPEID_MOD_ROTARY)
			{
				std::cout << "wrong module type configured. Type must be rotary axis." << 
					"Use Windows configuration software to change type." << std::endl; 
				return false; 
			} 
		}	

		// Find out module_type.
		// The typ -if PW or PRL- can be distinguished by the typ of encoder. 
		pthread_mutex_lock(&m_mutex);
		iRet = PCube_getDefSetup(m_iDeviceHandle, viModulIDs[i], &defConfig);
		pthread_mutex_unlock(&m_mutex);

		ROS_DEBUG("module type check: %li (std::dec)",defConfig); 
		
		if (iRet != 0)
		{
			std::ostringstream errorMsg;
			errorMsg << "Error on communication with module " << viModulIDs[i] << 
				", m5api error code: " << iRet;
			m_sErrorMessage = errorMsg.str();
			
			return false;
		}	
	
		// Firmware version 4634 of PRL modules replies ABSOULTE_FEEDBACK, firmware 
		// 4638 replies RESOLVER_FEEDBACK. Both means the same: Module is PRL. 
		// PW modules have encoders (ENCODER_FEEDBACK, s.M5API), but this bit is not 
		// set is DefConfig word. For new firmware versions this needs to be evaluated. 
		if (((defConfig & CONFIG_ABSOLUTE_FEEDBACK)==CONFIG_ABSOLUTE_FEEDBACK) || 
			((defConfig & CONFIG_RESOLVER_FEEDBACK)==CONFIG_RESOLVER_FEEDBACK))
		{
			m_vsModuleTypes[i] = "PRL"; 
			ROS_DEBUG("Module %i is from type: PRL", i);
		}
		else
		{
			m_vsModuleTypes[i] = "PW"; 
			ROS_DEBUG("Module %i is from type: PW", i);
		}

		// Otherwise success.
		std::cout << "Found module " << std::dec << viModulIDs[i] << " Serial: " <<
			serNo << " Version: " << std::hex << verNo << std::endl;
	}

	// Modules should be initialized now.
	m_pcStatus = PC_CTRL_OK;
	m_bInitialized = true; 

	// Check if modules are in normal state.
	std::vector<std::string> errorMessages;
	PC_CTRL_STATUS status;

	// Update status variables.
	updateStates();

	// Grep updated status.
	getStatus(status, errorMessages);
	
	// Set the maximum values for velocity and acceleration.
	if (!setMaxVelocities()) return false;
	if (!setMaxAccelerations()) return false;

	// Homing dependant on moduletype and if already homed.
	bool successful = false;
	successful = doHoming();
	if (!successful)
	{
		std::cout << "PowerCubeCtrl:Init: homing not successful, aborting ...\n";
		
		return false;
	}

	// All modules initialized successfully.
	m_pcStatus = PC_CTRL_OK;

	return true;
}


bool PowerCubeCtrl::Close()
{
	if (m_bCANDeviceOpened)
	{
		m_bInitialized = false;
		m_bCANDeviceOpened = false;

		pthread_mutex_lock(&m_mutex);
		PCube_closeDevice(m_iDeviceHandle);
		pthread_mutex_unlock(&m_mutex);

		return true;
	}
	else
	{
		return false;
	}
}


bool PowerCubeCtrl::MoveRamp(const std::vector<double>& vdPos,
							 const std::vector<double>& vdVel,
							 const std::vector<double>& vdAcc)
{
	unsigned int iDOF;
	int iRet;
	float fPos;
	std::vector<int> viModulIDs;
	std::vector<double> vdPositions, vdVelocities, vdAccelerations,
			vdLowerLimits, vdUpperLimits, vdMaxVels, vdMaxAccs, vdOffsets;
	std::vector<std::string> vsErrorMessages;
	PC_CTRL_STATUS status;

	PCTRL_CHECK_INITIALIZED();

	//== init var ==================================================

	// Get the degrees of freedom and the modul IDs from the parameter object.
	iDOF = m_pPcParams->GetDOF();
	viModulIDs = m_pPcParams->GetModuleIDs();

	// Save the position values.
	vdPositions.resize(iDOF);
	vdPositions = vdPos;

	// Save the velocity values.
	vdVelocities.resize(iDOF);
	vdVelocities = vdVel;

	// Save the acceleration values.
	vdAccelerations.resize(iDOF);
	vdAccelerations = vdAcc;

	// Get the limit values.
	vdLowerLimits = m_pPcParams->GetLowerLimits();
	vdUpperLimits = m_pPcParams->GetUpperLimits();
	vdMaxVels = m_pPcParams->GetMaxVel();
	vdMaxAccs = m_pPcParams->GetMaxAcc();

	// Get offsets.
	vdOffsets = m_pPcParams->GetOffsets();

	// Save this time as the last publish time.
	m_rtLastTimePub = ros::Time::now();

	// == check input parameter =====================================

	// Check the dimension of the position, velocity and acceleration vector.
	if (vdPositions.size() != iDOF || vdVelocities.size() != iDOF || vdAccelerations.size() != iDOF)
	{
		m_sErrorMessage = "Skipping command: Commanded ramp and iDOF are not same dimension.";
		return false;
	}

	for (unsigned int i = 0; i < iDOF; i++)
	{
		// Check position lower limits.
		if (vdPositions[i] < (vdLowerLimits[i]+vdOffsets[i]))
		{
			ROS_INFO("Skipping command: %f Target position exceeds lower limit (%f).",
					 vdPositions[i], vdLowerLimits[i]);

			pthread_mutex_lock(&m_mutex);
			PCube_haltModule(m_iDeviceHandle, viModulIDs.at(i));
			pthread_mutex_unlock(&m_mutex);

			return true;
		}

		// Check position upper limits.
		if (vdPositions[i] > (vdUpperLimits[i]+vdOffsets[i]))
		{
			ROS_INFO("Skipping command: %f Target position exceeds upper limit (%f).",
					 vdPositions[i], vdUpperLimits[i]);

			pthread_mutex_lock(&m_mutex);
			PCube_haltModule(m_iDeviceHandle, viModulIDs.at(i));
			pthread_mutex_unlock(&m_mutex);

			return true;
		}

		// Check the velocity limits.
		if(vdVelocities[i] > vdMaxVels[i])
		{
			// Set velocities command to max value.
			vdVelocities[i] = vdMaxVels[i];

			ROS_INFO("Velocity %f exceeds limit %f for axis %i. moving with max velocity %f instead",
					 vdVelocities[i], vdMaxVels[i], i, vdMaxVels[i]);
		}

		// Check the acceleration limits.
		if(vdAccelerations[i] > vdMaxAccs[i])
		{
			// Set acceleration command to max value.
			vdAccelerations[i] = vdMaxAccs[i];

			ROS_INFO("Acceleration %f exceeds limit %f for axis %i. moving with max acceleration %f instead",
					 vdAccelerations[i], vdMaxAccs[i], i, vdMaxAccs[i]);
		}
	}

	//== check system status ======================================

	getStatus(status, vsErrorMessages);

	if ((status != PC_CTRL_OK))
	{
		m_sErrorMessage.assign("");
		for (unsigned int i = 0; i < iDOF; i++)
		{
			m_sErrorMessage.append(vsErrorMessages[i]);
		}
		ROS_INFO("Error during movement. Status: %i	\n", status);

		return false;
	}

	//== send position cmd to modules ==============================
	for (unsigned int i = 0; i < iDOF; i++)
	{
		pthread_mutex_lock(&m_mutex);
		iRet = PCube_moveRampExtended(m_iDeviceHandle, viModulIDs[i],
									  vdPositions[i], vdVelocities[i], vdAccelerations[i],
									  &m_vulStatus[i], &m_vucDios[i], &fPos);
		pthread_mutex_unlock(&m_mutex);

		// Error handling.
		if (iRet != 0)
		{
			ROS_DEBUG("Com Error: %i", iRet);
		}
	}

	pthread_mutex_lock(&m_mutex);
	PCube_startMotionAll(m_iDeviceHandle);
	pthread_mutex_unlock(&m_mutex);

	return true;
}


bool PowerCubeCtrl::MovePos(const std::vector<double>& vdPos)
{
	unsigned int iDOF;
	int iRet;
	float fPos;
	std::vector<int> viModulIDs;
	std::vector<double> vdPositions, vdLowerLimits, vdUpperLimits, maxVels, vdOffsets;
	std::vector<std::string> vsErrorMessages;
	PC_CTRL_STATUS status;
	
	PCTRL_CHECK_INITIALIZED();

	//== init var ==================================================

	// Get the degrees of freedom and the modul IDs from the parameter object.
	iDOF = m_pPcParams->GetDOF();
	viModulIDs = m_pPcParams->GetModuleIDs();

	// Save the position values.
	vdPositions.resize(iDOF); 
	vdPositions = vdPos;

	// Get the limit values.
	vdLowerLimits = m_pPcParams->GetLowerLimits();
	vdUpperLimits = m_pPcParams->GetUpperLimits();
	maxVels = m_pPcParams->GetMaxVel();

	// Get offsets.
	vdOffsets = m_pPcParams->GetOffsets();

	// Save this time as the last publish time.
	m_rtLastTimePub = ros::Time::now();  

	// == check input parameter =====================================

	// Check the dimension of the position vector.
	if (vdPositions.size() != iDOF)
	{
		m_sErrorMessage = "Skipping command: Commanded positions and iDOF are not same dimension.";
		return false;
	}

	for (unsigned int i = 0; i < iDOF; i++)
	{
		// Check position lower limits.
		if (vdPositions[i] < (vdLowerLimits[i]+vdOffsets[i]))
		{	
			ROS_INFO("Skipping command: %f Target position exceeds lower limit (%f).", 
					 vdPositions[i], vdLowerLimits[i]);		

			pthread_mutex_lock(&m_mutex);
			PCube_haltModule(m_iDeviceHandle, viModulIDs.at(i));
			pthread_mutex_unlock(&m_mutex);
			
			return true; 
		} 
			
		// Check position upper limits.
		if (vdPositions[i] > (vdUpperLimits[i]+vdOffsets[i]))
		{	
			ROS_INFO("Skipping command: %f Target position exceeds upper limit (%f).", 
					 vdPositions[i], vdUpperLimits[i]);		

			pthread_mutex_lock(&m_mutex);
			PCube_haltModule(m_iDeviceHandle, viModulIDs.at(i));
			pthread_mutex_unlock(&m_mutex);

			return true; 
		} 
	}

	//== check system status ====================================== 

	getStatus(status, vsErrorMessages);

	if ((status != PC_CTRL_OK))
	{
		m_sErrorMessage.assign("");
		for (unsigned int i = 0; i < iDOF; i++)
		{
			m_sErrorMessage.append(vsErrorMessages[i]);
		}
		ROS_INFO("Error during movement. Status: %i	\n", status);
		
		return false;
	}

    PCube_resetAll(0);

	//== send position cmd to modules ============================== 
	for (unsigned int i = 0; i < iDOF; i++)
	{
		pthread_mutex_lock(&m_mutex);
        PCube_setRampVel(m_iDeviceHandle, m_pPcParams->GetModuleID(i), 0.1);
        PCube_setRampAcc(m_iDeviceHandle, m_pPcParams->GetModuleID(i), 0.1);
        iRet = PCube_movePosExtended(m_iDeviceHandle, m_pPcParams->GetModuleID(i),
                                     vdPositions[i], &m_vulStatus[i], &m_vucDios[i], &fPos);
        //iRet = PCube_movePos(m_iDeviceHandle, m_pPcParams->GetModuleID(i), vdPositions[i]);
		pthread_mutex_unlock(&m_mutex);

        ROS_INFO("MODULE ID: %i", m_pPcParams->GetModuleID(i));
        ROS_INFO("vdPositions: %f", vdPositions[i]);
        std::cout << "m_vulStatus: " << m_vulStatus.at(i) << std::endl;
        std::cout << "m_vucDios: " << m_vucDios.at(i) << std::endl;
        ROS_INFO("fPos: %f", fPos);
		// Error handling.
		if (iRet != 0)
		{
            ROS_INFO("Com Error: %i", iRet);
		}
	}

	pthread_mutex_lock(&m_mutex);
	PCube_startMotionAll(m_iDeviceHandle);
	pthread_mutex_unlock(&m_mutex);

	return true;
}


bool PowerCubeCtrl::MoveVel(const std::vector<double>& vdVel)
{
	unsigned int iDOF;
	int iRet;
	float fPos;
	std::vector<int> viModulIDs;
	std::vector<double> vdVelocities, vdLowerLimits, vdUpperLimits, vdMaxVels, vdOffsets;
	std::vector<std::string> vsErrorMessages;
	PC_CTRL_STATUS status;
	
	PCTRL_CHECK_INITIALIZED();

	//== init var ==================================================
	
	// Get the degrees of freedom and the modul IDs from the parameter object.
	iDOF = m_pPcParams->GetDOF();
	viModulIDs = m_pPcParams->GetModuleIDs();

	// Save the velocity values.
	vdVelocities.resize(iDOF); 
	vdVelocities = vdVel;

	// Get the limit values.
	vdLowerLimits = m_pPcParams->GetLowerLimits();
	vdUpperLimits = m_pPcParams->GetUpperLimits();
	vdMaxVels = m_pPcParams->GetMaxVel();

	// Get offsets.
	vdOffsets = m_pPcParams->GetOffsets();

	// Save this time as the last publish time.
	m_rtLastTimePub = ros::Time::now(); 

	//== check input parameter =====================================

	// Check the dimension of the velocity vector.
	if (vdVelocities.size() != iDOF)
	{
		m_sErrorMessage = "Skipping command: Commanded velocities and iDOF are not same dimension.";
		return false;
	}

	for (unsigned int i = 0; i < iDOF; i++)
	{
		// Check the velocity limits.
		if(vdVelocities[i] > vdMaxVels[i])
		{ 
			// Set velocities command to max value.
			vdVelocities[i] = vdMaxVels[i];

			ROS_INFO("Velocity %f exceeds limit %f for axis %i. moving with max velocity %f instead", 
					 vdVelocities[i], vdMaxVels[i], i, vdMaxVels[i]);
		}
	}

	//== check system status ====================================== 

	getStatus(status, vsErrorMessages);

	if ((status != PC_CTRL_OK))
	{
		m_sErrorMessage.assign("");
		for (unsigned int i = 0; i < iDOF; i++)
		{
			m_sErrorMessage.append(vsErrorMessages[i]);
		}
		ROS_INFO("Error during movement. Status: %i	\n", status);
		
		return false;
	}

	//== send velocity cmd to modules ============================== 

	for (unsigned int i = 0; i < iDOF; i++)
	{
		pthread_mutex_lock(&m_mutex);
		iRet = PCube_moveVelExtended(m_iDeviceHandle, m_pPcParams->GetModuleID(i), 
									 vdVelocities[i], &m_vulStatus[i], &m_vucDios[i], &fPos);
		pthread_mutex_unlock(&m_mutex);

		// Error handling.
		if (iRet != 0)
		{
			ROS_DEBUG("Com Error: %i", iRet); 		  
		}
	}

	pthread_mutex_lock(&m_mutex);
	PCube_startMotionAll(m_iDeviceHandle);
	pthread_mutex_unlock(&m_mutex);

	return true;
}


bool PowerCubeCtrl::Stop()
{	
	unsigned int iDOF;
	std::vector<int> viModulIDs;
	
	// Get the degrees of freedom and the module IDs from the paramerter object.
	iDOF = m_pPcParams->GetDOF();
	viModulIDs = m_pPcParams->GetModuleIDs();

	// Halt the modules.
	for (unsigned int i = 0; i < iDOF; i++)
	{  
		pthread_mutex_lock(&m_mutex);
		int iRet =  PCube_haltModule(m_iDeviceHandle, viModulIDs.at(i));
		pthread_mutex_unlock(&m_mutex);
		if (iRet != 0)
		{
			std::ostringstream errorMsg;
			errorMsg << "Could not reset all modules, m5api error code: " << iRet;
			m_sErrorMessage = errorMsg.str();
			
			return false;
		}
	}

	// Time for reboot.
	usleep(500000);

	// Reset all modules of the chain.
	for (unsigned int i = 0; i < iDOF; i++)
	{  
		pthread_mutex_lock(&m_mutex);
		int iRet =  PCube_resetModule(m_iDeviceHandle, viModulIDs.at(i));
		pthread_mutex_unlock(&m_mutex);
		if (iRet != 0)
		{
			std::ostringstream errorMsg;
			errorMsg << "Could not reset all modules, m5api error code: " << iRet;
			m_sErrorMessage = errorMsg.str();
			
			return false;
		}
	}
	
	return true;
}


bool PowerCubeCtrl::Recover()
{	
	std::vector<std::string> vsErrorMessages;
	PC_CTRL_STATUS status;
	unsigned long ulState;
	unsigned char ucDio;
	float fPos;
	int iRet;
	unsigned int iDOF;
	std::vector<int> viModulIDs;
	std::vector<double> vdMaxVel, vdMaxAcc, vdOffsets;
	
	// Get some parameters from the parameter object.
	iDOF = m_pPcParams->GetDOF();
	viModulIDs = m_pPcParams->GetModuleIDs();
	vdMaxVel = m_pPcParams->GetMaxVel();
	vdMaxAcc = m_pPcParams->GetMaxAcc();
	vdOffsets = m_pPcParams->GetOffsets();

	// Set some default values.
	status = PC_CTRL_OK;
	iRet = 0;

	// Check for each module if reset is necessary.
	for (unsigned int i = 0; i < iDOF; i++)
	{	
		// Get the state and the position of the module.
		pthread_mutex_lock(&m_mutex);
		iRet = PCube_getStateDioPos(m_iDeviceHandle, m_pPcParams->GetModuleID(i), &ulState, &ucDio, &fPos);
		pthread_mutex_unlock(&m_mutex);
		if (iRet != 0)
		{
			m_pcStatus = PC_CTRL_ERR;
			std::cout << "State: Error com with Module: " << i << " Time: " << ros::Time::now() << std::endl; 	
			
			return false; 		
		}
		
		// Reset the model, if it is in error state.
		if (ulState & STATEID_MOD_ERROR)
		{	
			pthread_mutex_lock(&m_mutex);
			iRet = PCube_resetModule(m_iDeviceHandle, m_pPcParams->GetModuleID(i));
			pthread_mutex_unlock(&m_mutex);
			if (iRet != 0)
			{
				m_pcStatus = PC_CTRL_ERR;
				std::cout << "State: Error com with Module: " << i << " Time: " << ros::Time::now() << std::endl; 
				
				return false; 		
			}
		}
	}

	// Time for reboot.
	usleep(500000); 

	// Check is everything is ok.
	updateStates(); 

	// Move the power cube to the home position if necessary.
	if (m_pcStatus == PC_CTRL_NOT_HOMED)
	{
		if (!doHoming())
		{
			return false;
		}
	}

	// Time for reboot.
	usleep(500000);

	// Modules should be recovered now.
	m_pcStatus = PC_CTRL_OK;	

	// Check is everything is ok.
	updateStates(); 
	
	// Check if modules are really back to normal state.
	getStatus(status, vsErrorMessages);

	if ((status != PC_CTRL_OK))
	{
		m_sErrorMessage.assign("");

		for (unsigned int i = 0; i < iDOF; i++)
		{
			m_sErrorMessage.append(vsErrorMessages[i]);
		}
		
		return false;
	}

	// Modules successfully recovered.
	m_pcStatus = PC_CTRL_OK;
	
	return true;
}


bool PowerCubeCtrl::setMaxVelocities()
{	
	std::vector<double> vdMaxVel;
	std::vector<int> viModulIDs;
	unsigned int iDOF;
	int iRet;
	
	PCTRL_CHECK_INITIALIZED();
	
	// Get some parameters from the parameter object.
	iDOF = m_pPcParams->GetDOF();
	vdMaxVel = m_pPcParams->GetMaxVel();
	viModulIDs = m_pPcParams->GetModuleIDs();
	
	for (unsigned int i = 0; i < iDOF; i++)
	{
		// Set the maximum velocity.
		pthread_mutex_lock(&m_mutex);
		iRet = PCube_setMaxVel(m_iDeviceHandle, viModulIDs[i], vdMaxVel[i]);
		pthread_mutex_unlock(&m_mutex);
		if (iRet!=0)
		{	
			std::ostringstream ssErrorMsg;
			ssErrorMsg << "Could not set MaxVelocity in Module ID: " << 
				viModulIDs[i] << ", m5api error code: " << iRet;
			m_sErrorMessage = ssErrorMsg.str();
			
			return false;
		}
	}

	return true;
}


bool PowerCubeCtrl::setMaxAccelerations()
{
	std::vector<double> vdMaxAcc;
	std::vector<int> viModulIDs;
	unsigned int iDOF;
	int iRet;
	
	PCTRL_CHECK_INITIALIZED();
	
	// Get some parameters from the parameter object.
	iDOF = m_pPcParams->GetDOF();
	vdMaxAcc = m_pPcParams->GetMaxAcc();
	viModulIDs = m_pPcParams->GetModuleIDs();
	
	for (unsigned int i = 0; i < iDOF; i++)
	{
		// Set the maximum acceleration.
		pthread_mutex_lock(&m_mutex);
		iRet =  PCube_setMaxAcc(m_iDeviceHandle, viModulIDs[i], vdMaxAcc[i]);
		pthread_mutex_unlock(&m_mutex);
		if (iRet!=0)
		{	
			std::ostringstream ssErrorMsg;
			ssErrorMsg << "Could not set MaxAcceleration in Module ID: " << 
				viModulIDs[i] << ", m5api error code: " << iRet;
			m_sErrorMessage = ssErrorMsg.str();
			
			return false;
		}
	}

	return true;
}


bool PowerCubeCtrl::setSyncMotion()
{
	std::vector<int> viModulIDs;
	unsigned long ulConfWord;
	unsigned int iDOF;
	int iRet;
	
	// Get some parameters from the parameter object.
	viModulIDs = m_pPcParams->GetModuleIDs();
	iDOF = m_pPcParams->GetDOF();

	if (m_bCANDeviceOpened)
	{
		for (unsigned int i = 0; i < iDOF; i++)
		{
			// Get the configuration.
			pthread_mutex_lock(&m_mutex);
			PCube_getConfig(m_iDeviceHandle, viModulIDs[i], &ulConfWord);
			pthread_mutex_unlock(&m_mutex);

			// Set the configuration to synchronous.
			pthread_mutex_lock(&m_mutex);
			iRet = PCube_setConfig(m_iDeviceHandle, viModulIDs[i], 
								   ulConfWord | CONFIGID_MOD_SYNC_MOTION);
			pthread_mutex_unlock(&m_mutex);
			if (iRet!=0)
			{	
				std::ostringstream ssErrorMsg;
				ssErrorMsg << "Could not set SyncMotion in Module ID: " << viModulIDs[i] << 
					", m5api error code: " << iRet;
				m_sErrorMessage = ssErrorMsg.str();
				
				return false;
			}
		}
		
		return true;
	}
	else
	{
		return false;
    }
}


bool PowerCubeCtrl::setASyncMotion()
{
	std::vector<int> viModulIDs;
	unsigned long ulConfWord;
	unsigned int iDOF;
	int iRet;
	
	// Get some parameters from the parameter object.
	viModulIDs = m_pPcParams->GetModuleIDs();
	iDOF = m_pPcParams->GetDOF();

	if (m_bCANDeviceOpened)
	{
		for (unsigned int i = 0; i < iDOF; i++)
		{
			// Get the configuration.
			pthread_mutex_lock(&m_mutex);
			PCube_getConfig(m_iDeviceHandle, viModulIDs[i], &ulConfWord);
			pthread_mutex_unlock(&m_mutex);

			// Set the configuration to asynchronous.
			pthread_mutex_lock(&m_mutex);
			iRet = PCube_setConfig(m_iDeviceHandle, viModulIDs[i], 
								   ulConfWord & (~CONFIGID_MOD_SYNC_MOTION));
			pthread_mutex_unlock(&m_mutex);
			if (iRet!=0)
			{	
				std::ostringstream ssErrorMsg;
				ssErrorMsg << "Could not set ASyncMotion in Module ID: " << viModulIDs[i] << 
					", m5api error code: " << iRet;
				m_sErrorMessage = ssErrorMsg.str();
				
				return false;
			}
		}
		
		return true;
	}
	else
	{
		return false;
    }
}


bool PowerCubeCtrl::updateStates()
{	
	unsigned int iDOF;
	unsigned long ulState;
	PC_CTRL_STATUS pcStatus; 
	std::vector<std::string> vsErrorMessages;
	std::vector<int> viModulIDs;
	std::vector<double> vdOffsets;
	unsigned char ucDio;
	float fPos, fVel;
	int iRet;
	
	PCTRL_CHECK_INITIALIZED();

	// Get some parameters from the parameter object.
	viModulIDs = m_pPcParams->GetModuleIDs();
	iDOF = m_pPcParams->GetDOF();
	vdOffsets = m_pPcParams->GetOffsets();
	
	// Set some default values.
	pcStatus = PC_CTRL_ERR; 
	iRet = 0;

	for (unsigned int i = 0; i < iDOF; i++)
	{	
		// Get the state, the dio, the position and the velocity of each module.
		ulState = m_vulStatus[i]; 
		pthread_mutex_lock(&m_mutex);
		iRet = PCube_getStateDioPos(m_iDeviceHandle, viModulIDs[i], &ulState, &ucDio, &fPos);
		iRet |= PCube_getVel(m_iDeviceHandle, viModulIDs[i], &fVel);
		pthread_mutex_unlock(&m_mutex);
		
		if (iRet != 0)
		{
			ROS_DEBUG("Error on com in UpdateStates");
			return false;
		}
		else
		{	
			ROS_DEBUG("Module %i, State: %li, Time: %f",i, ulState, ros::Time::now().toSec());
			
			// Save the information.
			m_vulStatus[i] = ulState; 		
			m_vucDios[i] = ucDio;
			m_vdPositions[i] = fPos + vdOffsets[i];
			m_vdVelocities[i] = fVel;
		}		
	}

	// Evaluate state for diagnostics msgs.
	getStatus(pcStatus, vsErrorMessages);

	for (unsigned int i = 0; i < vsErrorMessages.size(); i++)
	{	
		m_sErrorMessage.clear();
		m_sErrorMessage.assign("");
		m_sErrorMessage.append(vsErrorMessages[i]);
	}

	return true;
}


bool PowerCubeCtrl::getStatus(PowerCubeCtrl::PC_CTRL_STATUS& pcStatus,
							  std::vector<std::string>& vsErrorMessages)
{	
	std::vector<PC_CTRL_STATUS> StatusArray;
	unsigned int iDOF;
	std::vector<int> viModuleIDs;

	// Get some parameters from the parameter object.
	iDOF = m_pPcParams->GetDOF();
	viModuleIDs = m_pPcParams->GetModuleIDs();
	
	// Clear the vectors.
	vsErrorMessages.clear();
	
	// Resize the vectors.
	vsErrorMessages.resize(iDOF);
	StatusArray.resize(iDOF);	

	// Set some default values.
	pcStatus = PC_CTRL_OK;

	// Get the state and a status message of each module.
	for (unsigned int i = 0; i < iDOF; i++)
	{
		std::ostringstream ssErrorMsg;
		
		if (m_vulStatus[i] & STATEID_MOD_POWERFAULT)
		{	
			if (m_vulStatus[i] & STATEID_MOD_POW_VOLT_ERR)
			{
				ssErrorMsg << "Error in Module " << viModuleIDs[i] << ": ";
				ssErrorMsg << "Motor voltage below minimum value! Check Emergency Stop!";
				vsErrorMessages[i] = ssErrorMsg.str();
			}
			else if (m_vulStatus[i] & STATEID_MOD_POW_FET_TEMP)
			{
				ssErrorMsg << "Error in Module " << viModuleIDs[i] << ": ";
				ssErrorMsg << "Overheated power transitors! Power must be switched of to reset this error.";
				vsErrorMessages[i] = ssErrorMsg.str();
			}
			else if (m_vulStatus[i] & STATEID_MOD_POW_INTEGRALERR)
			{
				ssErrorMsg << "Error in Module " << viModuleIDs[i] << ": ";
				ssErrorMsg << "The drive has been overloaded and the servo loop has been disabled. Power must be switched off to reset this error.";
				vsErrorMessages[i] = ssErrorMsg.str();
			}
			StatusArray[i] = PC_CTRL_POW_VOLT_ERR;
		}
		else if (m_vulStatus[i] & STATEID_MOD_TOW_ERROR)
		{
			ssErrorMsg << "Error in Module " << viModuleIDs[i] << ": ";
			ssErrorMsg << "The servo loop was not able to follow the target position!";
			vsErrorMessages[i] = ssErrorMsg.str();
			StatusArray[i] = PC_CTRL_ERR;
		}
		else if (m_vulStatus[i] & STATEID_MOD_ERROR)
		{	
			// STOP the motion for each module.
			pthread_mutex_lock(&m_mutex);
			PCube_haltModule(m_iDeviceHandle, m_pPcParams->GetModuleID(i));
			pthread_mutex_unlock(&m_mutex);
				
			ssErrorMsg << "Error in  Module " << viModuleIDs[i];
			ssErrorMsg << " : Status code: " << std::hex << m_vulStatus[i];
			vsErrorMessages[i] = ssErrorMsg.str();
			StatusArray[i] = PC_CTRL_ERR;
		}
		else if (m_pcStatus & PC_CTRL_ERR)
		{	
			Stop(); // stop all motion
			ssErrorMsg << "PowerCubeCtrl is in global error state";
			vsErrorMessages[i] = ssErrorMsg.str();
			StatusArray[i] =  PC_CTRL_ERR;
		}
		else
		{
			ssErrorMsg << "Module with Id " << viModuleIDs[i];
			ssErrorMsg << ": Status OK.";
			vsErrorMessages[i] = ssErrorMsg.str();
			StatusArray[i] = PC_CTRL_OK;
		}
	}

	// Search for the worst status.
	for (unsigned int i = 0; i < iDOF; i++)
	{	
		if ((int)StatusArray[i] <= (int)pcStatus)
		{
			pcStatus = StatusArray[i]; 
		}	
	}

	// Save the worst status.
	m_pcStatus = pcStatus;

	return true;
}


std::vector<double> PowerCubeCtrl::getPositions()
{
	return m_vdPositions;
}


std::vector<double> PowerCubeCtrl::getVelocities()
{
	return m_vdVelocities;
}


std::vector<double> PowerCubeCtrl::getAccelerations()
{
	return m_vdAccelerations;
}


bool PowerCubeCtrl::doHoming()
{
	unsigned int iDOF;
	std::vector<int> viModuleIDs;
	std::vector<double> vdLowerLimits, vdUpperLimits, vdOffsets;
	double dMaxHomingTime, dHomingTime, dIntervall;
	unsigned long ulState;
	unsigned char ucDio;
	float fPos;
	int iRet;
	unsigned long int uliHelp;
	
	// Get some parameters from the parameter object.
	iDOF = m_pPcParams->GetDOF();
	viModuleIDs = m_pPcParams->GetModuleIDs();
	vdLowerLimits = m_pPcParams->GetLowerLimits();
	vdUpperLimits = m_pPcParams->GetUpperLimits();
	vdOffsets = m_pPcParams->GetOffsets();

	// Set some default values.
	dMaxHomingTime = 15.0; // seconds   
	dHomingTime = 999.0; // set to 0 if any module is homed
	dIntervall = 0.1;
	ulState = PC_CTRL_ERR;
	iRet = 0;

	// Start homing.
	for (unsigned int i = 0; i < iDOF; i++)
	{	
		// Get the state, the dio and the position of each module.
		pthread_mutex_lock(&m_mutex);
		iRet = PCube_getStateDioPos(m_iDeviceHandle, viModuleIDs[i], &ulState, &ucDio, &fPos);
		pthread_mutex_unlock(&m_mutex);
			
		// Check and init m_vdPositions variable for trajectory controller.
		if ((fPos > vdUpperLimits[i] + vdOffsets[i]) || (fPos < vdLowerLimits[i] + vdOffsets[i]))
		{	
			std::ostringstream ssErrorMsg;
			ssErrorMsg << "Module " << viModuleIDs[i] << " has position " << fPos << 
				" that is outside limits (" << vdUpperLimits[i] + vdOffsets[i] << " <-> " 
				<< vdLowerLimits[i] + vdOffsets[i] << std::endl; 
			if ((m_vsModuleTypes.at(i)=="PW") || (m_vsModuleTypes.at(i) == "other"))
			{	
				std::cout << "Position error for PW-Module. Init is aborted. Try to reboot the robot." << std::endl;
				m_sErrorMessage = ssErrorMsg.str(); 
				m_pcStatus = PC_CTRL_ERR;
				
				return false;
			}
			else if (m_vsModuleTypes.at(i)=="PRL") 
			{	
				ROS_INFO("Position of Module %i is outside limits. Module can only be moved in opposite direction.",i );
				ROS_INFO("Homing for Module: %i not necessary", viModuleIDs[i]);
			} 	
			else 
			{	
				ROS_INFO("Module type incorrect. (in func. PowerCubeCtrl::doHoming();)");
				
				return false;
			}
		}
		else
		{
			m_vdPositions[i] = fPos + vdOffsets[i]; 
		}

		// Check module type before homing (PRL-Modules need not to be homed by ROS).
		if ( (m_vsModuleTypes.at(i) == "PW") || (m_vsModuleTypes.at(i) == "other") )
		{	
			// Get the state, the dio and the position of each module.
			pthread_mutex_lock(&m_mutex);
			iRet = PCube_getStateDioPos(m_iDeviceHandle, m_pPcParams->GetModuleID(i), 
										&ulState, &ucDio, &fPos);
			pthread_mutex_unlock(&m_mutex);

			if (iRet != 0)
			{	
				ROS_INFO("Error on communication with Module: %i.", m_pPcParams->GetModuleID(i)); 
				m_pcStatus = PC_CTRL_ERR;
				
				return false;
			}
				
			// Only do homing when necessary .
			if (!(ulState & STATEID_MOD_HOME))
			{
				// Homing timer.
				dHomingTime = 0.0;

				// Home the module.
				pthread_mutex_lock(&m_mutex);
				iRet = PCube_homeModule(m_iDeviceHandle, viModuleIDs[i]);
				pthread_mutex_unlock(&m_mutex);

				ROS_INFO("Homing started at: %f", ros::Time::now().toSec()); 

				if (iRet != 0)
				{	 
					ROS_INFO("Error while sending homing command to Module: %i. I try to reset the module.", i); 

					// Reset module with the hope that homing works afterwards.
					pthread_mutex_lock(&m_mutex);
					iRet = PCube_resetModule(m_iDeviceHandle, viModuleIDs[i]);
					pthread_mutex_unlock(&m_mutex);
					if (iRet != 0)
					{	
						std::ostringstream ssErrorMsg;
						ssErrorMsg << "Can't reset module after homing error" << viModuleIDs[i] << 
							", m5api error code: " << iRet;
						m_sErrorMessage = ssErrorMsg.str();
					}

					// little break for reboot
					usleep(200000); 

					// Home the module.
					pthread_mutex_lock(&m_mutex);
					iRet = PCube_homeModule(m_iDeviceHandle, viModuleIDs[i]);
					pthread_mutex_unlock(&m_mutex);
					if (iRet != 0)
					{	
						std::ostringstream ssErrorMsg;
						ssErrorMsg << "Can't start homing for module " << viModuleIDs[i] << 
							", tried reset with no success, m5api error code: " << iRet;
						m_sErrorMessage = ssErrorMsg.str();
						
						return false;				
					}
				}
			}
			else
			{
				ROS_INFO("Homing for Module: %i not necessary", m_pPcParams->GetModuleID(i));
			} 
		}
		else
		{
			ROS_INFO("Homing for PRL-Module %i not necessary", m_pPcParams->GetModuleID(i));
		} 
	}

	for (unsigned int i = 0; i < iDOF; i++)
	{	 
		do
		{
			// Get the state of the module.
			pthread_mutex_lock(&m_mutex);
			PCube_getModuleState(m_iDeviceHandle, viModuleIDs[i], &uliHelp);
			pthread_mutex_unlock(&m_mutex);
			ROS_DEBUG("Homing active for Module: %i State: %li", viModuleIDs.at(i), uliHelp);

			// Timeout watchdog for homing.
			usleep(dIntervall * 1000000);	// convert sec to usec
			dHomingTime += dIntervall; 
			if (dHomingTime >= dMaxHomingTime) {Stop(); break;} 

		} while ((uliHelp & STATEID_MOD_HOME) == 0);
		
		m_vulStatus[i] = uliHelp;
		ROS_DEBUG("State of Module %i : %li", viModuleIDs.at(i), uliHelp);
	}

	for (unsigned int i = 0; i < iDOF; i++)
	{	
		// Check the result.
		if (!(m_vulStatus[i] & STATEID_MOD_HOME) || (m_vulStatus[i] & STATEID_MOD_ERROR) )
		{
			std::cout << "Homing failed: Error in  Module " << viModuleIDs[i] << std::endl;
			m_pcStatus = PC_CTRL_NOT_HOMED;
			return false;
		}
		
		ROS_INFO("Homing for Modul %i done.", viModuleIDs.at(i)); 
	}

	// Modules successfully homed.
	m_pcStatus = PC_CTRL_OK;
	
	return true;
}


bool PowerCubeCtrl::isInitialized() const
{
	return m_bInitialized;
}


std::string PowerCubeCtrl::getErrorMessage() const
{
	return m_sErrorMessage;
}


PowerCubeCtrl::PC_CTRL_STATUS PowerCubeCtrl::getPC_Status() const
{
	return m_pcStatus;
}


bool PowerCubeCtrl::PCTRL_CHECK_INITIALIZED()
{
	if (isInitialized() == false)
	{
		m_sErrorMessage.assign("Manipulator not initialized.");
		
		return false;
	}
	
	return true;
}
