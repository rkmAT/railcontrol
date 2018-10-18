#pragma once

#include "datatypes.h"

class Manager;

namespace hardware
{

	class HardwareParams
	{
		public:
			HardwareParams(controlID_t controlID, hardwareType_t hardwareType, std::string name, std::string ip);
			HardwareParams(Manager* manager, controlID_t controlID, hardwareType_t hardwareType, std::string name, std::string ip);
			Manager* manager;
			controlID_t controlID;
			hardwareType_t hardwareType;
			std::string name;
			std::string ip;
	};

} // namespace hardware
