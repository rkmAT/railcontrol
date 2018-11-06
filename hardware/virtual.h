#pragma once

#include <cstring>

#include "hardware/HardwareInterface.h"
#include "hardware/HardwareParams.h"

namespace hardware
{

	class Virtual : HardwareInterface
	{
		public:
			// Constructor
			Virtual(const HardwareParams* params);

			// name() must be implemented
			const std::string GetName() const override { return name; };

			// get available protocols of this control
			void GetProtocols(std::vector<protocol_t>& protocols) const override;

			// is given protocol supported
			bool ProtocolSupported(protocol_t protocol) const override;

			// turn booster on or off
			void Booster(const boosterStatus_t status) override;

			// set loco speed
			void LocoSpeed(const protocol_t& protocol, const address_t& address, const speed_t& speed) override;

			// set loco direction
			void LocoDirection(const protocol_t& protocol, const address_t& address, const direction_t& direction) override;

			// set loco function
			void LocoFunction(const protocol_t protocol, const address_t address, const function_t function, const bool on) override;

			// accessory command
			void Accessory(const protocol_t protocol, const address_t address, const accessoryState_t state, const bool on) override;

		private:
			std::string name;
	};

} // namespace

