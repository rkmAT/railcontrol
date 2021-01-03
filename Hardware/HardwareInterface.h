/*
RailControl - Model Railway Control Software

Copyright (c) 2017-2020 Dominik (Teddy) Mahrer - www.railcontrol.org

RailControl is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 3, or (at your option) any
later version.

RailControl is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with RailControl; see the file LICENCE. If not see
<http://www.gnu.org/licenses/>.
*/

#pragma once

#include <map>
#include <string>
#include <vector>

#include "DataModel/AccessoryBase.h"
#include "DataTypes.h"
#include "Hardware/Capabilities.h"
#include "Manager.h"
#include "Utils/Utils.h"

namespace Hardware
{

	class HardwareInterface
	{
		public:
			// non virtual default constructor is needed to prevent polymorphism
			HardwareInterface(Manager* manager, const ControlID controlID, const std::string& name)
			:	manager(manager),
			 	controlID(controlID),
			 	name(name)
			{};

			// pure virtual destructor prevents polymorphism in derived class
			virtual ~HardwareInterface() {};

			// get the name of the hardware
			const std::string GetName() const { return name; }

			// get hardware capabilities
			virtual Hardware::Capabilities GetCapabilities() const = 0;

			// get available loco protocols of this control
			virtual void GetLocoProtocols(__attribute__((unused)) std::vector<Protocol>& protocols) const {};

			// is given loco protocol supported
			virtual bool LocoProtocolSupported(__attribute__((unused)) const Protocol protocol) const { return false; }

			// get available accessory protocols of this control
			virtual void GetAccessoryProtocols(__attribute__((unused)) std::vector<Protocol>& protocols) const {}

			// is given accessory protocol supported
			virtual bool AccessoryProtocolSupported(__attribute__((unused)) const Protocol protocol) const { return false; }

			// turn booster on or off
			virtual void Booster(__attribute__((unused)) const BoosterState status) {};

			// set loco speed
			virtual void LocoSpeed(__attribute__((unused)) const Protocol protocol, __attribute__((unused)) const Address address, __attribute__((unused)) const Speed speed) {};

			// set loco orientation
			virtual void LocoOrientation(__attribute__((unused)) const Protocol protocol, __attribute__((unused)) const Address address, __attribute__((unused)) const Orientation orientation) {};

			// set loco function
			virtual void LocoFunction(__attribute__((unused)) const Protocol protocol,
				__attribute__((unused)) const Address address,
				__attribute__((unused)) const DataModel::LocoFunctionNr function,
				__attribute__((unused)) const DataModel::LocoFunctionState on)
			{}

			// set loco
			virtual void LocoSpeedOrientationFunctions(const Protocol protocol,
				const Address address,
				const Speed speed,
				const Orientation orientation,
				std::vector<DataModel::LocoFunctionEntry>& functions)
			{
				// sleeps are necessary to prevent command overflow in command stations (especially Märklin Gleisbox)
				LocoSpeed(protocol, address, speed);
				Utils::Utils::SleepForMilliseconds(25);
				LocoOrientation(protocol, address, orientation);
				Utils::Utils::SleepForMilliseconds(25);
				for (const DataModel::LocoFunctionEntry& functionEntry : functions)
				{
					LocoFunction(protocol, address, functionEntry.nr, functionEntry.state);
					Utils::Utils::SleepForMilliseconds(25);
				}
			}

			// accessory command
			virtual void Accessory(const Protocol protocol, const AccessoryGroup group, const Address address, const DataModel::AccessoryState state, const DataModel::AccessoryPulseDuration duration)
			{
				AccessoryOnOrOff(protocol, group, address, state, true);
				std::async(std::launch::async, AccessoryOnOrOffStatic, this, protocol, group, address, state, duration);
			};

			// read CV value
			virtual void ProgramRead(__attribute__((unused)) const ProgramMode mode, __attribute__((unused)) const Address address, __attribute__((unused)) const CvNumber cv) {}

			// write CV value
			virtual void ProgramWrite(__attribute__((unused)) const ProgramMode mode, __attribute__((unused)) const Address address, __attribute__((unused)) const CvNumber cv, __attribute__((unused)) const CvValue value) {}

		protected:
			Manager* manager;
			const ControlID controlID;
			const std::string name;

			virtual void AccessoryOnOrOff(__attribute__((unused)) const Protocol protocol, __attribute__((unused)) const AccessoryGroup group, __attribute__((unused)) const Address address, __attribute__((unused)) const DataModel::AccessoryState state, __attribute__((unused)) const bool on) {}

		private:
			static void AccessoryOnOrOffStatic(HardwareInterface* hardware, const Protocol protocol, const AccessoryGroup group, const Address address, const DataModel::AccessoryState state, const DataModel::AccessoryPulseDuration duration)
			{
				Utils::Utils::SleepForMilliseconds(duration);
				hardware->AccessoryOnOrOff(protocol, group, address, state, false);
			}
	};

} // namespace

