/*
RailControl - Model Railway Control Software

Copyright (c) 2020 Maik Ranke - www.railcontrol.org

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

#include "HardwareInterface.h"
#include "HardwareParams.h"
#include "Logger/Logger.h"
#include "Network/TcpClient.h"

namespace Hardware
{
	class DCCppLocoCache
	{
		public:

			void SetSpeed(const Address address, const Speed speed)
			{
				m_xLocoCache[address].speed = speed;
			}

			Speed GetSpeed(const Address address)
			{
				itcLocoCache it = m_xLocoCache.find(address);
				if (it != m_xLocoCache.end())
				{
					return it->second.speed;
				}
				return 0;
			}

			void SetOrientation(const Address address, const Orientation orientation)
			{
				m_xLocoCache[address].orientation = orientation;
			}

			Orientation GetOrientation(const Address address)
			{
				itcLocoCache it = m_xLocoCache.find(address);
				if (it != m_xLocoCache.end())
				{
					return it->second.orientation;
				}
				return OrientationRight;
			}


		private:
			struct loco
			{
				Speed speed = 0;
				Orientation orientation = OrientationRight;
			};
			std::map<Address, loco> m_xLocoCache;
			typedef std::map<Address, loco>::const_iterator itcLocoCache;
	};

	class DCCpp : protected HardwareInterface
	{
		public:
			DCCpp(const HardwareParams* params);
			~DCCpp();

			inline Hardware::Capabilities GetCapabilities() const override
			{
				return Hardware::CapabilityLoco
					| Hardware::CapabilityAccessory
					| Hardware::CapabilityFeedback
					| Hardware::CapabilityProgram
					| Hardware::CapabilityProgramDccRegisterRead
					| Hardware::CapabilityProgramDccRegisterWrite
					| Hardware::CapabilityProgramDccDirectRead
					| Hardware::CapabilityProgramDccDirectWrite
					| Hardware::CapabilityProgramDccPomRead
					| Hardware::CapabilityProgramDccPomWrite;
			}

			void GetLocoProtocols(std::vector<Protocol>& protocols) const override
			{
				protocols.push_back(ProtocolDCC14);
				protocols.push_back(ProtocolDCC28);
				protocols.push_back(ProtocolDCC128);
			}

			bool LocoProtocolSupported(Protocol protocol) const override
			{
				return (protocol == ProtocolDCC14) || (protocol == ProtocolDCC28) || (protocol == ProtocolDCC128);
			}

			void GetAccessoryProtocols(std::vector<Protocol>& protocols) const override
			{
				protocols.push_back(ProtocolDCC);
				protocols.push_back(ProtocolCsInternalDefined);
				protocols.push_back(ProtocolDirectIO);
			}

			bool AccessoryProtocolSupported(Protocol protocol) const override
			{
				return (protocol == ProtocolDCC) || (protocol == ProtocolCsInternalDefined) || (protocol == ProtocolDirectIO);
			}

			static void GetArgumentTypesAndHint(std::map<unsigned char,ArgumentType>& argumentTypes, std::string& hint)
			{
				argumentTypes[1] = ArgumentTypeIpAddress;
				argumentTypes[2] = ArgumentTypeIPPort;
				argumentTypes[3] = ArgumentTypeSerialPort;
				hint = Languages::GetText(Languages::TextHintDccPP);
			}

			void Booster(const BoosterState status) override;

			void LocoSpeed(const Protocol protocol, const Address address, const Speed speed) override;
			void LocoOrientation(const Protocol protocol, const Address address, const Orientation orientation) override;

			void LocoFunction(const Protocol protocol,
				const Address address,
				const DataModel::LocoFunctionNr function,
				const DataModel::LocoFunctionState on) override;

			void LocoSpeedOrientationFunctions(const Protocol protocol,
				const Address address,
				const Speed speed,
				const Orientation orientation,
				std::vector<DataModel::LocoFunctionEntry>& functions) override;

			void AccessoryOnOrOff(const Protocol protocol, const AccessoryGroup group, const Address address, const DataModel::AccessoryState state, const bool on) override;

		private:
			static const unsigned short DCCppDefaultPort = 2560;
			int getPort(const HardwareParams* params);
			void Send(const char* data);
			void Receiver();
			void Heartbeat();

			void Parser(const char* cBuffer, const int iNumChars);

			Logger::Logger* logger;
			volatile bool run;
			volatile bool runHeartbeat;
			std::thread receiverThread;
			std::thread heartbeatThread;

			Network::TcpConnection tcp;
			const HardwareParams* m_params; // save for reconnect
			DataModel::AccessoryState invertAccessoryState(const DataModel::AccessoryState state);
			void LocoSpeedOrientation(const Protocol protocol, const Address address, const Speed speed, const Orientation orientation);
			Speed EncodeSpeed14(const Speed speed);
			Speed EncodeSpeed28(const Speed speed);
			Speed EncodeSpeed128(const Speed speed);


			DCCppLocoCache m_xLoco;
	};

	extern "C" DCCpp* create_DCCpp(const HardwareParams* params);
	extern "C" void destroy_DCCpp(DCCpp* dccpp);
} // namespace
