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

#include <deque>
#include <cstring>

#include "DataModel/AccessoryBase.h"
#include "Hardware/DCCpp.h"
#include "Utils/Utils.h"

using std::deque;
using std::string;
using std::strlen;
using std::to_string;

namespace Hardware
{
	extern "C" DCCpp* create_DCCpp(const HardwareParams* params)
	{
		return new DCCpp(params);
	}


	extern "C" void destroy_DCCpp(DCCpp* dccpp)
	{
		delete(dccpp);
	}


	DCCpp::DCCpp(const HardwareParams* params)
	:	HardwareInterface(params->GetManager(),
			params->GetControlID(),
			"DCC++ / " + params->GetName() + " at IP " + params->GetArg1() + ":" + params->GetArg2()),
		logger(Logger::Logger::GetLogger("DCC++ " + params->GetName() + " " + params->GetArg1() + ":" + params->GetArg2())),
	 	run(false),
		runHeartbeat(false),
 	 	tcp(Network::TcpClient::GetTcpClientConnection(logger, params->GetArg1(), getPort(params))),
		m_params(params)  
	{
		logger->Info(Languages::TextStarting, name);
		if (!tcp.IsConnected())
		{
			return;
		}
		receiverThread = std::thread(&Hardware::DCCpp::Receiver, this);
		heartbeatThread = std::thread(&Hardware::DCCpp::Heartbeat, this);
	}


	DCCpp::~DCCpp()
	{
		if (run)
		{
			run = false;
			receiverThread.join();
		}
		if(runHeartbeat)
		{
			runHeartbeat = false;
			heartbeatThread.join();
		}
		logger->Info(Languages::TextTerminatingSenderSocket);
	}


	// turn booster on or off
	void DCCpp::Booster(const BoosterState status)
	{
		logger->Info(status ? Languages::TextTurningBoosterOn : Languages::TextTurningBoosterOff);
		std::string sCommand = status == BoosterStateGo ? "<1>" : "<0>";
		Send(sCommand.c_str());
	}


	void DCCpp::LocoSpeed(const Protocol protocol, const Address address, const Speed speed)
	{
		if (!LocoProtocolSupported(protocol))
		{
			return;
		}
		Orientation orientation = m_xLoco.GetOrientation(address);
		m_xLoco.SetSpeed(address, speed);
		LocoSpeedOrientation(protocol, address, speed, orientation);
	}


	void DCCpp::LocoOrientation(const Protocol protocol, const Address address, const Orientation orientation)
	{
		if (!LocoProtocolSupported(protocol))
		{
			return;
		}

		Speed speed = m_xLoco.GetSpeed(address);
		m_xLoco.SetOrientation(address, orientation);
		LocoSpeedOrientation(protocol, address, speed, orientation);
	}


	void DCCpp::LocoSpeedOrientation(const Protocol protocol, const Address address, const Speed speed, const Orientation orientation)
	{
		Speed internalSpeed = 0;
		switch (protocol)
		{
			case ProtocolDCC14:
				internalSpeed = EncodeSpeed14(speed);
				break;

			case ProtocolDCC28:
				internalSpeed = EncodeSpeed28(speed);
				break;

			case ProtocolDCC128:
				internalSpeed = EncodeSpeed128(speed);
				break;

			default:
				return;
		}
		std::string sCommand = "<t 1 " + std::to_string(address) + " " + std::to_string(internalSpeed) + " " + std::to_string(static_cast<int>(orientation)) + ">";
		Send(sCommand.c_str());
	}


	Speed DCCpp::EncodeSpeed14(const Speed speed)
	{
		Speed speedInternal = (speed >> 6);
		speedInternal = speedInternal > 14 ? 14 : speedInternal;
		return speedInternal;
	}


	Speed DCCpp::EncodeSpeed28(const Speed speed)
	{
		Speed speedInternal = (speed >> 5) - 4 ;
		speedInternal = speedInternal > 28 ? 28 : speedInternal;
		return speedInternal;
	}


	Speed DCCpp::EncodeSpeed128(const Speed speed)
	{
		Speed speedInternal = speed >> 3;
		// DCC++ range is 0..126 so check here
		speedInternal = speedInternal > 126 ? 126 : speedInternal;
		return speedInternal;
	}


	void DCCpp::LocoFunction(__attribute__ ((unused)) const Protocol protocol,
		const Address address,
		const DataModel::LocoFunctionNr function,
		const DataModel::LocoFunctionState on)
	{
		if (!LocoProtocolSupported(protocol))
		{
			return;
		}
		logger->Info(Languages::TextSettingFunctionWithProtocol, static_cast<int>(function), static_cast<int>(protocol), address, Languages::GetOnOff(on));
		std::string sCommand = "<F " + std::to_string(address) + " " + std::to_string(function) + " " + std::to_string(static_cast<int>(on)) + ">";
		Send(sCommand.c_str());
	}


	void DCCpp::LocoSpeedOrientationFunctions(const Protocol protocol,
		const Address address,
		const Speed speed,
		const Orientation orientation,
		std::vector<DataModel::LocoFunctionEntry>& functions)
	{
		m_xLoco.SetSpeed(address, speed);
		m_xLoco.SetOrientation(address, orientation);
		LocoSpeedOrientation(protocol, address, speed, orientation);

		for (const DataModel::LocoFunctionEntry& functionEntry : functions)
		{
			LocoFunction(protocol, address, functionEntry.nr, functionEntry.state);
			Utils::Utils::SleepForMilliseconds(25);
		}
	}


	void DCCpp::AccessoryOnOrOff(const Protocol protocol, const AccessoryGroup group, const Address address, const DataModel::AccessoryState state, const bool on)
	{
		DataModel::AccessoryState invertState = invertAccessoryState(state); // In DCC++ 0: unthrown(straight) 1: thrown(turned)
		logger->Info(Languages::TextSettingAccessoryWithProtocol, static_cast<int>(protocol), address, Languages::GetGreenRed(invertState), Languages::GetOnOff(on));
		if(protocol == ProtocolDCC)
		{
			u_int16_t uAddress = address / 4;
			u_int16_t uSubAddress = address % 4;
			std::string sCommand = "<a " + std::to_string(uAddress) + " " + std::to_string(uSubAddress) + " " + std::to_string(static_cast<int>(invertState)) + ">";
			Send(sCommand.c_str());
		}
		else if(protocol == ProtocolCsInternalDefined)
		{
			if((group == AccessoryGroupSwitch) && (on))
			{
				std::string sCommand = "<T " + std::to_string(address) + " " + std::to_string(static_cast<int>(invertState)) + ">";
				Send(sCommand.c_str());
			}
			else if(((group == AccessoryGroupSignal) || (group == AccessoryGroupCommon)) && (on))
			{
				std::string sCommand = "<Z " + std::to_string(address) + " " + std::to_string(static_cast<int>(invertState)) + ">";
				Send(sCommand.c_str());
			}
		}
		else if(protocol == ProtocolDirectIO)
		{

		}
	}

	void DCCpp::Send(const char* data)
	{
		if(strlen(data) == 0)
		{
			return;
		}
		int ret = tcp.Send(data);
		if (ret < 0)
		{
			logger->Error(Languages::TextUnableToSendDataToControl);
		}
	}


	void DCCpp::Heartbeat()
	{
		Utils::Utils::SetThreadName("DCCppHeartbeat");
		logger->Info(Languages::TextReceiverThreadHeartbeat);
		runHeartbeat = true;
		while(runHeartbeat)
		{
			Utils::Utils::SleepForSeconds(10);
			Send("<s>");
		}
	}


	void DCCpp::Receiver()
	{
		Utils::Utils::SetThreadName("DCCppReceiver");
		logger->Info(Languages::TextReceiverThreadStarted);
		char cBuffer[1024];
		run = true;
		while(run)
		{
			if (run == false)
			{
				break;
			}
			int iDataLength = tcp.Receive(cBuffer, sizeof(cBuffer));
			if (iDataLength < 0)
			{
				if (errno == ETIMEDOUT)
				{
					continue;
				}
				else
				{
					logger->Error(Languages::TextConnectionLost, errno);
					break;
				}
			}
			else if(iDataLength > 0)
			{
				logger->Info(Languages::TextVersion, cBuffer);
				Parser(cBuffer, iDataLength);	
			}
		}
		logger->Hex(cBuffer);
		tcp.Terminate();
		logger->Info(Languages::TextTerminatingReceiverThread);
	}


	void DCCpp::Parser(const char* cBuffer, int iNumChars)
	{
		if (cBuffer[0] != '<')	// DCC++ start char
		{
			logger->Error(Languages::TextInvalidDataReceived);
			return;
		}
		if (iNumChars > 1)
		{
			switch (cBuffer[1])
			{
				case '0':
					return;

				default:
					return;
			}
		}
	}

	int DCCpp::getPort(const HardwareParams* params)
	{
		u_int16_t uPort = 0;
		try
		{
			uPort = std::stoi(params->GetArg2(), nullptr, 10);
		}
		catch(const std::exception& e)
		{
			uPort = DCCppDefaultPort;
		}
		return uPort;
	}

	DataModel::AccessoryState DCCpp::invertAccessoryState(const DataModel::AccessoryState state)
	{
		return state == DataModel::AccessoryState::AccessoryStateOn ? DataModel::AccessoryState::AccessoryStateOff : DataModel::AccessoryState::AccessoryStateOn;
	}

} // namespace
