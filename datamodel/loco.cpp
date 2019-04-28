#include <algorithm>
#include <map>
#include <sstream>

#include "datamodel/loco.h"
#include "datamodel/track.h"
#include "manager.h"
#include "util.h"

using std::map;
using std::stringstream;
using std::string;
using std::vector;

namespace datamodel
{
	Loco::~Loco()
	{
		while (true)
		{
			{
				std::lock_guard<std::mutex> Guard(stateMutex);
				if (state == LocoStateManual)
				{
					return;
				}
			}
			logger->Info("Waiting until {0} has stopped", name);
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}

	std::string Loco::Serialize() const
	{
		trackID_t trackIdFrom = TrackNone;
		if (trackFrom != nullptr)
		{
			trackIdFrom = trackFrom->GetID();
		}
		stringstream ss;
		ss << "objectType=Loco"
			<< ";" << Object::Serialize()
			<< ";" << HardwareHandle::Serialize()
			<< ";functions=" << functions.Serialize()
			<< ";direction=" << (direction == DirectionRight ? "right" : "left")
			<< ";trackID=" << static_cast<int>(trackIdFrom)
			<< ";length=" << length
			<< ";commuter=" << static_cast<int>(commuter)
			<< ";maxspeed=" << maxSpeed
			<< ";travelspeed=" << travelSpeed
			<< ";reducedspeed=" << reducedSpeed
			<< ";creepspeed=" << creepSpeed;
		return ss.str();
	}

	bool Loco::Deserialize(const std::string& serialized)
	{
		map<string,string> arguments;
		ParseArguments(serialized, arguments);
		Object::Deserialize(arguments);
		if (!arguments.count("objectType") || arguments.at("objectType").compare("Loco") != 0)
		{
			return false;
		}
		HardwareHandle::Deserialize(arguments);
		trackID_t trackIdFrom = GetIntegerMapEntry(arguments, "trackID", TrackNone);
		trackFrom = manager->GetTrack(trackIdFrom);
		functions.Deserialize(GetStringMapEntry(arguments, "functions", "0"));
		direction = (GetStringMapEntry(arguments, "direction", "right").compare("right") == 0 ? DirectionRight : DirectionLeft);
		length = static_cast<length_t>(GetIntegerMapEntry(arguments, "length", 0));
		commuter = GetBoolMapEntry(arguments, "commuter", false);
		maxSpeed = GetIntegerMapEntry(arguments, "maxspeed", MaxSpeed);
		travelSpeed = GetIntegerMapEntry(arguments, "travelspeed", DefaultTravelSpeed);
		reducedSpeed = GetIntegerMapEntry(arguments, "reducedspeed", DefaultReducedSpeed);
		creepSpeed = GetIntegerMapEntry(arguments, "creepspeed", DefaultCreepSpeed);
		return true;
	}

	bool Loco::ToTrack(const trackID_t trackID)
	{
		std::lock_guard<std::mutex> Guard(stateMutex);
		// there must not be set a track
		if (this->trackFrom != nullptr)
		{
			return false;
		}
		this->trackFrom = manager->GetTrack(trackID);
		return true;
	}

	bool Loco::Release()
	{
		std::lock_guard<std::mutex> Guard(stateMutex);
		if (state != LocoStateManual)
		{
			state = LocoStateOff;
		}

		if (streetFirst != nullptr)
		{
			streetFirst->Release(objectID);
			streetFirst = nullptr;
		}
		if (streetSecond != nullptr)
		{
			streetSecond->Release(objectID);
			streetSecond = nullptr;
		}
		if (trackFrom != nullptr)
		{
			trackFrom->Release(objectID);
			trackFrom = nullptr;
		}
		if (trackFirst != nullptr)
		{
			trackFirst->Release(objectID);
			trackFirst = nullptr;
		}
		if (trackSecond != nullptr)
		{
			trackSecond->Release(objectID);
			trackSecond = nullptr;
		}
		feedbackIdOver = FeedbackNone;
		feedbackIdStop = FeedbackNone;
		feedbackIdCreep = FeedbackNone;
		feedbackIdReduced = FeedbackNone;
		feedbackIdFirst = FeedbackNone;
		return true;
	}

	bool Loco::GoToAutoMode()
	{
		std::lock_guard<std::mutex> Guard(stateMutex);
		if (trackFrom == nullptr)
		{
			logger->Warning("Can not start {0} because it is not on a track", name);
			return false;
		}
		if (state == LocoStateError)
		{
			logger->Warning("Can not start {0} because it is in error state", name);
			return false;
		}
		if (state == LocoStateOff)
		{
			locoThread.join();
			state = LocoStateManual;
		}
		if (state != LocoStateManual)
		{
			logger->Info("Can not start {0} because it is already running", name);
			return false;
		}

		state = LocoStateSearchingFirst;
		locoThread = std::thread(&datamodel::Loco::AutoMode, this);

		return true;
	}

	bool Loco::GoToManualMode()
	{
		{
			std::lock_guard<std::mutex> Guard(stateMutex);
			switch (state)
			{
				case LocoStateManual:
					manager->LocoSpeed(ControlTypeInternal, objectID, 0);
					return true;

				case LocoStateSearchingFirst:
				case LocoStateOff:
				case LocoStateError:
					state = LocoStateOff;
					break;

				case LocoStateSearchingSecond:
				case LocoStateRunning:
				case LocoStateStopping:
					logger->Info("{0} is actually running, waiting until reached its destination", name);
					state = LocoStateStopping;
					return false;

				default:
					logger->Error("{0} is in unknown state. Setting to error state and setting speed to 0.", name);
					state = LocoStateError;
					manager->LocoSpeed(ControlTypeInternal, objectID, 0);
					return false;
			}
		}
		locoThread.join();
		state = LocoStateManual;
		return true;
	}

	void Loco::SetMinThreadPriorityAndThreadName()
	{
		sched_param param;
		int policy;
		pthread_t self = pthread_self();
		pthread_getschedparam(self, &policy, &param);
		param.__sched_priority = sched_get_priority_min(policy);
		pthread_setschedparam(self, policy, &param);
		pthread_setname_np(self, name.c_str());
	}

	void Loco::AutoMode()
	{
		SetMinThreadPriorityAndThreadName();
		const string& name = GetName();
		logger->Info("{0} is now in automode", name);
		feedbackIdFirstReached = false;
		feedbackIdStopReached = false;

		while (true)
		{
			{ // sleep is outside the lock
				std::lock_guard<std::mutex> Guard(stateMutex);
				if (feedbackIdFirstReached == true)
				{
					FeedbackIdFirstReached();
				}
				if (feedbackIdStopReached == true)
				{
					FeedbackIdStopReached();
				}
				switch (state)
				{
					case LocoStateOff:
						// automode is turned off, terminate thread
						logger->Info("{0} is now in manual mode", name);
						return;

					case LocoStateSearchingFirst:
						SearchDestinationFirst();
						break;

					case LocoStateSearchingSecond:
						if (manager->GetNrOfTracksToReserve() > 1)
						{
							SearchDestinationSecond();
						}
						break;

					case LocoStateRunning:
						// loco is already running, waiting until destination reached
						break;

					case LocoStateStopping:
						logger->Info("{0} has not yet reached its destination. Going to manual mode when it reached its destination.", name);
						break;

					case LocoStateManual:
						logger->Error("{0} is in manual state while automode is running. Putting loco into error state", name);
						state = LocoStateError;
						[[fallthrough]];

					case LocoStateError:
						logger->Error("{0} is in error state.", name);
						manager->LocoSpeed(ControlTypeInternal, objectID, 0);
						break;
				}
			}
			// FIXME: make configurable
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}

	void Loco::SearchDestinationFirst()
	{
		// check if already running
		if (streetFirst != nullptr || streetSecond != nullptr)
		{
			state = LocoStateError;
			logger->Error("{0} has already a street reserved. Going to error state.", name);
			return;
		}

		if (trackFrom == nullptr)
		{
			state = LocoStateOff;
			logger->Info("{0} is not on a track. Switching to manual mode.", name);
			return;
		}

		if (trackFrom->GetLoco() != objectID)
		{
			state = LocoStateError;
			logger->Error("{0} thinks it is on track {1} but there is {2}. Going to error state.", name, trackFrom->GetName(), manager->GetLocoName(trackFrom->GetLoco()));
			return;
		}
		logger->Debug("Looking for new destination starting from {0}.", trackFrom->GetName());

		Street* usedStreet = SearchDestination(trackFrom, true);
		if (usedStreet == nullptr)
		{
			logger->Debug("No valid street found for {0}", name);
			return;
		}


		trackID_t newTrackIdFirst = usedStreet->GetToTrack();
		Track* newTrack = manager->GetTrack(newTrackIdFirst);
		if (newTrack == nullptr)
		{
			return;
		}

		trackFirst = newTrack;
		streetFirst = usedStreet;
		feedbackIdFirst = FeedbackNone;
		feedbackIdReduced = streetFirst->GetFeedbackIdReduced();
		feedbackIdCreep = streetFirst->GetFeedbackIdCreep();
		feedbackIdStop = streetFirst->GetFeedbackIdStop();
		feedbackIdOver = streetFirst->GetFeedbackIdOver();
		bool turnLoco = (trackFrom->GetLocoDirection() != streetFirst->GetFromDirection());
		direction_t newLocoDirection = static_cast<direction_t>(direction != turnLoco);
		if (turnLoco)
		{
			trackFrom->SetLocoDirection(streetFirst->GetFromDirection());
			manager->TrackPublishState(trackFrom);
		}
		manager->LocoDirection(ControlTypeInternal, this, newLocoDirection);
		newTrack->SetLocoDirection(static_cast<direction_t>(!streetFirst->GetToDirection()));
		logger->Info("Heading to {0} via {1}", newTrack->GetName(), streetFirst->GetName());

		// start loco
		manager->TrackPublishState(newTrack);
		manager->LocoSpeed(ControlTypeInternal, objectID, travelSpeed);
		state = LocoStateSearchingSecond;
	}

	void Loco::SearchDestinationSecond()
	{
		// check if already running
		if (streetSecond != nullptr)
		{
			state = LocoStateError;
			logger->Error("{0} has already a street reserved. Going to error state.", name);
			return;
		}

		if (trackFirst == nullptr)
		{
			state = LocoStateOff;
			logger->Info("{0} is not on a track. Switching to manual mode.", name);
			return;
		}

		if (trackFirst->GetLoco() != objectID)
		{
			state = LocoStateError;
			logger->Error("{0} thinks it is on track {1} but there is {2}. Going to error state.", name, trackFirst->GetName(), manager->GetLocoName(trackFirst->GetLoco()));
			return;
		}
		logger->Debug("Looking for new destination starting from {0}.", trackFirst->GetName());

		Street* usedStreet = SearchDestination(trackFirst, false);
		if (usedStreet == nullptr)
		{
			logger->Debug("No valid street found for {0}", name);
			return;
		}
		// FIXME: replace with code in SearchDestination
		bool turnLoco = (trackFirst->GetLocoDirection() != usedStreet->GetFromDirection());
		if (turnLoco)
		{
			return;
		}

		trackID_t newTrackIdSecond = usedStreet->GetToTrack();
		Track* newTrack = manager->GetTrack(newTrackIdSecond);
		if (newTrack == nullptr)
		{
			return;
		}

		trackSecond = newTrack;
		streetSecond = usedStreet;
		feedbackIdFirst = feedbackIdStop;
		feedbackIdOver = streetSecond->GetFeedbackIdOver();
		feedbackIdStop = streetSecond->GetFeedbackIdStop();
		feedbackIdCreep = streetSecond->GetFeedbackIdCreep();
		feedbackIdReduced = streetSecond->GetFeedbackIdReduced();
		newTrack->SetLocoDirection(static_cast<direction_t>(!streetSecond->GetToDirection()));
		logger->Info("Heading to {0} via {1}", newTrack->GetName(), streetSecond->GetName());

		// start loco
		manager->TrackPublishState(newTrack);
		manager->LocoSpeed(ControlTypeInternal, objectID, travelSpeed);
		state = LocoStateRunning;
	}

	Street* Loco::SearchDestination(Track* track, const bool allowLocoTurn)
	{
		vector<Street*> validStreets;
		track->GetValidStreets(this, allowLocoTurn, validStreets);
		for (auto street : validStreets)
		{
			if (street->Reserve(objectID) == false)
			{
				continue;
			}

			if (street->Lock(objectID) == false)
			{
				street->Release(objectID);
				continue;
			}

			if (street->Execute() == false)
			{
				street->Release(objectID);
				continue;
			}
			return street;
		}
		return nullptr;
	}

	void Loco::LocationReached(const feedbackID_t feedbackID)
	{
		if (feedbackID == feedbackIdOver)
		{
			manager->LocoSpeed(ControlTypeInternal, this, MinSpeed);
			manager->Booster(ControlTypeInternal, BoosterStop);
			logger->Error("{0} hit overrun feedback {1}", name, manager->GetFeedbackName(feedbackID));
			return;
		}

		if (feedbackID == feedbackIdStop)
		{
			manager->LocoSpeed(ControlTypeInternal, this, MinSpeed);
			feedbackIdStopReached = true;
			return;
		}

		if (feedbackID == feedbackIdCreep)
		{
			if (speed > creepSpeed)
			{
				manager->LocoSpeed(ControlTypeInternal, this, creepSpeed);
			}
			return;
		}

		if (feedbackID == feedbackIdReduced)
		{
			if (speed > reducedSpeed)
			{
				manager->LocoSpeed(ControlTypeInternal, this, reducedSpeed);
			}
			return;
		}

		if (feedbackID == feedbackIdFirst)
		{
			feedbackIdFirstReached = true;
			return;
		}
	}

	void Loco::FeedbackIdFirstReached()
	{
		feedbackIdFirstReached = false;
		// set loco to new track
		if (streetFirst == nullptr || trackFrom == nullptr)
		{
			Speed(MinSpeed);
			state = LocoStateError;
			logger->Error("{0} is running in automode without a street / track. Putting loco into error state", name);
			return;
		}

		streetFirst->Release(objectID);
		streetFirst = streetSecond;
		streetSecond = nullptr;

		trackFrom->Release(objectID);
		trackFrom = trackFirst;
		trackFirst = trackSecond;
		trackSecond = nullptr;

		feedbackIdFirst = FeedbackNone;

		// set state
		switch (state)
		{
			case LocoStateRunning:
				state = LocoStateSearchingSecond;
				return;

			case LocoStateStopping:
				// do nothing
				return;

			default:
				state = LocoStateError;
				logger->Error("{0} is running in impossible automode state. Putting loco into error state", name);
				return;
		}
	}

	void Loco::FeedbackIdStopReached()
	{
		feedbackIdStopReached = false;
		// set loco to new track
		if (streetFirst == nullptr || trackFrom == nullptr)
		{
			Speed(MinSpeed);
			state = LocoStateError;
			logger->Error("{0} is running in automode without a street / track. Putting loco into error state", name);
			return;
		}

		manager->LocoDestinationReached(objectID, streetFirst->GetID(), trackFrom->GetID());
		streetFirst->Release(objectID);
		streetFirst = nullptr;

		trackFrom->Release(objectID);
		trackFrom = trackFirst;
		trackFirst = nullptr;

		feedbackIdStop = FeedbackNone;
		feedbackIdCreep = FeedbackNone;
		feedbackIdReduced = FeedbackNone;
		logger->Info("{0} reached its destination", name);
		// set state
		switch (state)
		{
			case LocoStateSearchingSecond:
				state = LocoStateSearchingFirst;
				return;

			case LocoStateStopping:
				state = LocoStateOff;
				return;

			default:
				state = LocoStateError;
				logger->Error("{0} is running in impossible automode state. Putting loco into error state", name);
				return;
		}
	}

	const char* Loco::GetStateText() const
	{
		switch (state)
		{
			case LocoStateManual:
				return "manual";

			case LocoStateOff:
				return "off";

			case LocoStateSearchingFirst:
			case LocoStateSearchingSecond:
				return "searching";

			case LocoStateRunning:
				return "running";

			case LocoStateStopping:
				return "stopping";

			case LocoStateError:
				return "error";

			default:
				return "unknown";
		}
	}
} // namespace datamodel

