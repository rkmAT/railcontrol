#include <map>
#include <sstream>
#include <string>

#include "relation.h"

using std::map;
using std::stringstream;
using std::string;

namespace datamodel {

	Relation::Relation(Manager* const manager,
		const relationID_t relationID,
		const std::string& name,
		const objectType_t objectType1,
		const objectID_t objectID1,
		const objectType_t objectType2,
		const objectID_t objectID2,
		const switchState_t switchState,
		const lockState_t lockState)
	:	manager(manager),
		relationID(relationID),
		name(name),
		objectType1(objectType1),
		objectID1(objectID1),
		objectType2(objectType2),
		objectID2(objectID2),
		switchState(switchState),
		lockState(lockState)
	{};

	std::string Relation::serialize() const {
		stringstream ss;
		ss << "relationID=" << (int)relationID
			<< ";name=" << name
			<< ";objectType1=" << objectType1
			<< ";objectID1=" << objectID1
			<< ";objectType2=" << objectType2
			<< ";objectID2=" << objectID2
			<< ";switchState=" << static_cast<int>(switchState)
			<< ";lockState=" << static_cast<int>(lockState);
		return ss.str();
	}

	bool Relation::deserialize(const std::string& serialized) {
		map<string,string> arguments;
		parseArguments(serialized, arguments);
		return deserialize(arguments);
	}

	bool Relation::deserialize(const map<string,string>& arguments) {
		if (arguments.count("relationID")) relationID = stoi(arguments.at("relationID"));
		if (arguments.count("name")) name = arguments.at("name");
		if (arguments.count("objectType1")) objectType1 = stoi(arguments.at("objectType1"));
		if (arguments.count("objectID1")) objectID1 = stoi(arguments.at("objectID1"));
		if (arguments.count("objectType2")) objectType2 = stoi(arguments.at("objectType2"));
		if (arguments.count("objectID2")) objectID2 = stoi(arguments.at("objectID2"));
		if (arguments.count("switchState")) switchState = stoi(arguments.at("switchState"));
		if (arguments.count("lockState")) lockState = stoi(arguments.at("lockState"));
		return true;
	}

	bool Relation::execute(const locoID_t locoID) {
		if (objectType2 != ObjectTypeSwitch) {
			return false;
		}
		Switch* mySwitch = manager->getSwitch(objectID2);
		if (mySwitch == nullptr) {
			return false;
		}
		if (!mySwitch->reserve(locoID)) {
			return false;
		}
		if (lockState == LOCK_STATE_HARD_LOCKED) {
			mySwitch->hardLock(locoID, switchState);
		}
		else {
			mySwitch->softLock(locoID, switchState);
		}

		manager->handleSwitch(ControlTypeAutomode, objectID2, switchState);
		return true;
	}

	bool Relation::release(const locoID_t locoID) {
		if (objectType2 != ObjectTypeSwitch) {
			return false;
		}
		Switch* mySwitch = manager->getSwitch(objectID2);
		if (mySwitch == nullptr) {
			return false;
		}
		return mySwitch->release(locoID);
	}

} // namespace datamodel
