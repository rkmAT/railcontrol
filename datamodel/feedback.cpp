#include <map>
#include <sstream>

#include "feedback.h"
#include "manager.h"

using std::map;
using std::stringstream;
using std::string;

namespace datamodel {

	Feedback::Feedback(Manager* manager, const feedbackID_t feedbackID, const std::string& name, const layoutPosition_t x, const layoutPosition_t y, const layoutPosition_t z, const controlID_t controlID, const feedbackPin_t pin, bool inverted) :
		LayoutItem(feedbackID, name, x, y, z, WIDTH_1, HEIGHT_1, ROTATION_0),
		controlID(controlID),
		pin(pin),
		manager(manager),
		state(FEEDBACK_STATE_FREE),
		locoID(LOCO_NONE),
		inverted(inverted) {
	}

	Feedback::Feedback(Manager* manager, const std::string& serialized) :
		manager(manager),
		locoID(LOCO_NONE) {
		deserialize(serialized);
	}

	std::string Feedback::serialize() const {
		stringstream ss;
		ss << "objectType=Feedback;" << LayoutItem::serialize() << ";controlID=" << (int)controlID << ";pin=" << (int)pin << ";inverted=" << (int)inverted << ";state=" << (int)state;
		return ss.str();
	}

	bool Feedback::deserialize(const std::string& serialized) {
		map<string,string> arguments;
		parseArguments(serialized, arguments);
		if (arguments.count("objectType") && arguments.at("objectType").compare("Feedback") == 0) {
			LayoutItem::deserialize(arguments);
			if (arguments.count("controlID")) controlID = stoi(arguments.at("controlID"));
			if (arguments.count("pin")) pin = stoi(arguments.at("pin"));
			if (arguments.count("inverted")) inverted = (bool)stoi(arguments.at("inverted"));
			if (arguments.count("state")) state = stoi(arguments.at("state"));
			return true;
		}
		return false;
	}

	bool Feedback::setLoco(const locoID_t locoID) {
		// FIXME: should check if already a loco is set / basically is done by street
		/*
		if (locoID == LOCO_NONE) {
			return false;
		}
		*/
		this->locoID = locoID;
		return true;
	}

	bool Feedback::setState(const feedbackState_t state) {
		this->state = (state ^ inverted) & 0x01;
		if (state == FEEDBACK_STATE_FREE) {
			return true;
		}

		if (locoID == LOCO_NONE) {
			return true;
		}

		manager->getLoco(locoID)->destinationReached();

		return true;
	}

} // namespace datamodel

