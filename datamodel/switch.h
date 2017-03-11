#ifndef DATAMODEL_SWITCH_H
#define DATAMODEL_SWITCH_H

#include <string>

#include "datatypes.h"
#include "accessory.h"

namespace datamodel {

	class Switch : public Accessory {
		public:
			Switch(switchID_t switchID, std::string name, controlID_t controlID, protocol_t protocol, address_t address, switchType_t type, layoutRotation_t rotation, layoutPosition_t x, layoutPosition_t y, layoutPosition_t z);

			switchID_t switchID;
			layoutRotation_t rotation;
	};

} // namespace datamodel

#endif // DATAMODEL_SWITCH_H