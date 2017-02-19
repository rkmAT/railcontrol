#ifndef HARDWARE_HARDAWRE_INTERFACE_H
#define HARDWARE_HARDWARE_INTERFACE_H

#include <string>

#include "../datatypes.h"

namespace hardware {

	class HardwareInterface {
		public:
		  // non virtual default constructor is needed to prevent polymorphism
			HardwareInterface() {};

			// pure virtual destructor prevents polymorphism in derived class
			virtual ~HardwareInterface() {};

			// get the name of the hardware
			virtual std::string getName() const = 0;

			// GO-command (turn on booster)
			virtual void go() = 0;

			// Stop-command (turn off booster)
			virtual void stop() = 0;

			// set loco speed
			virtual void locoSpeed(const protocol_t& protocol, const address_t& address, const speed_t& speed) = 0;
	};

} // namespace


#endif // HARDWARE_HARDWARE_INTERFACE_H

