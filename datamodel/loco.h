#ifndef DATAMODEL_LOCO_H
#define DATAMODEL_LOCO_H

#include <mutex>
#include <string>
#include <thread>

#include "datatypes.h"
#include "object.h"

class Manager;

namespace datamodel {

	class Loco : public Object {
		public:
			Loco(Manager* manager, const locoID_t locoID, const std::string& name, const controlID_t controlID, const protocol_t protocol, const address_t address);
			Loco(Manager* manager, const std::string& serialized);
			~Loco();

			std::string serialize() const override;
			bool deserialize(const std::string& serialized) override;

			bool start();
			bool stop();

			bool toBlock(const blockID_t blockID);
			bool toBlock(const blockID_t blockIDOld, const blockID_t blockIDNew);
			bool releaseBlock();
			void destinationReached();

			void Speed(const speed_t speed);
			const speed_t Speed() const;

			controlID_t controlID;
			protocol_t protocol;
			address_t address;

		private:
			void autoMode(Loco* loco);
			Manager* manager;

			speed_t speed;
			locoState_t state;
			blockID_t blockID;
			streetID_t streetID;
			std::mutex stateMutex;
			std::thread locoThread;
	};

	inline void Loco::Speed(const speed_t speed) {
		this->speed = speed;
	}

	inline const speed_t Loco::Speed() const {
		return this->speed;
	}

} // namespace datamodel

#endif // DATAMODEL_LOCO_H
