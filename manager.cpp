#include <iostream>

#include "hardware_handler.h"
#include "manager.h"
#include "util.h"
#include "webserver/webserver.h"

using webserver::WebServer;

Manager::Manager() {
  controllers.push_back(new WebServer(*this, 8080));
	controllers.push_back(new HardwareHandler(*this));
}

Manager::~Manager() {
  for (auto control : controllers) {
    delete control;
  }
}

void Manager::go(const controlID_t controlID) {
  for (auto control : controllers) {
		control->go(controlID);
	}
}

void Manager::stop(const controlID_t controlID) {
  for (auto control : controllers) {
		control->stop(controlID);
	}
}

bool Manager::getProtocolAddress(const locoID_t locoID, hardwareControlID_t& hardwareControlID, protocol_t& protocol, address_t& address) {
	hardwareControlID = 0;
	protocol = PROTOCOL_DCC;
	address = 1228;
	return true;
}

void Manager::locoSpeed(const controlID_t controlID, const locoID_t locoID, const speed_t speed) {
  for (auto control : controllers) {
    control->locoSpeed(controlID, locoID, speed);
  }
}

