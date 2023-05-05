#pragma once

namespace SCR
{
	enum DeviceState : uint8_t
	{
		OFF = 0,
		STANDBY = 1,
		READY = 2,
		OPERATING = 3
	};

	std::string toString(DeviceState state)
	{
		switch (state)
		{
		case OFF:
			return "OFF";
		case STANDBY:
			return "STANDBY";
		case READY:
			return "READY";
		case OPERATING:
			return "OPERATING";
		default:
			return "UNKNOWN";
		}
	}
}