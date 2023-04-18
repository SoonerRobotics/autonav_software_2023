#pragma once

namespace SCR
{
	enum SystemState : uint8_t
	{
		DISABLED = 0,
		AUTONOMOUS = 1,
		MANUAL = 2,
		SHUTDOWN = 3
	};

	std::string toString(SystemState state)
	{
		switch(state)
		{
			case SystemState::DISABLED:
				return "Disabled";
			case SystemState::AUTONOMOUS:
				return "Autonomous";
			case SystemState::MANUAL:
				return "Manual";
			case SystemState::SHUTDOWN:
				return "Shutdown";
		}
		return "Unknown";
	}
}