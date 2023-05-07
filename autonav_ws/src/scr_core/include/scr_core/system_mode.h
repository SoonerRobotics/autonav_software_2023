#pragma once

namespace SCR
{
	enum SystemMode : uint8_t
	{
		COMPETITION = 0,
		SIMULATION = 1,
		PRACTICE = 2,
	};

	std::string toString(SystemMode state)
	{
		switch(state)
		{
            case SystemMode::COMPETITION:
                return "Competition";
            case SystemMode::SIMULATION:
                return "Simulation";
            case SystemMode::PRACTICE:
                return "Practice";
		}
		return "Unknown";
	}
}