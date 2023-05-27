var systemState = {
	state: 0,
	mode: 0,
	mobility: false,
	estop: false
}
var preferences = {
	gpsFormat: "DD",
	host: "127.0.0.1",
	port: 8023,
	theme: "dark"
}
var config = {}
var conbus = {};
var deviceStates = {};
var logs = [];
var iterator = 0;
var iterators = [];

var addressKeys = {
	"autonav_serial_imu": {
		"internal_title": "[Serial] IMU",
		"imu_read_rate": "float"
	},

	"autonav_serial_camera": {
		"internal_title": "[Serial] Camera",
		"refresh_rate": "int",
		"output_width": "int",
		"output_height": "int",
		"camera_index": "int"
	},

	"autonav_vision_transformer": {
		"internal_title": "[Vision] Transformer",
		"lower_hue": "int",
		"lower_saturation": "int",
		"lower_value": "int",
		"upper_hue": "int",
		"upper_saturation": "int",
		"upper_value": "int",
		"blur": "int",
		"blur_iterations": "int",
		"region_of_disinterest_offset": "int"
	},

	"autonav_filters": {
		"internal_title": "[Localization] Filters",
		"filter_type": {
			0: "Deadreckoning",
			1: "Particle Filter"
		},
		"degree_offset": "float",
		"seed_heading": "bool",
	},

	"autonav_manual_steamcontroller": {
		"internal_title": "[Manual] Steam Controller",
		"forward_speed": "float",
		"steering_deadzone": "float",
		"throttle_deadzone": "float",
		"turn_speed": "float",
		"max_forward_speed": "float",
		"max_turn_speed": "float"
	},

	"autonav_nav_astar": {
		"internal_title": "[Navigation] A*",
		"pop_distance": "float",
		"direction": {
			0: "North",
			1: "South",
			2: "Misc 1",
			3: "Misc 2",
			4: "Misc 3",
			5: "Misc 4",
			6: "Misc 5",
		},
		"use_only_waypoints": "bool",
		"waypoint_delay": "float",
	},

	"autonav_nav_resolver": {
		"internal_title": "[Navigation] Resolver",
		"forward_speed": "float",
		"reverse_speed": "float",
		"radius_multiplier": "float",
		"radius_max": "float",
		"radius_start": "float",
		"angular_aggression": "float",
		"max_angular_speed": "float"
	},

	"autonav_playback": {
		"internal_title": "[Playback]",
		"record_imu": "bool",
		"record_gps": "bool",
		"record_position": "bool",
		"record_feedback": "bool",
		"record_objectdetection": "bool",
		"record_manual": "bool",
		"record_autonomous": "bool",
		"record_input": "bool",
		"record_debugfeedback": "bool",
	}
}

var conbusDevices = {
	0x10: {
		title: "Motor Controller",
		registers: {
			0x0: {
				title: "Update Period",
				type: "float",
				readonly: true
			},
			0x1: {
				title: "Pulses Per Radian",
				type: "float",
			},
			0x2: {
				title: "Wheel Radius",
				type: "float",
			},
			0x3: {
				title: "Wheel Base Length",
				type: "float",
			},
			0x4: {
				title: "Slew Rate Limit",
				type: "float",
			},
			0x5: {
				title: "Left Encoder Factor",
				type: "float",
			},
			0x6: {
				title: "Right Encoder Factor",
				type: "float",
			},
			0x10: {
				title: "Velocity Kp",
				type: "float",
			},
			0x11: {
				title: "Velocity Ki",
				type: "float",
			},
			0x12: {
				title: "Velocity Kd",
				type: "float",
			},
			0x13: {
				title: "Velocity Kf",
				type: "float",
			},
			0x20: {
				title: "Angular Kp",
				type: "float",
			},
			0x21: {
				title: "Angular Ki",
				type: "float",
			},
			0x22: {
				title: "Angular Kd",
				type: "float",
			},
			0x23: {
				title: "Angular Kf",
				type: "float",
			},
			0x30: {
				title: "Use Obstacle Avoidance",
				type: "bool",
			},
			0x31: {
				title: "Collision Box Distance",
				type: "uint",
			},
			0x40: {
				title: "Send Statistics",
				type: "bool",
			},
			0x50: {
				title: "Motor Updates Between Deltaodom",
				type: "uint",
			}
		}
	},
	0x11: {
		title: "Safety Lights",
		registers: {
			0x0: {
				title: "Blink Rate",
				type: "uint",
			}
		}
	}
}