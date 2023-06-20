const getBytesView = (bytes) => {
	var buffer = new ArrayBuffer(4);
	var view = new DataView(buffer);
	bytes.forEach(function (b, i) {
		view.setUint8(i, b);
	});
	return view;
}

const fromBytesToFloat = (bytes) => {
	return getBytesView(bytes).getFloat32(0, true);
}

const fromBytesToInt = (bytes) => {
	return getBytesView(bytes).getInt32(0, false);
}

const fromBytesToUInt = (bytes) => {
	return getBytesView(bytes).getInt32(0, true);
}

const fromBytesToBool = (bytes) => {
	return getBytesView(bytes).getUint8(0) == 1;
}

const transferImageToElement = (id, data) => {
	const img = document.getElementById(id);
	img.src = "data:image/jpeg;base64," + data;
}

const fromFloatToBytes = (value) => {
	var buffer = new ArrayBuffer(4);
	var view = new DataView(buffer);
	view.setFloat32(0, value, true);
	return new Uint8Array(buffer);
}

const fromIntToBytes = (value) => {
	var buffer = new ArrayBuffer(4);
	var view = new DataView(buffer);
	view.setInt32(0, value, false);
	return new Uint8Array(buffer);
}

const fromUIntToBytes = (value) => {
	var buffer = new ArrayBuffer(4);
	var view = new DataView(buffer);
	view.setInt32(0, value, true);
	return new Uint8Array(buffer);
}

const fromBoolToBytes = (value) => {
	var buffer = new ArrayBuffer(1);
	var view = new DataView(buffer);
	view.setUint8(0, value ? 1 : 0);
	return new Uint8Array(buffer);
}

const createConbusReadInstruction = (deviceId, address) => {
	return {
		id: 1000 + deviceId,
		data: [address]
	}
}

const createConbusReadResponse = (deviceId, data) => {
	return {
		id: deviceId - 1100,
		address: data[0],
		length: data[1],
		reserved: data[2],
		data: data?.slice(3) ?? []
	}
}

const createConbusWriteInstruction = (deviceId, address, data) => {
	return {
		id: 1200 + deviceId,
		data: [address, data.length, 0, ...data]
	}
}

const createConbusWriteResponse = (deviceId, data) => {
	return {
		id: deviceId - 1300,
		address: data[0],
		length: data[1],
		reserved: data[2],
		data: data?.slice(3) ?? []
	}
}

const savePreferences = () => {
	localStorage.setItem("preferences", JSON.stringify(preferences));
}

const ddToDMS = (d) => {
	return {
		deg: 0 | d,
		min: 0 | (d = (d < 0 ? -d : d) + 1e-9) % 1 * 60,
		sec: d * 60 % 1 * 60
	}
}

const formatLatLong = (lat, long, includeParenthesis = false) => {
	if (preferences.gpsFormat == "DMS") {
		const newLat = ddToDMS(lat);
		const newLong = ddToDMS(long);
		return includeParenthesis ? `(${newLat.deg}°${newLat.min}'${newLat.sec.toFixed(5)}", ${newLong.deg}°${newLong.min}'${newLong.sec.toFixed(5)}")` : `${newLat.deg}°${newLat.min}'${newLat.sec.toFixed(5)}", ${newLong.deg}°${newLong.min}'${newLong.sec.toFixed(5)}"`;
	}

	return includeParenthesis ? `(${lat}, ${long})` : `${lat}, ${long}`;
}

const formatToFixed = (str, precision) => {
	return parseFloat(str).toFixed(precision);
}

const radiansToDegrees = (radians) => {
	return (radians * (180 / Math.PI) + 360) % 360;
}

const deviceStateToName = (state) => {
	return state == 0 ? "Off" : state == 1 ? "Standby" : state == 2 ? "Ready" : "Operating";
}

const clearGlobals = () => {
	systemState = {
		state: 0,
		mode: 0,
		mobility: false,
		estop: false
	}
	config = {}
	conbus = {};
	deviceStates = {};
	logs = [];
}

const clearElements = () => {
	const innerResets = [
		"var_motors_velocity", "var_motors_feedback", "var_position_origin", "var_position_global",
		"var_gps_position", "var_gps_fix", "var_gps_fixed", "var_gps_satellites",
		"var_imu_angular", "var_imu_acceleration", "var_imu_orientation",
		"var_astar_time", "var_astar_heading", "var_astar_waypoint", "var_astar_distance", "var_astar_waypoints",
		"var_system_state", "var_system_mode", "var_system_mobility", "var_system_estop",
	]

	const clearResets = [
		"element_device_states", "configuration", "conbus", "log_body"
	]

	for (const element of innerResets) {
		$(`#${element}`).html("");
	}

	for (const element of clearResets) {
		$(element).empty();
	}

	// Clear the three canvas's
	const canvas1 = document.getElementById("#target_raw_camera");
	const canvas2 = document.getElementById("#target_processed_camera");
	const canvas3 = document.getElementById("#target_astar");

	const ctx1 = canvas1.getContext("2d");
	const ctx2 = canvas2.getContext("2d");
	const ctx3 = canvas3.getContext("2d");

	ctx1.clearRect(0, 0, canvas1.width, canvas1.height);
	ctx2.clearRect(0, 0, canvas2.width, canvas2.height);
	ctx3.clearRect(0, 0, canvas3.width, canvas3.height);
}

const generateUUID = () => {
	return "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx".replace(/[xy]/g, function (c) {
		const r = Math.random() * 16 | 0,
			v = c == "x" ? r : (r & 0x3 | 0x8);
		return v.toString(16);
	});
}

const iterate = () => {
	iterator++;
	iterators.push(iterator);
	return iterator;
}