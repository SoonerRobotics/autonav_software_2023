$(document).ready(function () {
    var ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });

    ros.on('connection', function () {
        $(".connecting").hide();

        setTimeout(() => {
            broadcastStatePublisher.publish(new ROSLIB.Message({}));
        }, 500);
    });

    ros.on("open", function () {
        $(".connecting").hide();
    });

    ros.on('error', function (error) {
        console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function () {
        $(".connecting").show();
        const socket = ros.socket;
        if (socket) {
            socket.close();
        }

        setTimeout(function () {
            // location.reload();
        }, 500);
    });

    ////////////////////////////////// Globals //////////////////////////////////

    var systemState = {
        state: 0,
        mode: 0,
        mobility: false,
        estop: false
    }

    var preferences = {
        gpsFormat: "LL"
    }

    var config = {}

    ////////////////////////////////// Helpers //////////////////////////////////

    const formatToFixed = (str, precision) => {
        return parseFloat(str).toFixed(precision);
    }

    const radiansToDegrees = (radians) => {
        return (360 - (radians * (180 / Math.PI))) % 360; 
    }

    const convertLLToMSD = (lat, lon) => {
        const latDeg = Math.floor(lat);
        const latMin = Math.floor((lat - latDeg) * 60);
        const latSec = Math.floor(((lat - latDeg) * 60 - latMin) * 60);

        const lonDeg = Math.floor(lon);
        const lonMin = Math.floor((lon - lonDeg) * 60);
        const lonSec = Math.floor(((lon - lonDeg) * 60 - lonMin) * 60);

        return `${latDeg}°${latMin}'${latSec}"N ${lonDeg}°${lonMin}'${lonSec}"W`;
    }

    var broadcastStatePublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/scr/state/broadcast',
        messageType: 'std_msgs/Empty'
    });

    const deviceStateToName = (state) => {
        return state == 0 ? "Off" : state == 1 ? "Standby" : state == 2 ? "Ready" : "Operating";
    }

    ////////////////////////////////// Subscribers //////////////////////////////////

    // System State
    var systemStateListener = new ROSLIB.Topic({
        ros: ros,
        name: '/scr/state/system',
        messageType: 'scr_msgs/SystemState'
    });
    systemStateListener.subscribe(function (message) {
        const { state, mode, mobility, estop } = message;
        $("#var_system_state").text(state == 0 ? "Diabled" : state == 1 ? "Autonomous" : state == 2 ? "Manual" : "Shutdown");
        $("#var_system_mode").text(mode == 0 ? "Competition" : state == 1 ? "Simulation" : "Practice");
        $("#var_system_mobility").text(mobility ? "Enabled" : "Disabled");
        $("#var_system_estop").text(estop ? "Yes" : "No");

        // Update the input fields
        systemState.state = state;
        systemState.mode = mode;
        systemState.mobility = mobility;
        systemState.estop = estop;

        $("#input_system_state").val(state);
        $("#input_system_mode").val(mode);
        $("#input_system_mobility").prop("checked", mobility);
        $("#input_system_estop").prop("checked", estop);
    });

    var deviceStateListener = new ROSLIB.Topic({
        ros: ros,
        name: '/scr/state/device',
        messageType: 'scr_msgs/DeviceState'
    });
    var deviceStates = {};
    deviceStateListener.subscribe(async function (message) {
        const { device, state } = message;

        if (deviceStates[device] == undefined) {
            deviceStates[device] = state;
        }

        unorderedListElement = $("#element_device_states");
        unorderedListElement.empty();
        for (const id in deviceStates)
        {
            const state = deviceStates[id];
            unorderedListElement.append(`<h5>${id}: <span data-state=\"${state}\">${deviceStateToName(state)}</span></h5>`);
        }
    });

    // Motor Feedback
    var motorFeedbackListener = new ROSLIB.Topic({
        ros: ros,
        name: '/autonav/MotorFeedback',
        messageType: 'autonav_msgs/MotorFeedback'
    });
    motorFeedbackListener.subscribe(function (message) {
        const { delta_x, delta_y, delta_theta } = message;
        $("#var_motors_feedback").text(`(${formatToFixed(delta_x, 4)}, ${formatToFixed(delta_y, 4)}, ${formatToFixed(delta_theta, 4)}°)`);
    });

    // Motor Input
    var motorInputListener = new ROSLIB.Topic({
        ros: ros,
        name: '/autonav/MotorInput',
        messageType: 'autonav_msgs/MotorInput'
    });
    motorInputListener.subscribe(function (message) {
        const { forward_velocity, angular_velocity } = message;
        $("#var_motors_velocity").text(`(${formatToFixed(forward_velocity, 3)}, ${formatToFixed(angular_velocity, 3)})`);
    });

    // GPS
    var gpsListener = new ROSLIB.Topic({
        ros: ros,
        name: '/autonav/gps',
        messageType: 'autonav_msgs/GPSFeedback'
    });
    gpsListener.subscribe(function (message) {
        const { latitude, longitude, gps_fix, is_locked, satellites } = message;
        $("#var_gps_position").text(`(${formatToFixed(latitude, 8)}, ${formatToFixed(longitude, 8)})`);
        $("#var_gps_fix").text(gps_fix);
        $("#var_gps_fixed").text(is_locked ? "Locked" : "Not Locked");
        $("#var_gps_satellites").text(satellites);
    });

    // GPS
    var gpsListener = new ROSLIB.Topic({
        ros: ros,
        name: '/autonav/debug/astar',
        messageType: 'autonav_msgs/PathingDebug'
    });
    gpsListener.subscribe(function (message) {
        const { desired_heading, desired_latitude, desired_longitude, distance_to_destination, waypoints } = message;
        $("#var_astar_heading").text(`${radiansToDegrees(parseFloat(desired_heading)).toFixed(3)}°`);
        $("#var_astar_waypoint").text(`(${formatToFixed(desired_latitude, 8)}, ${formatToFixed(desired_longitude, 8)})`);
        $("#var_astar_distance").text(formatToFixed(distance_to_destination, 3));
        $("#var_astar_waypoints").text(
            waypoints.reduce((acc, val, i) => {
                if (i % 2 == 0) {
                    acc.push([val, waypoints[i + 1]]);
                }

                return acc;
            }, []).map((waypoint) => {
                if (preferences.gpsFormat == "LL")
                {
                    return `(${formatToFixed(waypoint[0], 8)}, ${formatToFixed(waypoint[1], 8)})`;
                }

                const lat = waypoint[0];
                const lon = waypoint[1];
                return `(${convertLLToMSD(lat, lon)})`;
            }).join(", ")
        );
    });  

    // Position
    var positionListener = new ROSLIB.Topic({
        ros: ros,
        name: '/autonav/position',
        messageType: 'autonav_msgs/Position'
    });
    positionListener.subscribe(function (message) {
        const { x, y, theta, latitude, longitude } = message;
        $("#var_position_origin").text(`(${formatToFixed(x, 4)}, ${formatToFixed(y, 4)}, ${radiansToDegrees(parseFloat(theta)).toFixed(3)}°)`);
        $("#var_position_global").text(`(${formatToFixed(latitude, 8)}, ${formatToFixed(longitude, 8)})`);
    });  

    // Raw Camera
    var rawCameraListener = new ROSLIB.Topic({
        ros: ros,
        name: '/autonav/camera/compressed',
        messageType: 'sensor_msgs/CompressedImage'
    });
    rawCameraListener.subscribe(function (message) {
        const targetImgElement = document.getElementById("target_raw_camera");
        targetImgElement.src = `data:image/jpeg;base64,${message.data}`;
    });

    // Filtered Camera
    var filteredCameraListener = new ROSLIB.Topic({
        ros: ros,
        name: '/autonav/cfg_space/raw/image',
        messageType: 'sensor_msgs/CompressedImage'
    });
    filteredCameraListener.subscribe(function (message) {
        const targetImgElement = document.getElementById("target_filtered_camera");
        targetImgElement.src = `data:image/jpeg;base64,${message.data}`;
    });

    var filteredCameraListener = new ROSLIB.Topic({
        ros: ros,
        name: '/autonav/debug/astar/image',
        messageType: 'sensor_msgs/CompressedImage'
    });
    filteredCameraListener.subscribe(function (message) {
        const targetImgElement = document.getElementById("target_astar_path");
        targetImgElement.src = `data:image/jpeg;base64,${message.data}`;
    });


    ////////////////////////////////// Publishers //////////////////////////////////

    var systemStateService = new ROSLIB.Service({
        ros: ros,
        name: '/scr/state/set_system_state',
        serviceType: 'scr_msgs/SystemState'
    });

    function setSystemState() {
        const request = new ROSLIB.ServiceRequest({
            state: systemState.state,
            mode: systemState.mode,
            mobility: systemState.mobility,
            estop: systemState.estop
        });
        systemStateService.callService(request, function (result) {
            console.log(result);
        });
    }

    // get aria-labelledby="dropdown_system_state"
    $(".dropdown-menu a").on("click", function () {
        const parentDataTarget = $(this).parents(".dropdown").attr("data-target");
        if (parentDataTarget == "system_state")
        {
            const id = $(this).attr("data-value");
            systemState.state = parseInt(id);
            setSystemState();
        } else if (parentDataTarget == "system_mode") {
            const id = $(this).attr("data-value");
            systemState.mode = parseInt(id);
            setSystemState();
        }
    });

    $("#checkbox_system_estop").on("change", function () {
        systemState.estop = $(this).is(":checked");
        setSystemState();
    });

    $("#checkbox_system_mobility").on("change", function () {
        systemState.mobility = $(this).is(":checked");
        setSystemState();
    });
})