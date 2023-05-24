$(document).ready(function () {
    var ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });

    ros.on('connection', function () {
        console.log('Connected to websocket server.');
    });

    ros.on('error', function (error) {
        console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function () {
        ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });
    });

    // System State
    var systemStateListener = new ROSLIB.Topic({
        ros: ros,
        name: '/scr/state/system',
        messageType: 'scr_msgs/SystemState'
    });
    systemStateListener.subscribe(function (message) {
        console.log(message);
        const { state, mode, mobiltiy, estop } = message;
        $("#var/system/state").text(state);
        $("#var/system/mode").text(mode);
        $("#var/system/mobility").text(mobiltiy);
        $("#var/system/estop").text(estop);
    });

    // Motor Feedback
    var motorFeedbackListener = new ROSLIB.Topic({
        ros: ros,
        name: '/autonav/MotorFeedback',
        messageType: 'autonav_msgs/MotorFeedback'
    });
    motorFeedbackListener.subscribe(function (message) {
        const { delta_x, delta_y, delta_theta } = message;
        $("#var_motors_feedback").text(`(${delta_x}, ${delta_y}, ${delta_theta})`);
    });

    // Motor Input
    var motorInputListener = new ROSLIB.Topic({
        ros: ros,
        name: '/autonav/MotorInput',
        messageType: 'autonav_msgs/MotorInput'
    });
    motorInputListener.subscribe(function (message) {
        const { forward_velocity, angular_velocity } = message;
        $("#var_motors_velocity").text(`(${forward_velocity}, ${angular_velocity})`);
    });

    // GPS
    var gpsListener = new ROSLIB.Topic({
        ros: ros,
        name: '/autonav/gps',
        messageType: 'autonav_msgs/GPSFeedback'
    });
    gpsListener.subscribe(function (message) {
        const { latitude, longitude, gps_fix, is_locked, satellites } = message;
        $("#var_gps_position").text(`(${latitude}, ${longitude})`);
        $("#var_gps_fix").text(gps_fix);
        $("#var_gps_fixed").text(is_locked ? "Locked" : "Not Locked");
        $("#var_gps_satellites").text(satellites);
    });

    // GPS
    var gpsListener = new ROSLIB.Topic({
        ros: ros,
        name: '/autonav/PathingDebug',
        messageType: 'autonav_msgs/PathingDebug'
    });
    gpsListener.subscribe(function (message) {
        const { desired_heading, desired_latitude, desired_longitude, distance_to_destination, waypoints } = message;
        $("#var_astar_heading").text(desired_heading);
        $("#var_astar_waypoint").text(`(${desired_latitude}, ${desired_longitude})`);
        $("#var_astar_distance").text(distance_to_destination);
        $("#var_astar_waypoints").text(waypoints);
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
})