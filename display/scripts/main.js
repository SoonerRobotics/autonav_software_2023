var systemState = {
    state: 0,
    mode: 0,
    mobility: false,
    estop: false
}
var preferences = {
    gpsFormat: "LL",
    host: "192.168.1.109",
    port: 8023,
}
var config = {}
var conbus = {};
var deviceStates = {};
var logs = [];

var addressKeys = {
    "autonav_serial_imu": {
        "internal_title": "[Serial] IMU",
        "imu_read_rate": "float",
        "imu_notfound_retry": "float",
        "imu_badconnect_retry": "float"
    },

    "autonav_serial_camera": {
        "internal_title": "[Serial] Camera",
        "refresh_rate": "int",
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
                type: "float",
            },
            0x40: {
                title: "Send Statistics",
                type: "bool",
            },
            0x50: {
                title: "Pulses Between Encoders",
                type: "float",
            }
        }
    }
}

function getBytesView(bytes) {
    var buffer = new ArrayBuffer(4);
    var view = new DataView(buffer);
    bytes.forEach(function (b, i) {
        view.setUint8(i, b);
    });
    return view;
}

function fromBytesToFloat(bytes) {
    return getBytesView(bytes).getFloat32(0, true);
}

function fromBytesToInt(bytes) {
    return getBytesView(bytes).getInt32(0, false);
}

function fromBytesToBool(bytes) {
    return getBytesView(bytes).getUint8(0) == 1;
}

function transferImageToElement(id, data) {
    const img = document.getElementById(id);
    img.src = "data:image/jpeg;base64," + data;
}

function fromFloatToBytes(value) {
    var buffer = new ArrayBuffer(4);
    var view = new DataView(buffer);
    view.setFloat32(0, value, true);
    return new Uint8Array(buffer);
}

function fromIntToBytes(value) {
    var buffer = new ArrayBuffer(4);
    var view = new DataView(buffer);
    view.setInt32(0, value, false);
    return new Uint8Array(buffer);
}

function fromBoolToBytes(value) {
    var buffer = new ArrayBuffer(1);
    var view = new DataView(buffer);
    view.setUint8(0, value ? 1 : 0);
    return new Uint8Array(buffer);
}

function createConbusReadInstruction(deviceId, address)
{
    return {
        id: 1000 + deviceId,
        data: [address]
    }
}

function createConbusReadResponse(deviceId, data)
{
    return {
        id: deviceId - 1100,
        address: data[0],
        length: data[1],
        reserved: data[2],
        data: data?.slice(3) ?? []
    }
}

function createConbusWriteInstruction(deviceId, address, data)
{
    return {
        id: 1200 + deviceId,
        data: [ address, data.length, 0, ...data ]
    }
}

function createConbusWriteResponse(deviceId, data)
{
    return {
        id: deviceId - 1300,
        address: data[0],
        length: data[1],
        reserved: data[2],
        data: data?.slice(3) ?? []
    }
}

$(document).ready(function () {
    ////////////////////////////////// Websocekt //////////////////////////////////

    var websocket;

    function createWebsocket()
    {
        websocket = new WebSocket(`ws://${preferences.host}:${preferences.port}`);

        websocket.onopen = function (event) {
            $("#connecting-state").text("Updating Data");
            $(".connecting-input").hide();
    
            send({
                op: "broadcast"
            });
            send({
                op: "get_nodes"
            });

            send({
                op: "conbus",
                ...createConbusReadInstruction(0x10, 0xFF)
            })
    
            setTimeout(() => {
                $(".connecting").hide();
                $("#main").show();
            }, 1000);
        };
    
        websocket.onmessage = function (event) {
            const obj = JSON.parse(event.data);
            const { op, topic } = obj;
    
            if (op == "data") {
                onTopicData(topic, obj);
            }
    
            if (op == "get_nodes_callback") {
                for (let i = 0; i < obj.nodes.length; i++) {
                    const node = obj.nodes[i];
                    send({
                        op: "configuration",
                        device: node,
                        opcode: 4
                    });
                }
            }
        };
    
        websocket.onclose = function (event) {
            $("#connecting-state").text("Waiting for the Weeb Wagon");
            $(".connecting").show();
            $(".connecting-input").show();
            $("#main").hide();
    
            setTimeout(() => {
                createWebsocket();
            }, 2000);
        };
    
        websocket.onerror = function (event) {
            console.error(event);
        };
    }

    createWebsocket();

    var sendQueue = [];

    function send(obj) {
        sendQueue.push(obj);
    }

    function setSystemState()
    {
        send({
            op: "set_system_state",
            state: systemState.state,
            estop: systemState.estop,
            mobility: systemState.mobility,
        });
    }

    function generateElementForConfiguration(data, type, device, text) {
        if (type == "bool") {
            const checked = fromBytesToBool(data);

            // Create a dropdown
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const select = document.createElement("select");
            select.classList.add("form-select");
            select.onchange = function () {
                send({
                    op: "configuration",
                    opcode: 1,
                    device: device,
                    address: text,
                    data: Array.from(fromBoolToBytes(select.value == 1))
                });
            }

            const optionTrue = document.createElement("option");
            optionTrue.value = 1;
            optionTrue.innerText = "True";
            optionTrue.selected = checked;

            const optionFalse = document.createElement("option");
            optionFalse.value = 0;
            optionFalse.innerText = "False";
            optionFalse.selected = !checked;

            select.appendChild(optionTrue);
            select.appendChild(optionFalse);

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(select);
            return div;
        }
        else if (type == "float") {
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const input = document.createElement("input");
            input.type = "number";
            input.classList.add("form-control");
            input.value = fromBytesToFloat(data).toFixed(6);
            input.onchange = function () {
                send({
                    op: "configuration",
                    opcode: 1,
                    device: device,
                    address: text,
                    data: Array.from(fromFloatToBytes(input.value))
                });
            }

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(input);
            return div;
        }
        else if (type == "int") {
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const input = document.createElement("input");
            input.type = "number";
            input.classList.add("form-control");
            input.value = fromBytesToInt(data);
            input.onchange = function () {
                send({
                    op: "configuration",
                    opcode: 1,
                    device: device,
                    address: text,
                    data: Array.from(fromIntToBytes(input.value))
                });
            }

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(input);
            return div;
        }
        else {
            const options = addressKeys[device][text];

            if (typeof options == "object") {
                const index = fromBytesToInt(data);

                // Create a dropdown
                const div = document.createElement("div");
                div.classList.add("input-group");
                div.classList.add("mb-3");

                const select = document.createElement("select");
                select.classList.add("form-select");
                select.onchange = function () {
                    send({
                        op: "configuration",
                        opcode: 1,
                        device: device,
                        address: text,
                        data: Array.from(fromIntToBytes(select.value))
                    });
                }

                for (let i = 0; i < Object.keys(options).length; i++) {
                    const option = document.createElement("option");
                    option.value = i;
                    option.selected = i == index;
                    option.innerText = options[i];
                    select.appendChild(option);
                }

                const span = document.createElement("span");
                span.classList.add("input-group-text");
                span.innerText = text;

                div.appendChild(span);
                div.appendChild(select);
                return div;
            }
        }
    }

    function generateElementForConbus(data, type, text, deviceId, address) {
        if (type == "bool") {
            const checked = fromBytesToBool(data);

            // Create a dropdown
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const select = document.createElement("select");
            select.classList.add("form-select");
            select.onchange = function () {
                const instruction = createConbusWriteInstruction(
                    parseInt(deviceId),
                    parseInt(address),
                    Array.from(fromBoolToBytes(select.value == 1))
                )

                send({
                    op: "conbus",
                    ...instruction
                });
            }

            const optionTrue = document.createElement("option");
            optionTrue.value = 1;
            optionTrue.innerText = "True";
            optionTrue.selected = checked;

            const optionFalse = document.createElement("option");
            optionFalse.value = 0;
            optionFalse.innerText = "False";
            optionFalse.selected = !checked;

            select.appendChild(optionTrue);
            select.appendChild(optionFalse);

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(select);
            return div;
        }
        else if (type == "float") {
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const input = document.createElement("input");
            input.type = "number";
            input.classList.add("form-control");
            input.value = fromBytesToFloat(data).toFixed(6);
            input.onchange = function () {
                const instruction = createConbusWriteInstruction(
                    parseInt(deviceId),
                    parseInt(address),
                    Array.from(fromFloatToBytes(input.value))
                )

                send({
                    op: "conbus",
                    ...instruction
                });
            }

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(input);
            return div;
        }
        else if (type == "int") {
            const div = document.createElement("div");
            div.classList.add("input-group");
            div.classList.add("mb-3");

            const input = document.createElement("input");
            input.type = "number";
            input.classList.add("form-control");
            input.value = fromBytesToInt(data);
            input.onchange = function () {
                const instruction = createConbusWriteInstruction(
                    parseInt(deviceId),
                    parseInt(address),
                    Array.from(fromIntToBytes(input.value))
                )

                send({
                    op: "conbus",
                    ...instruction
                });
            }

            const span = document.createElement("span");
            span.classList.add("input-group-text");
            span.innerText = text;

            div.appendChild(span);
            div.appendChild(input);
            return div;
        }
    }

    setInterval(() => {
        if (sendQueue.length > 0 && websocket.readyState == 1 && websocket.bufferedAmount == 0) {
            const obj = sendQueue.shift();
            console.log("Sending -> ", obj)
            websocket.send(JSON.stringify(obj));
        }
    }, 10);

    function onTopicData(topic, msg) {
        if (topic == "/scr/state/system") {
            const { state, mode, mobility, estop } = msg;
            console.log("System State", state, mode, mobility, estop);

            $("#var_system_state").text(state == 0 ? "Diabled" : state == 1 ? "Autonomous" : state == 2 ? "Manual" : "Shutdown");
            $("#var_system_mode").text(mode == 0 ? "Competition" : mode == 1 ? "Simulation" : "Practice");
            $("#var_system_mobility").text(mobility ? "Enabled" : "Disabled");
            $("#var_system_estop").text(estop ? "Yes" : "No");

            systemState.state = state;
            systemState.mode = mode;
            systemState.mobility = mobility;
            systemState.estop = estop;

            $("#input_system_state").val(state);
            $("#input_system_mode").val(mode);
            $("#input_system_mobility").prop("checked", mobility);
            $("#input_system_estop").prop("checked", estop);
            return;
        }

        if (topic == "/scr/state/device") {
            const { device, state } = msg;
            console.log("Device State", device, state);

            deviceStates[device] = state;
            unorderedListElement = $("#element_device_states");
            unorderedListElement.empty();
            for (const id in deviceStates) {
                const state = deviceStates[id];
                unorderedListElement.append(`<h5>${id}: <span data-state=\"${state}\">${deviceStateToName(state)}</span></h5>`);
            }
            return;
        }

        if (topic == "/scr/configuration/instruction") {
            const { device, opcode, data, address } = msg;
            if (opcode == 4) {
                return;
            }

            if (opcode == 2 || opcode == 3) {
                if (!(device in config)) {
                    config[device] = {};
                }

                config[device][address] = data;

                const configElement = $("#configuration");
                configElement.empty();

                for (const deviceId in config) {
                    const deviceConfig = config[deviceId];
                    const title = addressKeys[deviceId]["internal_title"];
                    const deviceElement = $(`<div class="card" style="margin-bottom: 10px;"></div>`);
                    deviceElement.append(`<div class="card-header"><h5>${title}</h5></div>`);
                    const deviceBody = $(`<div class="card-body"></div>`);
                    deviceElement.append(deviceBody);

                    for (const address in deviceConfig) {
                        const data = deviceConfig[address];
                        const type = addressKeys[deviceId][address];
                        const inputElement = generateElementForConfiguration(data, type, deviceId, address);
                        deviceBody.append(inputElement);
                    }

                    configElement.append(deviceElement);
                }
            }
            return;
        }

        if (topic == "/scr/logging") {
            logs.push({ message: msg.data, node: msg.node });
            if (logs.length > 75) {
                logs.shift();
            }

            const logElement = $(".element_logs");
            logElement.empty();
            for (let i = logs.length - 1; i >= 0; i--) {
                const log = logs[i];
                logElement.append(`<h5>${log.node}: ${log.message}</h5>`);
            }
            return;
        }

        if (topic == "/autonav/gps") {
            const { latitude, longitude, gps_fix, is_locked, satellites } = msg;
            $("#var_gps_position").text(`(${formatToFixed(latitude, 8)}, ${formatToFixed(longitude, 8)})`);
            $("#var_gps_fix").text(gps_fix);
            $("#var_gps_fixed").text(is_locked ? "Locked" : "Not Locked");
            $("#var_gps_satellites").text(satellites);
            return;
        }

        if (topic == "/autonav/MotorFeedback") {
            const { delta_x, delta_y, delta_theta } = msg;
            $("#var_motors_feedback").text(`(${formatToFixed(delta_x, 4)}, ${formatToFixed(delta_y, 4)}, ${formatToFixed(delta_theta, 4)}°)`);
            return;
        }

        if (topic == "/autonav/MotorInput") {
            const { forward_velocity, angular_velocity } = msg;
            $("#var_motors_velocity").text(`(${formatToFixed(forward_velocity, 3)}, ${formatToFixed(angular_velocity, 3)})`);
            return;
        }

        if (topic == "/autonav/debug/astar") {
            const { desired_heading, desired_latitude, desired_longitude, distance_to_destination, waypoints } = msg;
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
                    if (preferences.gpsFormat == "LL") {
                        return `(${formatToFixed(waypoint[0], 8)}, ${formatToFixed(waypoint[1], 8)})`;
                    }

                    const lat = waypoint[0];
                    const lon = waypoint[1];
                    return `(${convertLLToMSD(lat, lon)})`;
                }).join(", ")
            );
            return;
        }

        if (topic == "/autonav/position") {
            const { x, y, theta, latitude, longitude } = msg;
            $("#var_position_origin").text(`(${formatToFixed(x, 4)}, ${formatToFixed(y, 4)}, ${radiansToDegrees(parseFloat(theta)).toFixed(3)}°)`);
            $("#var_position_global").text(`(${formatToFixed(latitude, 8)}, ${formatToFixed(longitude, 8)})`);
            return;
        }

        if (topic == "/autonav/camera/compressed")
        {
            // const targetImgElement = document.getElementById("target_raw_camera");
            // targetImgElement.src = `data:image/jpeg;base64,${msg.data}`;
            // return;
        }

        if (topic == "/autonav/cfg_space/raw/image")
        {
            // const filteredImgElement = document.getElementById("target_filtered_camera");
            // filteredImgElement.src = `data:image/jpeg;base64,${msg.data}`;
            // return;
        }

        if (topic == "/autonav/imu")
        {
            const { accel_x, accel_y, accel_z, angular_x, angular_y, angular_z, yaw, pitch, roll } = msg;
            $("#var_imu_acceleration").text(`(${formatToFixed(accel_x, 4)}, ${formatToFixed(accel_y, 4)}, ${formatToFixed(accel_z, 4)})`);
            $("#var_imu_angular").text(`(${formatToFixed(angular_x, 4)}, ${formatToFixed(angular_y, 4)}, ${formatToFixed(angular_z, 4)})`);
            $("#var_imu_orientation").text(`(${radiansToDegrees(parseFloat(yaw)).toFixed(3)}°, ${radiansToDegrees(parseFloat(pitch)).toFixed(3)}°, ${radiansToDegrees(parseFloat(roll)).toFixed(3)}°)`);
        }

        if (topic == "/autonav/conbus")
        {
            const { id, data } = msg;
            let response;
            console.log("Received Conbus Message", id, data);
            if(id >= 1100 && id < 1200)
            {
                response = createConbusReadResponse(id, data);
                if (!(response.id in conbusDevices))
                {
                    return;
                }
            } else if (id >= 1300 && id < 1400)
            {
                response = createConbusWriteResponse(id, data);
                if (!(response.id in conbusDevices))
                {
                    return;
                }
            } else {
                return;
            }

            if(!(response.id in conbus))
            {
                conbus[response.id] = {};
            }
            conbus[response.id][response.address] = response.data;

            const conbusElement = $(`#conbus`);
            const conbusCard = $(`#conbus_${response.id}`);
            if (conbusCard != undefined || conbusCard.length != 0)
            {
                conbusCard.remove();
            }

            const card = $(`<div class="card" id="conbus_${response.id}" style="margin-bottom: 10px;"></div>`);
            card.append(`<div class="card-header"><h5>${conbusDevices[response.id].title}</h5></div>`);
            const cardBody = $(`<div class="card-body"></div>`);
            card.append(cardBody);

            for (const address in conbus[response.id])
            {
                // Just generate a p element
                const data = conbus[response.id][address];
                if (!(address in conbusDevices[response.id].registers))
                {
                    const title = conbusDevices[response.id]?.registers?.[address]?.title ?? address.toString();
                    const alert = $(`<div class="alert alert-danger" role="alert">Unknown Address: ${title}</div>`);
                    cardBody.append(alert);
                    continue;
                }
                const type = conbusDevices[response.id].registers[address].type;
                const title = conbusDevices[response.id].registers[address].title;
                const inputElement = generateElementForConbus(data, type, title, address, response.id);
                cardBody.append(inputElement);
            }

            for (const address in conbusDevices[response.id].registers)
            {
                if (!(address in conbus[response.id]))
                {
                    const title = conbusDevices[response.id].registers[address].title;
                    const alert = $(`<div class="alert alert-warning" role="alert">Missing Address: ${title}</div>`);
                    cardBody.append(alert);
                }
            }

            conbusElement.append(card);
            return;
        }
    }

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

    const deviceStateToName = (state) => {
        return state == 0 ? "Off" : state == 1 ? "Standby" : state == 2 ? "Ready" : "Operating";
    }

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

    $("#input_port").on("change", function () {
        const intt = parseInt($(this).val());
        preferences.port = isNaN(intt) ? 8023 : intt;

        if (isNaN(intt))
        {
            $(this).val(8023);
        }
    });

    $("#input_host").on("change", function () {
        preferences.host = $(this).val();
    });
})