// Create ROS connection
var ros = new ROSLIB.Ros();

var host = window.location.hostname;
var port = 9090;

// Connection handlers
ros.on("connection", function() {
    console.log('Connected to rosbridge websocket server.');
    updateConnectionStatus(true);
});

ros.on("error", function(error) {
    console.log('Error connecting to rosbridge websocket server: ', error);
    updateConnectionStatus(false);
});

ros.on("close", function() {
    console.log('Connection to rosbridge websocket server closed.');
    updateConnectionStatus(false);
});

// Connect
ros.connect(`ws://${host}:${port}`);

// -----------------------
// Joystick Logic
// -----------------------

const joystickContainer = document.getElementById('joystick-container');
const joystick = document.getElementById('joystick');

let dragging = false;
let joystickInterval = null;
let normX = 0;
let normY = 0;

// Metrics
function updateJoystickMetrics() {
    const rect = joystickContainer.getBoundingClientRect();
    return {
        centerX: rect.width / 2 - joystick.offsetWidth / 2,
        centerY: rect.height / 2 - joystick.offsetHeight / 2,
        radius: rect.width / 2 - joystick.offsetWidth / 2
    };
}

let { centerX, centerY, radius } = updateJoystickMetrics();

// Start centered
joystick.style.left = `${centerX}px`;
joystick.style.top = `${centerY}px`;

// -----------------------
// Helper to stop joystick
// -----------------------
function stopJoystick() {
    if (!dragging) return;
    dragging = false;

    if (joystickInterval) {
        clearInterval(joystickInterval);
        joystickInterval = null;
    }

    joystick.style.left = `${centerX}px`;
    joystick.style.top = `${centerY}px`;

    normX = 0;
    normY = 0;
    sendJoystickCommand(0, 0);
}

// -----------------------
// Mouse & Touch move handler
// -----------------------
function handleMove(clientX, clientY) {
    const rect = joystickContainer.getBoundingClientRect();
    let x = clientX - rect.left - rect.width / 2;
    let y = clientY - rect.top - rect.height / 2;

    const dist = Math.sqrt(x*x + y*y);
    if (dist > radius) {
        x = (x / dist) * radius;
        y = (y / dist) * radius;
    }

    joystick.style.left = `${x + rect.width / 2 - joystick.offsetWidth / 2}px`;
    joystick.style.top = `${y + rect.height / 2 - joystick.offsetHeight / 2}px`;

    normX = x / radius;
    normY = -y / radius; // forward/back
}

// -----------------------
// Mouse events
// -----------------------
joystick.addEventListener('mousedown', (e) => {
    e.preventDefault();
    dragging = true;

    if (!joystickInterval) {
        joystickInterval = setInterval(() => {
            sendJoystickCommand(normX, normY);
        }, 50);
    }
});

document.addEventListener('mouseup', stopJoystick);
document.addEventListener('mousemove', (e) => {
    if (!dragging) return;
    handleMove(e.clientX, e.clientY);
});

// -----------------------
// Touch events
// -----------------------
joystick.addEventListener('touchstart', (e) => {
    e.preventDefault();
    dragging = true;

    if (!joystickInterval) {
        joystickInterval = setInterval(() => {
            sendJoystickCommand(normX, normY);
        }, 50);
    }
});

document.addEventListener('touchend', stopJoystick);
document.addEventListener('touchcancel', stopJoystick);
document.addEventListener('touchmove', (e) => {
    if (!dragging) return;
    const touch = e.touches[0];
    handleMove(touch.clientX, touch.clientY);
});


// -----------------------
// Send Twist Message
// -----------------------
var cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: "/cmd_vel",
    messageType: "geometry_msgs/Twist",
});

function sendJoystickCommand(x, y) {
    var twistMsg = {
        linear: { x: y/2, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: x/2 }
    };

    cmdVel.publish(twistMsg);
}

// Update connection status indicator
function updateConnectionStatus(connected) {
    var indicator = document.getElementById('connectionIndicator');
    var text = document.getElementById('connectionText');
    
    if (connected) {
        indicator.className = 'status-indicator status-connected';
        text.textContent = 'Connected to Robot';
    } else {
        indicator.className = 'status-indicator status-disconnected';
        text.textContent = 'Disconnected';
    }
}

// Subscribe to Battery topic
var bmsstate = new ROSLIB.Topic({
    ros: ros,
    name: "/bmsstate",
    messageType: "unitree_interfaces/msg/BmsState",
});

bmsstate.subscribe(function(message) {
    // Update battery SOC
    document.getElementById('batterySOC').textContent = message.soc + '%';
    
    // Update battery bar
    var batteryFill = document.getElementById('batteryFill');
    batteryFill.style.width = message.soc + '%';
    batteryFill.textContent = message.soc + '%';
    
    // Change color based on battery level
    if (message.soc > 50) {
        batteryFill.style.background = 'linear-gradient(90deg, #4CAF50, #8BC34A)';
    } else if (message.soc > 20) {
        batteryFill.style.background = 'linear-gradient(90deg, #FFC107, #FF9800)';
    } else {
        batteryFill.style.background = 'linear-gradient(90deg, #f44336, #E91E63)';
    }
});

// Subscribe to Sport Mode State
var sportmodestate = new ROSLIB.Topic({
    ros: ros,
    name: "/sportmodestate",
    messageType: "unitree_go/msg/SportModeState",
});

sportmodestate.subscribe(function(message) {
    // Update robot mode
    var modeNames = {
        0: 'Idle', 1: 'Force Stand', 2: 'Target Stand',
        5: 'Low Stand', 6: 'Low Walk',
        7: 'High Stand', 8: 'High Walk', 9: 'Climb Stair'
    };
    document.getElementById('robotMode').textContent = modeNames[message.mode] || 'Unknown (' + message.mode + ')';
    
    // Update gait type
    var gaitNames = {
        0: 'Idle', 1: 'Trot', 2: 'Trot Running',
        3: 'Climb Stair', 4: 'Trot Obstacle'
    };
    document.getElementById('gaitType').textContent = gaitNames[message.gait_type] || 'Unknown (' + message.gait_type + ')';
    
    // Update body height
    document.getElementById('bodyHeight').textContent = message.body_height.toFixed(3) + ' m';
    
    // Update position
    if (message.position && message.position.length >= 3) {
        document.getElementById('posX').textContent = message.position[0].toFixed(2) + ' m';
        document.getElementById('posY').textContent = message.position[1].toFixed(2) + ' m';
        document.getElementById('posZ').textContent = message.position[2].toFixed(2) + ' m';
    }
    
    // Update velocity
    if (message.velocity && message.velocity.length >= 3) {
        document.getElementById('velX').textContent = message.velocity[0].toFixed(2) + ' m/s';
        document.getElementById('velY').textContent = message.velocity[1].toFixed(2) + ' m/s';
        document.getElementById('velZ').textContent = message.velocity[2].toFixed(2) + ' m/s';
    }
    
    // Update yaw
    if (message.imu_state && message.imu_state.rpy && message.imu_state.rpy.length >= 3) {
        var yawDegrees = (message.imu_state.rpy[2] * 180 / Math.PI).toFixed(1);
        document.getElementById('yaw').textContent = yawDegrees + ' °';
    }
    
    // Update yaw speed
    document.getElementById('yawSpeed').textContent = message.yaw_speed.toFixed(2) + ' rad/s';
});

// Subscribe to SLAM Info
var slaminfo = new ROSLIB.Topic({
    ros: ros,
    name: "/slam_info",
    messageType: "std_msgs/msg/String",
});

slaminfo.subscribe(function(message) {
    try {
        var slamData = JSON.parse(message.data);
        
        if (slamData.type === 'heart_beat' && slamData.data) {
            var statusBox = document.getElementById('slamStatusBox');
            
            var statusNames = {
                0: 'Normal Operation',
                1: 'Initializing',
                2: 'Lost Tracking',
                3: 'Error'
            };
            
            var statusText = statusNames[slamData.data.currentStatus] || 'Unknown';
            document.getElementById('slamStatus').textContent = statusText;
            
            if (slamData.data.currentStatus === 0) {
                statusBox.className = 'slam-status slam-active';
            } else {
                statusBox.className = 'slam-status slam-inactive';
            }
            
            document.getElementById('slamError').textContent = slamData.data.errorCode;
            
            var date = new Date(slamData.data.sec * 1000);
            document.getElementById('slamTime').textContent = date.toLocaleTimeString();
        }
    } catch (e) {
        console.error('Error parsing SLAM data:', e);
    }
});

// Front Camera
var frontCanvas = document.getElementById('frontCamera');
var frontCtx = frontCanvas.getContext('2d');
var frontFrameCount = 0;

var frontcam = new ROSLIB.Topic({
    ros: ros,
    name: "/front_camera/image_raw/compressed",
    messageType: "sensor_msgs/msg/CompressedImage",
});

frontcam.subscribe(function(message) {
    const img = new Image();
    img.src = 'data:image/jpeg;base64,' + message.data;
    img.onload = function() {
        frontCanvas.width = img.width;
        frontCanvas.height = img.height;
        frontCtx.drawImage(img, 0, 0);
        document.getElementById('frontCameraStatus').textContent = 'Live';
    };
});


// Back Camera
var backCanvas = document.getElementById('backCamera');
var backCtx = backCanvas.getContext('2d');
var backFrameCount = 0;

var backcam = new ROSLIB.Topic({
    ros: ros,
    name: "/back_camera/image_raw/compressed",
    messageType: "sensor_msgs/msg/CompressedImage",
});

backcam.subscribe(function(message) {
    const img = new Image();
    img.src = 'data:image/jpeg;base64,' + message.data;
    img.onload = function() {
        backCanvas.width = img.width;
        backCanvas.height = img.height;
        backCtx.drawImage(img, 0, 0);
        document.getElementById('backCameraStatus').textContent = 'Live';
    };
});

var viewer = new ROS2D.Viewer({
    divID: 'mapCanvas',
    width: 600,
    height: 600
});

var gridClient = new ROS2D.OccupancyGridClient({
    ros: ros,
    rootObject: viewer.scene,
    topic: '/map',
    continuous: true
});

gridClient.on('change', function() {
    var grid = gridClient.currentGrid;
    if (!grid) return;

    if (!viewer.scene.contains(grid)) {
        viewer.scene.addChild(grid);
    }

    // Scale: pixels per meter
    const scale = 200; // 200 pixels per meter
    viewer.scaleX = scale;
    viewer.scaleY = scale;

    // Center map
    viewer.shift(grid.pose.position.x * scale, -grid.pose.position.y * scale);
});





console.log('Dashboard initialized. Waiting for ROS connection...');
