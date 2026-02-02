// Subscribe to odometry topic for robot position
var odomTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/odom",
    messageType: "nav_msgs/msg/Odometry",
});

odomTopic.subscribe(function(message) {
    // Update map renderer with robot position and orientation
    if (message.pose && message.pose.pose) {
        const robotPos = message.pose.pose.position;
        const robotOri = message.pose.pose.orientation;
        
        mapRenderer.updateRobotPose(robotPos, robotOri);
    }
});

class MapRenderer {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.mapData = null;
        this.zoom = 1.0;
        this.panX = 0;
        this.panY = 0;
        this.minZoom = 0.5;
        this.maxZoom = 5.0;
        this.zoomSensitivity = 0.1;
        
        // Set canvas size
        this.canvas.width = 600;
        this.canvas.height = 600;
        
        // Setup event listeners
        this.setupEventListeners();
        
        // Initialize with a default grid
        this.drawDefaultGrid();

        // Add this after the constructor
        this.robotPos = null;
        this.robotOri = null;

    }
    
    // Add this method to MapRenderer class
    updateRobotPose(position, orientation) {
        this.robotPos = position;
        this.robotOri = orientation;
        this.redraw();
    };

    setupEventListeners() {
        // Mouse wheel zoom
        this.canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const rect = this.canvas.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;
            
            const oldZoom = this.zoom;
            const zoomDirection = e.deltaY > 0 ? -1 : 1;
            this.zoom = Math.max(this.minZoom, Math.min(this.maxZoom, this.zoom + zoomDirection * this.zoomSensitivity));
            
            // Adjust pan to zoom towards mouse position
            const zoomRatio = this.zoom / oldZoom;
            this.panX = mouseX - (mouseX - this.panX) * zoomRatio;
            this.panY = mouseY - (mouseY - this.panY) * zoomRatio;
            
            document.getElementById('zoomLevel').textContent = this.zoom.toFixed(2) + 'x';
            this.redraw();
        });
        
        // Pan with mouse drag
        let isDragging = false;
        let lastX = 0;
        let lastY = 0;
        
        this.canvas.addEventListener('mousedown', (e) => {
            isDragging = true;
            lastX = e.clientX;
            lastY = e.clientY;
        });
        
        document.addEventListener('mousemove', (e) => {
            if (!isDragging) return;
            const deltaX = e.clientX - lastX;
            const deltaY = e.clientY - lastY;
            this.panX += deltaX;
            this.panY += deltaY;
            lastX = e.clientX;
            lastY = e.clientY;
            this.redraw();
        });
        
        document.addEventListener('mouseup', () => {
            isDragging = false;
        });
        
        // Touch events for mobile
        this.canvas.addEventListener('touchstart', (e) => {
            isDragging = true;
            lastX = e.touches[0].clientX;
            lastY = e.touches[0].clientY;
        });
        
        document.addEventListener('touchmove', (e) => {
            if (!isDragging) return;
            const deltaX = e.touches[0].clientX - lastX;
            const deltaY = e.touches[0].clientY - lastY;
            this.panX += deltaX;
            this.panY += deltaY;
            lastX = e.touches[0].clientX;
            lastY = e.touches[0].clientY;
            this.redraw();
        });
        
        document.addEventListener('touchend', () => {
            isDragging = false;
        });
    }
    
    drawDefaultGrid() {
        this.ctx.fillStyle = '#1a1a1a';
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
        
        this.ctx.strokeStyle = '#444';
        this.ctx.lineWidth = 1;
        
        // Draw grid
        for (let i = 0; i <= 10; i++) {
            const x = (i / 10) * this.canvas.width;
            const y = (i / 10) * this.canvas.height;
            
            this.ctx.beginPath();
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, this.canvas.height);
            this.ctx.stroke();
            
            this.ctx.beginPath();
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(this.canvas.width, y);
            this.ctx.stroke();
        }
        
        // Center marker
        this.ctx.fillStyle = '#4CAF50';
        this.ctx.beginPath();
        this.ctx.arc(this.canvas.width / 2, this.canvas.height / 2, 5, 0, Math.PI * 2);
        this.ctx.fill();
        
        this.ctx.fillStyle = '#aaa';
        this.ctx.font = '12px Arial';
        this.ctx.fillText('Map Origin', this.canvas.width / 2 + 10, this.canvas.height / 2 - 10);
    }
    
    updateMapData(mapData) {
        this.mapData = mapData;
        this.redraw();
        document.getElementById('mapStatus').textContent = `Map: ${mapData.info.width}x${mapData.info.height} (${mapData.info.resolution.toFixed(2)}m/cell)`;
    }
    
    redraw() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        if (this.mapData) {
            this.renderMapData();
        } else {
            this.drawDefaultGrid();
        }
    }
    
    // Helper function to draw a triangle marker
    drawTriangleMarker(x, y, yaw, size, color) {
        this.ctx.save();
        this.ctx.translate(x, y);
        this.ctx.rotate(yaw);
        
        this.ctx.fillStyle = color;
        this.ctx.beginPath();
        this.ctx.moveTo(0, 0);
        this.ctx.lineTo(0, size * 0.3);
        this.ctx.lineTo(size, 0);
        this.ctx.lineTo(0, -size * 0.3);
        this.ctx.closePath();
        this.ctx.fill();
        
        this.ctx.restore();
    }

    renderMapData() {
        const mapInfo = this.mapData.info;
        const mapData = this.mapData.data;
        const width = mapInfo.width;
        const height = mapInfo.height;
        const resolution = mapInfo.resolution;
        
        // Calculate how many pixels per cell on screen
        let cellPixels = Math.max(1, this.zoom);
        
        // Auto-fit map on first load
        if (this.zoom === 1.0 && this.panX === 0 && this.panY === 0) {
            const canvasWidth = this.canvas.width;
            const canvasHeight = this.canvas.height;
            const mapPixelWidth = width * cellPixels;
            const mapPixelHeight = height * cellPixels;
            
            // Calculate zoom to fit map in canvas
            const zoomX = canvasWidth / mapPixelWidth;
            const zoomY = canvasHeight / mapPixelHeight;
            this.zoom = Math.min(zoomX, zoomY) * 0.8; // 0.95 for some padding
            
            cellPixels = Math.max(1, this.zoom);
        }
        
        // Save context state
        this.ctx.save();
        
        // Apply pan and zoom transformations
        this.ctx.translate(this.panX, this.panY);
        this.ctx.scale(this.zoom, this.zoom);
        
        // Draw background
        this.ctx.fillStyle = '#a9a9a9'; // Unknown space (gray)
        this.ctx.fillRect(0, 0, width * cellPixels, height * cellPixels);
        
        // Draw occupancy grid
        for (let i = 0; i < mapData.length; i++) {
            const occupancy = mapData[i];
            
            // Calculate grid position
            const x = (i % width);
            const y = Math.floor(i / width);
            
            // Determine color based on occupancy value
            // -1: Unknown, 0-50: Free, 51-100: Occupied
            if (occupancy === -1) {
                // Unknown - don't draw (show background)
                continue;
            } else if (occupancy === 0) {
                // Free space - white or very light
                this.ctx.fillStyle = '#ffffff';
            } else if (occupancy <= 50) {
                // Probably free - light gray
                this.ctx.fillStyle = '#e8e8e8';
            } else {
                // Occupied - dark gray/black
                this.ctx.fillStyle = '#000000';
            }
            
            // Draw cell
            this.ctx.fillRect(x * cellPixels, y * cellPixels, cellPixels, cellPixels);
        }
        
        // Draw robot pose
        if (this.robotPos && this.robotOri) {
            const resolution = mapInfo.resolution;
            
            // Convert robot position from world coordinates to map coordinates
            const robotMapX = (this.robotPos.x - mapInfo.origin.position.x) / resolution;
            const robotMapY = (- this.robotPos.y - mapInfo.origin.position.y) / resolution;
            
            // Get robot orientation (convert quaternion to yaw)
            const q = this.robotOri;
            const robotYaw = -Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
            
            // Draw robot triangle
            const robotArrowSize = 5 * cellPixels;
            this.drawTriangleMarker(robotMapX * cellPixels, robotMapY * cellPixels, robotYaw, robotArrowSize, '#0066FF');
            
            this.ctx.restore();
        }

        // Draw origin marker (robot starting position) - Arrow pointing in direction
        this.ctx.restore();
        this.ctx.save();
        this.ctx.translate(this.panX, this.panY);
        this.ctx.scale(this.zoom, this.zoom);

        const originX = -mapInfo.origin.position.x / resolution;
        const originY = -mapInfo.origin.position.y / resolution;

        // Get rotation from origin quaternion (convert quaternion to yaw angle)
        let yaw = 0;
        if (mapInfo.origin.orientation) {
            const q = mapInfo.origin.orientation;
            // Convert quaternion to yaw (rotation around z-axis)
            yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        }

        // Draw arrow
        const arrowHeadSize = 5 * cellPixels;
        this.drawTriangleMarker(originX * cellPixels, originY * cellPixels, yaw, arrowHeadSize, '#00ff15');

        this.ctx.restore();
    }
}

// Initialize the map renderer
const mapRenderer = new MapRenderer('mapCanvas');

// Subscribe to map topic (when you have map data)
var mapTopic = new ROSLIB.Topic({
    ros: ros,
    name: "/map",
    messageType: "nav_msgs/msg/OccupancyGrid",
});

mapTopic.subscribe(function(message) {
    console.log('Map data received:', message);
    // Update map with real data
    mapRenderer.updateMapData(message);
});