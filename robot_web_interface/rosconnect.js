// Simple ROS WebSocket Connection (without external libraries)
function ROSConnection(url) {
    this.url = url;
    this.ws = null;
    this.listeners = {};
    this.connected = false;
    
    this.connect = function() {
        var self = this;
        this.ws = new WebSocket(this.url);
        
        this.ws.onopen = function() {
            console.log('Connected to ROS');
            self.connected = true;
            if (self.onConnect) self.onConnect();
        };
        
        this.ws.onclose = function() {
            console.log('Disconnected from ROS');
            self.connected = false;
            if (self.onClose) self.onClose();
        };
        
        this.ws.onerror = function(error) {
            console.log('WebSocket error:', error);
            if (self.onError) self.onError(error);
        };
        
        this.ws.onmessage = function(event) {
            var data = JSON.parse(event.data);
            
            if (data.op === 'publish' && data.topic) {
                if (self.listeners[data.topic]) {
                    self.listeners[data.topic].forEach(function(callback) {
                        callback(data.msg);
                    });
                }
            }
        };
    };
    
    // Subscribe to a topic
    this.subscribe = function(topic, messageType, callback) {
        var self = this;
        
        // Register listener
        if (!this.listeners[topic]) {
            this.listeners[topic] = [];
        }
        this.listeners[topic].push(callback);
        
        // Send subscribe message
        var subscribeMsg = {
            op: 'subscribe',
            topic: topic,
            type: messageType
        };
        
        if (this.connected) {
            this.ws.send(JSON.stringify(subscribeMsg));
        } else {
            // Wait for connection
            var interval = setInterval(function() {
                if (self.connected) {
                    self.ws.send(JSON.stringify(subscribeMsg));
                    clearInterval(interval);
                }
            }, 100);
        }
    };

    // Publish to a topic
    this.publish = function(topic, messageType, msg) {
        var self = this;
        var publishMsg = {
            op: 'publish',
            topic: topic,
            type: messageType,
            msg: msg
        };

        if (this.connected) {
            this.ws.send(JSON.stringify(publishMsg));
        } else {
            // Wait for connection
            var interval = setInterval(function() {
                if (self.connected) {
                    self.ws.send(JSON.stringify(publishMsg));
                    clearInterval(interval);
                }
            }, 100);
        }
    };
}
