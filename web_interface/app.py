#!/usr/bin/env python3

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import roslibpy
import threading
import time
import json
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)

app = Flask(__name__)
app.config['SECRET_KEY'] = 'vlm_ros2_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables to store ROS connection and latest response
ros_client = None
latest_response = ""
connection_status = "disconnected"

def connect_to_ros():
    """Connect to ROS2 via rosbridge"""
    global ros_client, connection_status
    
    try:
        # Connect to rosbridge (make sure rosbridge_server is running)
        ros_client = roslibpy.Ros(host='localhost', port=9090)
        ros_client.run()
        
        if ros_client.is_connected:
            connection_status = "connected"
            print("✅ Connected to ROS2 via rosbridge")
            
            # Subscribe to VLM responses
            response_listener = roslibpy.Topic(ros_client, '/vlm_response', 'std_msgs/String')
            response_listener.subscribe(on_vlm_response)
            
            return True
        else:
            connection_status = "failed"
            print("❌ Failed to connect to ROS2")
            return False
            
    except Exception as e:
        connection_status = "failed"
        print(f"❌ ROS connection error: {e}")
        return False

def on_vlm_response(message):
    """Callback for VLM responses"""
    global latest_response
    latest_response = message['data']
    print(f"📨 Received VLM response: {latest_response}")
    
    # Emit to web interface
    socketio.emit('vlm_response', {'response': latest_response})

def publish_prompt(prompt_text):
    """Publish prompt to ROS2"""
    global ros_client
    
    if ros_client and ros_client.is_connected:
        try:
            prompt_publisher = roslibpy.Topic(ros_client, '/vlm_prompt', 'std_msgs/String')
            prompt_publisher.advertise()
            
            message = roslibpy.Message({'data': prompt_text})
            prompt_publisher.publish(message)
            
            print(f"📤 Published prompt: {prompt_text}")
            return True
        except Exception as e:
            print(f"❌ Error publishing prompt: {e}")
            return False
    else:
        print("❌ ROS2 not connected")
        return False

@app.route('/')
def index():
    """Main web interface"""
    return render_template('index.html')

@app.route('/api/send_prompt', methods=['POST'])
def send_prompt():
    """API endpoint to send prompts"""
    try:
        data = request.get_json()
        prompt = data.get('prompt', '')
        
        if not prompt.strip():
            return jsonify({'success': False, 'error': 'Empty prompt'})
        
        success = publish_prompt(prompt)
        
        if success:
            return jsonify({'success': True, 'message': 'Prompt sent successfully'})
        else:
            return jsonify({'success': False, 'error': 'Failed to publish prompt'})
            
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})

@app.route('/api/status')
def get_status():
    """Get connection status"""
    return jsonify({
        'ros_connected': connection_status == "connected",
        'latest_response': latest_response
    })

@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print('🌐 Client connected to web interface')
    emit('status', {
        'ros_connected': connection_status == "connected",
        'latest_response': latest_response
    })

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print('🌐 Client disconnected from web interface')

if __name__ == '__main__':
    # Start ROS connection in a separate thread
    def ros_connection_thread():
        while not connect_to_ros():
            print("🔄 Retrying ROS connection in 5 seconds...")
            time.sleep(5)
    
    ros_thread = threading.Thread(target=ros_connection_thread, daemon=True)
    ros_thread.start()
    
    # Start Flask app
    print("🚀 Starting VLM Web Interface on http://localhost:5000")
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)
