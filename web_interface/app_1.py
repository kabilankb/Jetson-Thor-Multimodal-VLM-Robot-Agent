#!/usr/bin/env python3

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import roslibpy
import threading
import time
import json
import logging
import base64
import cv2
import numpy as np

# Configure logging
logging.basicConfig(level=logging.INFO)

app = Flask(__name__)
app.config['SECRET_KEY'] = 'vlm_ros2_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables to store ROS connection and latest data
ros_client = None
latest_response = ""
latest_image_data = None
connection_status = "disconnected"
connected_clients = set()

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
            
            # Subscribe to web camera feed (base64 images)
            camera_listener = roslibpy.Topic(ros_client, '/camera/web_feed', 'std_msgs/String')
            camera_listener.subscribe(on_camera_web_feed)
            
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

def on_camera_web_feed(message):
    """Callback for base64 camera images from camera bridge"""
    global latest_image_data
    try:
        # The camera bridge sends base64 encoded images as String messages
        image_base64 = message['data']
        latest_image_data = image_base64
        
        # Emit to connected clients
        socketio.emit('camera_frame', {'image_data': image_base64})
        print(f"📷 Camera frame sent to web interface")
        
    except Exception as e:
        print(f"❌ Error processing camera feed: {e}")

def on_camera_image_raw(message):
    """Callback for raw camera images (backup method)"""
    global latest_image_data
    try:
        # Handle different data formats from rosbridge
        image_data = message.get('data', [])
        width = message.get('width', 0)
        height = message.get('height', 0)
        encoding = message.get('encoding', 'bgr8')
        
        if not image_data or width == 0 or height == 0:
            print(f"⚠️ Invalid image: width={width}, height={height}")
            return
            
        # Convert image data based on type
        if isinstance(image_data, str):
            # If data is a string, decode it properly
            try:
                # Try decoding as base64 first
                import binascii
                image_bytes = base64.b64decode(image_data)
            except:
                # If not base64, treat as raw bytes encoded as string
                image_bytes = image_data.encode('latin-1')
        elif isinstance(image_data, list):
            # If data is a list of integers
            image_bytes = bytes(image_data)
        else:
            print(f"⚠️ Unknown image data type: {type(image_data)}")
            return
            
        # Convert to numpy array and process
        import numpy as np
        
        if encoding == 'bgr8':
            img_array = np.frombuffer(image_bytes, dtype=np.uint8)
            if len(img_array) != width * height * 3:
                print(f"⚠️ Image size mismatch: expected {width*height*3}, got {len(img_array)}")
                return
                
            img_array = img_array.reshape((height, width, 3))
            
            # Resize for web if too large
            if width > 640:
                scale = 640 / width
                new_width = int(width * scale)
                new_height = int(height * scale)
                img_array = cv2.resize(img_array, (new_width, new_height))
            
            # Encode as JPEG
            _, buffer = cv2.imencode('.jpg', img_array, [cv2.IMWRITE_JPEG_QUALITY, 80])
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            latest_image_data = image_base64
            socketio.emit('camera_frame', {'image_data': image_base64})
            
        else:
            print(f"⚠️ Unsupported encoding: {encoding}")
        
    except Exception as e:
        print(f"❌ Error processing raw camera image: {e}")
        import traceback
        traceback.print_exc()

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
    return render_template('index_1.html')

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
        'latest_response': latest_response,
        'camera_available': latest_image_data is not None
    })

@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print('🌐 Client connected to web interface')
    connected_clients.add(request.sid)
    
    emit('status', {
        'ros_connected': connection_status == "connected",
        'latest_response': latest_response
    })
    
    # Send latest camera frame if available
    if latest_image_data:
        emit('camera_frame', {'image_data': latest_image_data})

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print('🌐 Client disconnected from web interface')
    connected_clients.discard(request.sid)

@socketio.on('subscribe_camera')
def handle_camera_subscription():
    """Handle camera subscription request"""
    print('📷 Client subscribed to camera feed')
    if latest_image_data:
        emit('camera_frame', {'image_data': latest_image_data})

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
