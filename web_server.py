from flask import Flask, render_template, request, jsonify, Response
import logging
from camera_pi import Camera
app = Flask(__name__)

# Shared data placeholder (to be passed in the main script)
shared_data = None

def gen(camera):
    """Video streaming generator function."""
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def parseFloats(in_str):
    try:
        parts = in_str.strip().split(',')
        if len(parts) != 2:
            return 0, 0, 0
        
        num1, num2 = map(float, parts)
        
        return 1, num1, num2
    except ValueError as e:
        return 0, 0, 0

@app.route('/')
def index():
    # Access the shared data
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(gen(Camera()), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/get_curr_battery', methods=['GET'])
def get_curr_battery():
    global shared_data
    return jsonify({"currBattery": shared_data["batteryLevel"]})

@app.route('/get_curr_lat', methods=['GET'])
def get_curr_lat():
    global shared_data
    return jsonify({"currLat": shared_data["currLat"]})
    
@app.route('/get_curr_long', methods=['GET'])
def get_curr_long():
    global shared_data
    return jsonify({"currLong": shared_data["currLong"]})
    
@app.route('/get_target_lat0', methods=['GET'])
def get_target_lat0():
    global shared_data
    return jsonify({"targetLat": shared_data["target_lats"][0]})
    
@app.route('/get_target_long0', methods=['GET'])
def get_target_long0():
    global shared_data
    return jsonify({"targetLong": shared_data["target_longs"][0]})
    
@app.route('/get_target_lat1', methods=['GET'])
def get_target_lat1():
    global shared_data
    return jsonify({"targetLat": shared_data["target_lats"][1]})
    
@app.route('/get_target_long1', methods=['GET'])
def get_target_long1():
    global shared_data
    return jsonify({"targetLong": shared_data["target_longs"][1]})
    
@app.route('/get_target_lat2', methods=['GET'])
def get_target_lat2():
    global shared_data
    return jsonify({"targetLat": shared_data["target_lats"][2]})
    
@app.route('/get_target_long2', methods=['GET'])
def get_target_long2():
    global shared_data
    return jsonify({"targetLong": shared_data["target_longs"][2]})

@app.route('/get_target_lat3', methods=['GET'])
def get_target_lat3():
    global shared_data
    return jsonify({"targetLat": shared_data["target_lats"][3]})
    
@app.route('/get_target_long3', methods=['GET'])
def get_target_long3():
    global shared_data
    return jsonify({"targetLong": shared_data["target_longs"][3]})

@app.route('/get_target_lat4', methods=['GET'])
def get_target_lat4():
    global shared_data
    return jsonify({"targetLat": shared_data["target_lats"][4]})
    
@app.route('/get_target_long4', methods=['GET'])
def get_target_long4():
    global shared_data
    return jsonify({"targetLong": shared_data["target_longs"][4]})

@app.route('/get_manual_override', methods=['GET'])
def get_manual_override():
    global shared_data
    return jsonify({"manualOverride": shared_data["manualOverride"]})
    
@app.route('/get_on_land', methods=['GET'])
def get_on_land():
    global shared_data
    return jsonify({"onLand": shared_data["onLand"]})

@app.route('/get_command', methods=['GET'])
def get_message():
    global shared_data
    return jsonify({"command": shared_data["command"]})
    
@app.route('/get_basic_speed', methods=['GET'])
def get_basic_speed():
    global shared_data
    return jsonify({"basicSpeed": shared_data["basicSpeed"]})

@app.route('/get_speed_left', methods=['GET'])
def get_speed_left():
    global shared_data
    return jsonify({"speedLeft": shared_data["speedLeft"]})

@app.route('/get_speed_right', methods=['GET'])
def get_speed_right():
    global shared_data
    return jsonify({"speedRight": shared_data["speedRight"]})









@app.route('/manualOverride', methods=['POST'])
def manualOverride():
    global shared_data
    user_input = request.json.get("value")
    shared_data["manualOverride"] = user_input  # Update shared data
    print("MESSAGE RECEIVED: " + str(user_input))
    return jsonify({"status": "success", "message": "Data updated!"})
    
@app.route('/onLand', methods=['POST'])
def onLand():
    global shared_data
    user_input = request.json.get("value")
    shared_data["onLand"] = user_input  # Update shared data
    print("MESSAGE RECEIVED: " + str(user_input))
    return jsonify({"status": "success", "message": "Data updated!"})
    
@app.route('/command', methods=['POST'])
def command():
    global shared_data
    user_input = request.json.get("command")
    shared_data["command"] = user_input  # Update shared data
    print("MESSAGE RECEIVED: " + str(user_input))
    return jsonify({"status": "success", "message": "Data updated!"})
    
@app.route('/basicSpeed', methods=['POST'])
def basicSpeed():
    global shared_data
    user_input = request.json.get("basicSpeed")
    shared_data["basicSpeed"] = user_input  # Update shared data
    print("MESSAGE RECEIVED: " + str(user_input))
    return jsonify({"status": "success", "message": "Data updated!"})
    
@app.route('/speedLeft', methods=['POST'])
def speed_left():
    global shared_data
    user_input = request.json.get("speedLeft")
    shared_data["speedLeft"] = user_input  # Update shared data
    print("MESSAGE RECEIVED: " + str(user_input))
    return jsonify({"status": "success", "message": "Data updated!"})
    
@app.route('/speedRight', methods=['POST'])
def speed_right():
    global shared_data
    user_input = request.json.get("speedRight")
    shared_data["speedRight"] = user_input  # Update shared data
    print("MESSAGE RECEIVED: " + str(user_input))
    return jsonify({"status": "success", "message": "Data updated!"})
    
@app.route('/set_coordinates', methods=['POST'])
def set_coordinates():
    global shared_data
    user_input = request.json.get("user_input")
    
    success, target_lat, target_long = parseFloats(user_input)
    if(success): 
        
        test1 = shared_data["target_lats"]
        test2 = shared_data["target_longs"]
        test1.pop()
        test2.pop()
        test1.appendleft(target_lat)  # Update shared data
        test2.appendleft(target_long)  # Update shared data
        
        shared_data["target_lats"] = test1
        shared_data["target_longs"] = test2
        
        print(list(shared_data["target_lats"]))
        print(list(shared_data["target_longs"]))

    print("COORDINATES RECEIVED!: " + str(user_input))
    return jsonify({"status": "success", "message": "Data updated!"})

def run_server(shared_data_instance):
    # Assign the shared data to the global variable
    global shared_data
    shared_data = shared_data_instance
    # Run Flask app
    app.logger.setLevel(logging.ERROR)
    app.run(host='0.0.0.0', port=5000)
