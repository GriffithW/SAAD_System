from flask import Flask, render_template, request, jsonify

app = Flask(__name__)

# Shared data placeholder (to be passed in the main script)
shared_data = None

@app.route('/')
def index():
    # Access the shared data
    return render_template('index.html', message=shared_data["message"])

@app.route('/get_message', methods=['GET'])
def get_message():
    global shared_data
    return jsonify({"message": shared_data["message"]})
    
@app.route('/get_target_coordinates', methods=['GET'])
def get_target_coordinates():
    global shared_data
    return jsonify({"target_coordinates": shared_data["target_coordinates"]})

@app.route('/get_speed_left', methods=['GET'])
def get_speed_left():
    global shared_data
    return jsonify({"speedLeft": shared_data["speedLeft"]})

@app.route('/get_speed_right', methods=['GET'])
def get_speed_right():
    global shared_data
    return jsonify({"speedRight": shared_data["speedRight"]})




@app.route('/command', methods=['POST'])
def command():
    global shared_data
    user_input = request.json.get("command")
    shared_data["message"] = user_input  # Update shared data
    print("MESSAGE RECEIVED: " + str(user_input))
    return jsonify({"status": "success", "message": "Data updated!"})
    
@app.route('/speedLeft', methods=['POST'])
def speed_left():
    global shared_data
    user_input = request.json.get("speed")
    shared_data["speedLeft"] = user_input  # Update shared data
    print("MESSAGE RECEIVED: " + str(user_input))
    return jsonify({"status": "success", "message": "Data updated!"})
    
@app.route('/speedRight', methods=['POST'])
def speed_right():
    global shared_data
    user_input = request.json.get("speed")
    shared_data["speedRight"] = user_input  # Update shared data
    print("MESSAGE RECEIVED: " + str(user_input))
    return jsonify({"status": "success", "message": "Data updated!"})
    
@app.route('/set_coordinates', methods=['POST'])
def set_coordinates():
    global shared_data
    user_input = request.json.get("user_input")
    shared_data["target_coordinates"] = user_input  # Update shared data
    print("MESSAGE RECEIVED: " + str(user_input))
    return jsonify({"status": "success", "message": "Data updated!"})

def run_server(shared_data_instance):
    # Assign the shared data to the global variable
    global shared_data
    shared_data = shared_data_instance
    # Run Flask app
    app.run(host='0.0.0.0', port=5000)
