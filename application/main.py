#   CS22B1090
#   Shubh Khandelwal

from flask import Flask, request, jsonify, render_template
import pandas as pd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
import threading
from threading import Lock

app = Flask(__name__)
lock = Lock()

class TargetPublisherNode(Node):

    def __init__(self):
        super().__init__('target_publisher')
        self.msg = Int8()
        self.target_publisher = self.create_publisher(Int8, '/target', 10)
        self.timer = self.create_timer(1, self.publisher_callback)

    def publisher_callback(self):
        with lock:
            self.target_publisher.publish(self.msg)
            self.msg.data = 0


class Inventory:

    def __init__(self, file_path):
        self.file_path = file_path
        self.database = pd.read_csv(file_path)
    
    def add_item(self, id):
        if id in self.database["id"].values:
            idx = self.database[self.database["id"] == id].index[0]
            if self.database.loc[idx, "quantity"] < 3:
                with lock:
                    node.msg.data = int(self.database.loc[idx, "location"])
                self.database.loc[idx, "quantity"] += 1
                self.database.to_csv(self.file_path, index=False)
                return 200
            return 501
        return 404

    def remove_item(self, id):
        if id in self.database["id"].values:
            idx = self.database[self.database["id"] == id].index[0]
            if self.database.loc[idx, "quantity"] > 0:
                with lock:
                    node.msg.data = int(self.database.loc[idx, "location"])
                self.database.loc[idx, "quantity"] -= 1
                self.database.to_csv(self.file_path, index=False)
                return 200
            return 501
        return 404

    def show(self):
        return self.database[["id", "description", "quantity"]].to_dict(orient="records")

file_path = "inventory.csv"
inventory = Inventory(file_path)

@app.route('/')
def home():
    return render_template('home.html')

@app.route('/api/inventory', methods=['GET'])
def get():
    return jsonify(inventory.show())

@app.route('/api/add', methods=['PUT'])
def add():
    data = request.json
    code = inventory.add_item(int(data["id"]))
    if code == 200:
        return jsonify({"message": "Item deposited successfully."}), code
    elif code == 501:
        return jsonify({"error": "Inventory overflow."}), code
    elif code == 404:
        return jsonify({"error": "Item not found."}), code

@app.route('/api/remove', methods=['PUT'])
def remove():
    data = request.json
    code = inventory.remove_item(int(data["id"]))
    if code == 200:
        return jsonify({"message": "Item withdrawn successfully."}), code
    elif code == 501:
        return jsonify({"error": "Inventory underflow."}), code
    elif code == 404:
        return jsonify({"error": "Item not found."}), code


def ros_spin():
    rclpy.spin(node)

if __name__ == '__main__':

    rclpy.init()
    node = TargetPublisherNode()

    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.start()

    app.run(debug=True, use_reloader=False)

    node.destroy_node()
    rclpy.shutdown()