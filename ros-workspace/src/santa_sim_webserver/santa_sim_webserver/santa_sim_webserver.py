from pathlib import Path
from threading import Thread

import rclpy
from flask import Flask
from rclpy.node import Node
from santa_sim_interface.msg import SleighState


class Webserver(Node):
    last_state = SleighState()

    def __init__(self):
        super().__init__("santa_sim_webserver")
        self.subscription = self.create_subscription(
            SleighState,
            "/santa_sim/sleigh_state",
            self.on_sleigh_state,
            10,
        )

    def on_sleigh_state(self, sleigh_state: SleighState):
        self.last_state = sleigh_state


def main():
    # start ros in a separate python thread
    rclpy.init()
    node = Webserver()
    thread = Thread(target=lambda: rclpy.spin(node), daemon=True)
    thread.start()

    # and the webserver in the main thread
    app = Flask(__name__)

    @app.route("/")
    def index():
        index_fp = Path(__file__).parent / "index.html"
        with index_fp.open() as f:
            return f.read()

    @app.route("/get_state.json")
    def get_state():
        state = node.last_state
        return dict(
            pos_x=state.pos_x,
            pos_y=state.pos_y,
            speed_x=state.speed_x,
            speed_y=state.speed_y,
        )

    app.run(host="0.0.0.0", port=5000)


if __name__ == "__main__":
    main()
