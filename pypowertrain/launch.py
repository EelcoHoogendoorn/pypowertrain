from threading import Thread
import webbrowser
import time

from pypowertrain.components.actuator import Actuator
from pypowertrain.components.battery import define_battery
from pypowertrain.library import odrive
from pypowertrain.system import System

system = System(
	actuator=Actuator(
		motor=odrive.D5065_270KV().replace(
			# __turns_scale=5,
			# __termination='delta',
			__length_scale=1.5,
		),
		controller=odrive.s1().replace(
			freq_limit=2000,
			field_weakening=True,
		),
	),
	battery=define_battery(v=48, wh=1e3),
)

# URL where the Dash app will run
URL = "http://127.0.0.1:8050"

# Function to open the browser after a delay
def open_browser():
    time.sleep(2)  # Wait for the server to start
    webbrowser.open(URL)

from pypowertrain.app import system_dash
dash_app = system_dash(system)

# Start the browser in a separate thread
Thread(target=open_browser).start()

# Run the Dash app server (this is blocking)
dash_app.run_server(debug=False)