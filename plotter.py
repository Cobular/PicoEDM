import sys
import json
import numpy as np
import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output
import dash_daq as daq
from collections import deque
import threading
import ast

# Initialize the Dash app
app = dash.Dash(__name__)
app.layout = html.Div(
    style={"height": "100vh", "display": "flex", "flexDirection": "column"},
    children=[
        dcc.Graph(id="live-graph", animate=True, style={'flex': '1'}),
        dcc.Interval(
            id="graph-update", interval=1000, n_intervals=0
        ),  # Update every second
    ],
)

# Deque to store incoming data points
data_deque = deque(maxlen=40_000)  # Adjust maxlen as needed


@app.callback(Output("live-graph", "figure"), [Input("graph-update", "n_intervals")])
def update_graph(n_intervals):
    x = np.arange(len(data_deque))
    y = np.array(data_deque)

    figure = {
        "data": [
            {"x": x, "y": y, "type": "line", "name": "Real-time data"},
        ],
        "layout": {
            "title": "Real-time Data Plot",
            "xaxis": {"title": "Index"},
            "yaxis": {
                "title": "Value",
                "range": [0, 2_300],  # Set your desired y-axis range here
            },
        },
    }
    return figure


def read_stdin():
    while True:
        line = sys.stdin.readline().strip()
        if line:
            try:
                data = json.loads(line)
                if data.get("data") is not None:
                    new_data = np.array(ast.literal_eval(data["data"]))
                    # print("WEEE", type(data["data"]), new_data)
                    data_deque.extend(new_data)
                    # print(f"Got data, mean {np.mean(new_data)}")
            except ValueError:
                pass


# Run the stdin reading in a separate thread
stdin_thread = threading.Thread(target=read_stdin, daemon=True)
stdin_thread.start()

# Run the Dash app
if __name__ == "__main__":
    app.run_server(debug=True, use_reloader=False)
