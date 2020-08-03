#!/usr/bin/python
# -*- coding: utf-8 -*-

# Dash visualisation for the basal ganglia model

# Plotly Dash components
import dash
import dash_daq as daq
import dash_core_components as dcc
import dash_html_components as html
import dash_bootstrap_components as dbc
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go

# Dashboard layout
import layout

# Basal ganglia model
from model import BasalGanglia

# Other components
import numpy as np

##########
# BG data structures
bg_data = {
	reg: {
		pop: [go.Scatter() for _ in range(layout.BG_CHANNELS)]
		for pop in BasalGanglia().model[reg].keys()
	} for reg in BasalGanglia().model.keys()
}
bg_data['Input'] = [go.Scatter() for _ in range(layout.BG_CHANNELS)]

# Storage component
dash_storage = dcc.Store(
	id='bg-store',
	data={
		reg: {
			pop: [[] for _ in range(layout.BG_CHANNELS)]
			for pop in BasalGanglia().model[reg].keys()
		} for reg in BasalGanglia().model.keys()
	}
)
dash_storage.data['Input'] = [[] for _ in range(layout.BG_CHANNELS)]

##########
# Include everything in the app layout
# See other included themes: https://bootswatch.com
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.FLATLY])
app.title = 'Basal ganglia visualisation'
app.layout = html.Div([
	layout.dash_rows['Input'],
	layout.dash_rows['Output'],
	layout.dash_intervals,
	dash_storage,
])


##########
# Dashboard callback
@app.callback(
	[
		Output('input-graph', 'figure'),
		Output('ventral-graph', 'figure'),
		Output('dorsal-graph', 'figure'),
		Output('bg-store', 'data'),
	],
	[Input('interval-fast', 'n_intervals')],
	[State('bg-store', 'data')] + [State('input-' + str(ch), 'value') for ch in range(layout.BG_CHANNELS)] + [State('lh-' + str(ch), 'value') for ch in range(layout.BG_CHANNELS)],
)
def callback_update_plots(_, bg_history, *slider_values):
	# FIXME: Apparent off-by-one error in plotting? Plot doesn't extend to edge

	# BG input values
	for ch, val in enumerate(slider_values[: layout.BG_CHANNELS]):
		bg_input[ch] = val

	# DEBUG LH values
	for ch, val in enumerate(slider_values[layout.BG_CHANNELS :]):
		lh_input[ch] = val

	# Run BG one step
	bg_model.step(bg_input, LH_APPROACH=lh_input, LH_AVOID=np.zeros(layout.BG_CHANNELS))
	# bg_model.step(bg_input)

	for ch in range(layout.BG_CHANNELS):
		for reg in bg_model.model.keys():
			for pop in bg_model.model[reg].keys():
				# Append new data to plot history
				bg_history[reg][pop][ch].append(bg_model.model[reg][pop]['o'][ch])

				# Trim existing data to plot length
				if len(bg_history[reg][pop][ch]) > layout.PLOT_LENGTH:
					bg_history[reg][pop][ch].pop(0)

				# Update plots
				bg_data[reg][pop][ch] = go.Scatter(
					hoverinfo='none',
					marker={
						'color': layout.PLOT_COLOURS[ch],
						'size' : 15,
						'line' : {'width': 1}
					},
					mode='lines',
					name='Channel ' + str(ch + 1),
					opacity=1,
					x=np.arange(0, layout.PLOT_LENGTH, 1),
					y=bg_history[reg][pop][ch],
					xaxis=layout.bg_ax[reg][pop]['x'],
					yaxis=layout.bg_ax[reg][pop]['y'],
				)

		# Append new data to plot history
		bg_history['Input'][ch].append(bg_input[ch])

		# Trim existing data to plot length
		if len(bg_history['Input'][ch]) > layout.PLOT_LENGTH:
			bg_history['Input'][ch].pop(0)

		# Update plots
		bg_data['Input'][ch] = go.Scatter(
			hoverinfo='none',
			marker={
				'color': layout.PLOT_COLOURS[ch],
				'size' : 15,
				'line' : {'width': 1}
			},
			mode='lines',
			name='Channel ' + str(ch + 1),
			opacity=1,
			x=np.arange(0, layout.PLOT_LENGTH, 1),
			y=bg_history['Input'][ch],
		)

	# Create figure objects for output
	input_figure = {
		'data'  : bg_data['Input'],
		'layout': layout.dash_layouts['Input']
	}

	ventral_figure = {
		'data'  : sum([bg_data['Ventral'][pop] for pop in BasalGanglia().model['Ventral'].keys()], []),
		'layout': layout.dash_layouts['Ventral']
	}

	dorsal_figure = {
		'data'  : sum([bg_data['Dorsal'][pop] for pop in BasalGanglia().model['Dorsal'].keys()], []),
		'layout': layout.dash_layouts['Dorsal']
	}

	# Return input figure, BG figures, and data store
	return input_figure, ventral_figure, dorsal_figure, bg_history


##########
# Main dashboard loop
if __name__ == '__main__':
	# Uncomment to suppress warnings TEMPORARILY
	# app.config['suppress_callback_exceptions'] = True

	# Initialise basal ganglia
	bg_model = BasalGanglia(channels=layout.BG_CHANNELS)
	bg_input = np.zeros(layout.BG_CHANNELS)
	lh_input = np.zeros(layout.BG_CHANNELS)

	# "debug=False" because hot reloading causes "IOError: [Errno 11] Resource temporarily unavailable" errors
	# "host='0.0.0.0'" allows connections from non-localhost addresses
	app.run_server(debug=False, host='0.0.0.0')
