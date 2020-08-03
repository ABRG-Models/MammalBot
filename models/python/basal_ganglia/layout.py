# Layout for Dash visualisation of BG data

# Dash components
import dash_bootstrap_components as dbc
import dash_core_components as dcc
import dash_html_components as html

import plotly.graph_objs as go

# Basal ganglia model
# from model import BasalGanglia

# Fixed BG values
BG_CHANNELS = 5

# Plot attributes
PLOT_LENGTH = 100
PLOT_COLOURS = [
	'crimson',      # CH1
	'steelblue',    # CH2
	'seagreen',     # CH3
	'purple',       # CH4
	'darkorange',   # CH5
	'sienna'        # CH6
]
PLOT_GAP = 0.02
PLOT_SHOWGRIDX = True
PLOT_SHOWGRIDY = False

# Initialise input slider controls
# TODO: Make shorter and spaced further apart
dash_controls = {
	'Input': [
		dcc.Slider(
			id='input-' + str(ch),
			min=0,
			max=1,
			step=0.01,
			updatemode='drag',
			value=0,
			vertical=True,
			className='float-left',
			verticalHeight=120
		)
		for ch in range(BG_CHANNELS)
	],
	'LH': [
		dcc.Slider(
			id='lh-' + str(ch),
			min=0,
			max=1,
			step=0.01,
			updatemode='drag',
			value=0,
			vertical=True,
			className='float-left',
			verticalHeight=120
		)
		for ch in range(BG_CHANNELS)
	],
}

# Graph objects
dash_graphs = {
	'Input':  dcc.Graph(
		id='input-graph',
		config={'displayModeBar': False},
	),
	'Ventral': dcc.Graph(
		id='ventral-graph',
		config={'displayModeBar': False},
		# style={
		# 	'height': '400px',
		# 	'width' : '100%',
		# }
	),
	'Dorsal': dcc.Graph(
		id='dorsal-graph',
		config={'displayModeBar': False},
		# style={
		# 	'height': '400px',
		# 	'width' : '100%',
		# }
	),
}

# Update intervals
dash_intervals = html.Div([
	dcc.Interval(
		id='interval-fast',
		# Too short an interval causes issues as not all plots can be updated before the next callback
		interval=0.1 * 1000,
		n_intervals=0
	),
])

# Graph layouts
dash_layouts = {
	'Input': go.Layout(
		legend={
			'orientation': 'v',
			'x'          : 1,
			'xanchor'    : 'right',
			'y'          : 1,
			'yanchor'    : 'top',
		},
		showlegend=True,
		margin={
			'b': 20,
			'l': 20,
			'r': 0,
			't': 0
		},
		xaxis={
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			'title'         : 'Time',
			'zeroline'      : True
		},
		yaxis={
			'fixedrange'    : True,
			'range'         : [0, 1],
			'showgrid'      : PLOT_SHOWGRIDY,
			'showticklabels': False,
			'title'         : 'Activation',
			'zeroline'      : True
		}
	),
	'Ventral': go.Layout(
		annotations=[
			{
				'showarrow': False,
				'text'     : 'Striatal dMSNs',
				'x'        : (0.5 - PLOT_GAP) / 2,
				'y'        : 1.0,
				'xanchor'  : 'center',
				'yanchor'  : 'bottom',
				'xref'     : 'paper',
				'yref'     : 'paper',
			},
			{
				'showarrow': False,
				'text'     : 'Striatal iMSNs',
				'x'        : 1 - ((0.5 - PLOT_GAP) / 2),
				'y'        : 1.0,
				'xanchor'  : 'center',
				'yanchor'  : 'bottom',
				'xref'     : 'paper',
				'yref'     : 'paper',
			},
			{
				'showarrow': False,
				'text'     : 'Ventral tegmental area',
				'x'        : (0.5 - PLOT_GAP) / 2,
				'y'        : 0.82,
				'xanchor'  : 'center',
				'yanchor'  : 'bottom',
				'xref'     : 'paper',
				'yref'     : 'paper',
			},
			{
				'showarrow': False,
				'text'     : 'Ventral pallidum',
				'x'        : 1 - ((0.5 - PLOT_GAP) / 2),
				'y'        : 0.82,
				'xanchor'  : 'center',
				'yanchor'  : 'bottom',
				'xref'     : 'paper',
				'yref'     : 'paper',
			},
			{
				'showarrow': False,
				'text'     : 'Pedunculopontine nucleus',
				'x'        : 0.5,
				'y'        : 0.64,
				'xanchor'  : 'center',
				'yanchor'  : 'bottom',
				'xref'     : 'paper',
				'yref'     : 'paper',
			},
		],
		showlegend=False,
		margin={
			'b': 5,
			'l': 0,
			'r': 0,
			't': 20
		},
		# dMSN
		xaxis1={
			'anchor'        : 'y1',
			'domain'        : [0, 0.5 - PLOT_GAP],
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			# 'title'         : 'Time',
			'zeroline'      : True
		},
		# iMSN
		xaxis2={
			'anchor'        : 'y1',
			'domain'        : [0.5 + PLOT_GAP, 1],
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			# 'title'         : 'Time',
			'zeroline'      : True
		},
		# VTA
		xaxis3={
			'anchor'        : 'y2',
			'domain'        : [0, 0.5 - PLOT_GAP],
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			# 'title'         : 'Time',
			'zeroline'      : True
		},
		# Pal
		xaxis4={
			'anchor'        : 'y2',
			'domain'        : [0.5 + PLOT_GAP, 1],
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			# 'title'         : 'Time',
			'zeroline'      : True
		},
		# PPn
		xaxis5={
			'anchor'        : 'y3',
			'domain'        : [0, 1],
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			# 'title'         : 'Time',
			'zeroline'      : True
		},
		# All MSNs
		yaxis1={
			'anchor'        : 'x1',
			'domain'        : [0.9, 1],
			'fixedrange'    : True,
			'range'         : [0, 1.05],
			'showgrid'      : PLOT_SHOWGRIDY,
			'showticklabels': False,
			# 'title'         : 'Activation',
			'zeroline'      : True
		},
		# VTA & Pal
		yaxis2={
			'anchor'        : 'x3',
			'domain'        : [0.72, 0.82],
			'fixedrange'    : True,
			'range'         : [0, 1.05],
			'showgrid'      : PLOT_SHOWGRIDY,
			'showticklabels': False,
			# 'title'         : 'Activation',
			'zeroline'      : True
		},
		#PPn
		yaxis3={
			'anchor'        : 'x5',
			'domain'        : [0.54, 0.64],
			'fixedrange'    : True,
			'range'         : [0, 1.05],
			'showgrid'      : PLOT_SHOWGRIDY,
			'showticklabels': False,
			# 'title'         : 'Activation',
			'zeroline'      : True
		},
	),
	'Dorsal': go.Layout(
		annotations=[
			{
				'showarrow': False,
				'text'     : 'Striatal dMSNs',
				'x'        : (0.5 - PLOT_GAP) / 2,
				'y'        : 1.0,
				'xanchor'  : 'center',
				'yanchor'  : 'bottom',
				'xref'     : 'paper',
				'yref'     : 'paper',
			},
			{
				'showarrow': False,
				'text'     : 'Striatal iMSNs',
				'x'        : 1 - ((0.5 - PLOT_GAP) / 2),
				'y'        : 1.0,
				'xanchor'  : 'center',
				'yanchor'  : 'bottom',
				'xref'     : 'paper',
				'yref'     : 'paper',
			},
			{
				'showarrow': False,
				'text'     : 'Subthalamic nucleus',
				'x'        : 0.5,
				'y'        : 0.82,
				'xanchor'  : 'center',
				'yanchor'  : 'bottom',
				'xref'     : 'paper',
				'yref'     : 'paper',
			},
			{
				'showarrow': False,
				'text'     : 'Substantia nigra pars reticulata',
				'x'        : (0.5 - PLOT_GAP) / 2,
				'y'        : 0.64,
				'xanchor'  : 'center',
				'yanchor'  : 'bottom',
				'xref'     : 'paper',
				'yref'     : 'paper',
			},
			{
				'showarrow': False,
				'text'     : 'Globus pallidus (external)',
				'x'        : 1 - ((0.5 - PLOT_GAP) / 2),
				'y'        : 0.64,
				'xanchor'  : 'center',
				'yanchor'  : 'bottom',
				'xref'     : 'paper',
				'yref'     : 'paper',
			},
			{
				'showarrow': False,
				'text'     : 'Thalamus',
				'x'        : (0.5 - PLOT_GAP) / 2,
				'y'        : 0.46,
				'xanchor'  : 'center',
				'yanchor'  : 'bottom',
				'xref'     : 'paper',
				'yref'     : 'paper',
			},
			{
				'showarrow': False,
				'text'     : 'Thalamic retiuclar nucleus',
				'x'        : 1 - ((0.5 - PLOT_GAP) / 2),
				'y'        : 0.46,
				'xanchor'  : 'center',
				'yanchor'  : 'bottom',
				'xref'     : 'paper',
				'yref'     : 'paper',
			},
			{
				'showarrow': False,
				'text'     : 'Motor cortex',
				'x'        : 0.5,
				'y'        : 0.28,
				'xanchor'  : 'center',
				'yanchor'  : 'bottom',
				'xref'     : 'paper',
				'yref'     : 'paper',
			},
			{
				'showarrow': False,
				'text'     : 'Dopamine',
				'x'        : 0.5,
				'y'        : 0.1,
				'xanchor'  : 'center',
				'yanchor'  : 'bottom',
				'xref'     : 'paper',
				'yref'     : 'paper',
			},
		],
		showlegend=False,
		margin={
			'b': 5,
			'l': 0,
			'r': 0,
			't': 20
		},
		# dMSN
		xaxis1={
			'anchor'        : 'y1',
			'domain'        : [0, 0.5 - PLOT_GAP],
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			# 'title'         : 'Time',
			'zeroline'      : True
		},
		# iMSN
		xaxis2={
			'anchor'        : 'y1',
			'domain'        : [0.5 + PLOT_GAP, 1],
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			# 'title'         : 'Time',
			'zeroline'      : True
		},
		# STN
		xaxis3={
			'anchor'        : 'y2',
			'domain'        : [0, 1],
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			# 'title'         : 'Time',
			'zeroline'      : True
		},
		# SNr
		xaxis4={
			'anchor'        : 'y3',
			'domain'        : [0, 0.5 - PLOT_GAP],
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			# 'title'         : 'Time',
			'zeroline'      : True
		},
		# GPe
		xaxis5={
			'anchor'        : 'y3',
			'domain'        : [0.5 + PLOT_GAP, 1],
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			# 'title'         : 'Time',
			'zeroline'      : True
		},
		# Thal
		xaxis6={
			'anchor'        : 'y4',
			'domain'        : [0, 0.5 - PLOT_GAP],
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			# 'title'         : 'Time',
			'zeroline'      : True
		},
		# TRN
		xaxis7={
			'anchor'        : 'y4',
			'domain'        : [0.5 + PLOT_GAP, 1],
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			# 'title'         : 'Time',
			'zeroline'      : True
		},
		# Ctx
		xaxis8={
			'anchor'        : 'y5',
			'domain'        : [0, 1],
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			# 'title'         : 'Time',
			'zeroline'      : True
		},
		# DA
		xaxis9={
			'anchor'        : 'y6',
			'domain'        : [0, 1],
			'fixedrange'    : True,
			'range'         : [0, PLOT_LENGTH],
			'showgrid'      : PLOT_SHOWGRIDX,
			'showticklabels': False,
			# 'title'         : 'Time',
			'zeroline'      : True
		},
		# All MSNs
		yaxis1={
			'anchor'        : 'x1',
			'domain'        : [0.9, 1],
			'fixedrange'    : True,
			'range'         : [0, 1.05],
			'showgrid'      : PLOT_SHOWGRIDY,
			'showticklabels': False,
			# 'title'         : 'Activation',
			'zeroline'      : True
		},
		# STN
		yaxis2={
			'anchor'        : 'x3',
			'domain'        : [0.72, 0.82],
			'fixedrange'    : True,
			'range'         : [0, 1.05],
			'showgrid'      : PLOT_SHOWGRIDY,
			'showticklabels': False,
			# 'title'         : 'Activation',
			'zeroline'      : True
		},
		# SNr & GPe
		yaxis3={
			'anchor'        : 'x4',
			'domain'        : [0.54, 0.64],
			'fixedrange'    : True,
			'range'         : [0, 1.05],
			'showgrid'      : PLOT_SHOWGRIDY,
			'showticklabels': False,
			# 'title'         : 'Activation',
			'zeroline'      : True
		},
		# Thal & TRN
		yaxis4={
			'anchor'        : 'x6',
			'domain'        : [0.36, 0.46],
			'fixedrange'    : True,
			'range'         : [0, 1.05],
			'showgrid'      : PLOT_SHOWGRIDY,
			'showticklabels': False,
			# 'title'         : 'Activation',
			'zeroline'      : True
		},
		# Ctx
		yaxis5={
			'anchor'        : 'x8',
			'domain'        : [0.18, 0.28],
			'fixedrange'    : True,
			'range'         : [0, 1.05],
			'showgrid'      : PLOT_SHOWGRIDY,
			'showticklabels': False,
			# 'title'         : 'Activation',
			'zeroline'      : True
		},
		# DA
		yaxis6={
			'anchor'        : 'x9',
			'domain'        : [0, 0.1],
			'fixedrange'    : True,
			'range'         : [0, 1.05],
			'showgrid'      : PLOT_SHOWGRIDY,
			'showticklabels': False,
			# 'title'         : 'Activation',
			'zeroline'      : True
		},
	),
}

# Link BG graphs to specific layout axes
bg_ax = {
	'Ventral': {
		'dMSN': {
			'x': 'x1',
			'y': 'y1',
		},
		'iMSN': {
			'x': 'x2',
			'y': 'y1',
		},
		'VTA': {
			'x': 'x3',
			'y': 'y2',
		},
		'Pal': {
			'x': 'x4',
			'y': 'y2',
		},
		'PPn': {
			'x': 'x5',
			'y': 'y3',
		},
	},
	'Dorsal': {
		'dMSN': {
			'x': 'x1',
			'y': 'y1',
		},
		'iMSN': {
			'x': 'x2',
			'y': 'y1',
		},
		'STN' : {
			'x': 'x3',
			'y': 'y2',
		},
		'SNr': {
			'x': 'x4',
			'y': 'y3',
		},
		'GPe' : {
			'x': 'x5',
			'y': 'y3',
		},
		'Thal': {
			'x': 'x6',
			'y': 'y4',
		},
		'TRN': {
			'x': 'x7',
			'y': 'y4',
		},
		'Ctx': {
			'x': 'x8',
			'y': 'y5',
		},
		'DA': {
			'x': 'x9',
			'y': 'y6',
		},
		# TEMP
		'LH_APPROACH': {
			'x': 'x7',
			'y': 'y4',
		},
		'LH_AVOID': {
			'x': 'x7',
			'y': 'y4',
		}
	}
}

# Page layout
dash_rows = {
	'Input': dbc.Row(
		dbc.Col(
			dbc.Card(
				[
					dbc.CardHeader(
						['Input'],
						className='bg-primary font-weight-bold lead'
					),
					dbc.CardBody([
						dash_graphs['Input'],
						html.Div(dash_controls['Input']),
						html.Div(dash_controls['LH']),
					]),
				],
				color='primary',
				inverse=True,
				outline=True,
			),
		),
	),
	'Output': dbc.Row([
		dbc.Col(
			dbc.Card(
				[
					dbc.CardHeader(
						'Ventral BG',
						className='bg-info font-weight-bold lead'
					),
					dbc.CardBody(dash_graphs['Ventral']),
				],
				color='info',
				inverse=True,
				outline=True,
			),
		),
		dbc.Col(
			dbc.Card(
				[
					dbc.CardHeader(
						'Dorsal BG',
						className='bg-success font-weight-bold lead'
					),
					dbc.CardBody(dash_graphs['Dorsal']),
				],
				color='success',
				inverse=True,
				outline=True,
			),
		),
	]),
}
