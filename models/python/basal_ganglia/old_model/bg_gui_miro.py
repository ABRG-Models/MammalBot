import sys
import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer, QElapsedTimer
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QGridLayout, QGroupBox, QMenu, QPushButton, QRadioButton, QVBoxLayout, QWidget, QSlider)
from bg_gurney import BasalGanglia


class Window(QWidget):
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)

        # Initialise data and plot structures
        self.data = {}
        self.plot = {}

        self.BG_PLOTS = ['Inp', 'dMSN', 'iMSN', 'PPn', 'VTA_SNc', 'DA', 'Ctx']
        self.BG_REGIONS = ['Ventral']
        # BG_PLOTS = ['Input', 'NAc', 'STNv', 'SNr', 'DM', 'PL']
        self.PLOT_LENGTH = 1000
        self.PLOT_COLOURS = ('r', 'g', 'b', 'c', 'm', 'y', 'w')


    def configure( self, model ):
         # Initialise Qt objects
        grid = QGridLayout()
        self.slider = {}
        # Initialise time pointer
        self.ptr = 0

        for n in range(model.BG_CHANNELS):
            grid.addWidget(self.create_input_sliders(n), 0, n)

        for x in range(len(self.BG_REGIONS)):
            for y in range(len(self.BG_PLOTS)):
                pop = self.BG_REGIONS[x] + '_' + self.BG_PLOTS[y]
                self.data[pop] = {}
                self.data[pop]['Region'] = self.BG_REGIONS[x]
                self.data[pop]['Population'] = self.BG_PLOTS[y]

                # TODO: Tidy this up
                if x is 0:
                    col = 0
                else:
                    col = 3

                grid.addWidget(self.create_plots(model, x, y), y + 1, col, 1, 2)    # TODO: Tidy up these columns a bit

                for i in range(model.BG_CHANNELS):
                    self.data[pop][i] = np.zeros(self.PLOT_LENGTH)

        # Set window layout
        self.setLayout(grid)
        self.setWindowTitle('Basal Ganglia')


    def create_input_sliders(self, ch):
        ch_id = 'CH ' + str(ch + 1)
        group_box = QGroupBox(ch_id)
        # groupBox.setAlignment(Qt.AlignCenter)

        self.slider[ch_id] = QSlider(Qt.Vertical)
        self.slider[ch_id].setTickPosition(QSlider.TicksBothSides)
        self.slider[ch_id].setMinimum(0)
        self.slider[ch_id].setMaximum(100)
        self.slider[ch_id].setMinimumSize(5, 100)
        self.slider[ch_id].valueChanged.connect(lambda value, ch=ch: self.change_inputs(value, ch))

        vbox = QVBoxLayout()
        vbox.addWidget(self.slider[ch_id])
        vbox.addStretch(1)
        group_box.setLayout(vbox)

        return group_box

    def create_plots(self, model, region, pop):
        plot_id = self.BG_REGIONS[region] + '_' + self.BG_PLOTS[pop]
        group_box = QGroupBox(plot_id)

        # Create and configure PlotWidget for each population
        plt = pg.PlotWidget()
        # TODO: Set xRange so 0 is at right and PLOT_LENGTH is at left
        plt.setRange(yRange=[0, 1])

        # Configure PlotItem for each channel
        self.plot[plot_id] = {}
        # self.plot[plot_id]['Region'] = BG_REGIONS[region]
        # self.plot[plot_id]['Population'] = BG_PLOTS[pop]
        for n in range(model.BG_CHANNELS):
            self.plot[plot_id][n] = plt.plot([], pen=pg.mkPen(self.PLOT_COLOURS[n], width=2))

        vbox = QVBoxLayout()
        vbox.addWidget(plt)
        # vbox.addStretch(1)
        group_box.setLayout(vbox)

        return group_box

    def change_inputs(self, val, ch):
        self.inputs[ch] = float(val) / 100

    def notify(self, model):

        if model.FIXED_INPUTS:
            curr_time = self.ptr / 100
            for c in model.BG_INPUTS.keys():
                if model.BG_INPUTS[c]['Onset'] <= curr_time < model.BG_INPUTS[c]['Offset']:
                    self.inputs[c] = model.BG_INPUTS[c]['Size']
                    # Transient
                    if 'Transient' in model.BG_INPUTS[c] and model.BG_INPUTS[c]['Transient']['Onset'] <= curr_time < model.BG_INPUTS[c]['Transient']['Offset']:
                        self.inputs[c] = model.BG_INPUTS[c]['Size'] + model.BG_INPUTS[c]['Transient']['Size']
                else:
                    self.inputs[c] = 0

        # Increment time pointer
        self.ptr += 1

        for p in self.plot.keys():
            reg = self.data[p]['Region']
            pop = self.data[p]['Population']

            for n in range(model.BG_CHANNELS):
                # Shift all data along by one
                self.data[p][n][:-1] = self.data[p][n][1:]

                if pop == 'Inp':
                    self.data[p][n][-1] = model.inputs[n].item()
                else:
                    # item() needed to convert numpy.float64 to native Python float
                    self.data[p][n][-1] = model.bg.pop[reg][pop]['o'][n].item()

                self.plot[p][n].setData(self.data[p][n])
                # self.plot[pop][n].setPos(self.ptr, 0)


# Main run loop
if __name__ == '__main__':
    app = QApplication([])
    clock = Window()
    clock.show()

    sys.exit(app.exec_())
