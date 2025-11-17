import sys
import os
import time

package_path = os.path.dirname(os.path.dirname(__file__))
sys.path.append(package_path)

from typing import Union

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.animation as animation

from threading import Thread, Lock, Event

# from libraries.bota.bota_serial import BotaSerialSensor
# from libraries.bota.bota_ethercat import BotaEtherCATSensor
import bota_driver


class Plotter:
    def __init__(self, sensor: bota_driver.BotaDriver | None = None):
        self.sensor = sensor
        self.window_size = 10  # in seconds
        self.animation_framerate = 100  # in Hz
        self.sensor_read_rate = 100

        # Create figure with a responsive layout
        self.fig = plt.figure(figsize=(20, 15), dpi=100)
        # Don't set tight_layout here, we'll manage it during resizing

        self._axs = self.fig.subplots(2, 1, sharex=True)
        plt.subplots_adjust(hspace=0.3)

        y_labels = ["Force (N)", "Torque (Nm)"]
        legends = [["fx", "fy", "fz"], ["mx", "my", "mz"]]
        self._lines = []
        for i in range(2):
            colors = ['r', 'g', 'b']
            self._axs[i].set_ylabel(y_labels[i])
            self._axs[i].set_xlabel("Time (s)")
            for j in range(3):
                self._axs[i].set_xlim(-self.window_size - 0.1, 0.1)
                self._axs[0].set_ylim(-20, 20)
                self._axs[1].set_ylim(-2, 2)
                line, = self._axs[i].plot([], [], lw=2, color=colors[j])
                self._lines.append(line)

            self._axs[i].legend(legends[i])

        # Use tight_layout for better initial layout
        # self.fig.tight_layout()

        self._ani = animation.FuncAnimation(self.fig, self.animate_func,
                                            init_func=self.init_func,
                                            frames=range(1, 200),
                                            interval=1 / self.animation_framerate * 1000,
                                            blit=True)

        self.buffer_length = int(self.window_size * self.sensor_read_rate)
        self.data_buffer = np.zeros((6, self.buffer_length))  # Shared array of doubles

        self.stop_reading_event = Event()
        self.shared_lock = Lock()

        # Process for reading hardware
        self.reader_thread = None

        # Canvas reference for resizing
        self.canvas = None

    def _read_sensor(self):
        while (not self.stop_reading_event.is_set()) and self.sensor:
            start_time = time.perf_counter()
            data = self.sensor.read_frame()
            if data is None:
                continue
            with self.shared_lock:
                self.data_buffer[0:3, 0] = data.force
                self.data_buffer[3:, 0] = data.torque
                self.data_buffer = np.roll(self.data_buffer, -1, 1)

            time.sleep(1 / self.sensor_read_rate)

    def run(self):
        if self.sensor:
            self.stop_reading_event.clear()
            self.reader_thread = Thread(target=self._read_sensor)
            self.reader_thread.start()
        else:
            print("[Plotter] No sensor added, nothing to plot")

    def stop(self):
        self.stop_reading_event.set()
        if self.reader_thread:
            self.reader_thread.join()
        self.sensor = None

    def add_sensor(self, sensor: bota_driver.BotaDriver):
        self.sensor = sensor

    def init_func(self):
        for i in range(2 * 3):
            self._lines[i].set_data([], [])
        return (*self._lines,)

    def animate_func(self, i):
        with self.shared_lock:
            x = np.linspace(-self.window_size, 0, self.buffer_length)
            min_limits = [0.5, 0.01]
            decimals = [2, 3]
            for i in range(2):
                max_val = np.max(self.data_buffer[3 * i:3 * i + 3, :])
                min_val = np.min(self.data_buffer[3 * i:3 * i + 3, :])
                self._axs[i].set_ylim(
                    np.round(min_val - 0.1 * max(min_limits[i], max_val - min_val), decimals=decimals[i]),
                    np.round(max_val + 0.1 * max(min_limits[i], max_val - min_val), decimals=decimals[i]))
                for j in range(3):
                    y = self.data_buffer[3 * i + j, :]
                    self._lines[3 * i + j].set_data(x, y)

        self.fig.canvas.draw()
        return (*self._lines,)
