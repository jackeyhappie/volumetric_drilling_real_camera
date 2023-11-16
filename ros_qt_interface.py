from __future__ import annotations
import time
from typing import List, Tuple
from PyQt5.QtCore import  Qt
from PyQt5.QtCore import QSize
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QVBoxLayout, QWidget, QPushButton
from PyQt5.QtWidgets import QComboBox, QSlider, QGridLayout, QTextEdit
import sys
from dataclasses import dataclass

import rospy
from sensor_msgs.msg import Joy
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import Int32
from std_msgs.msg import Float32


@dataclass
class CustomSlider:
    controller: SlidersBlock
    label: str
    range: Tuple[int, int]
    init_val: float
    max_val: float

    def __post_init__(self):
        self.slider_label = QLabel(self.label+f"{self.init_val:05.2f}") 
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(self.range[0], self.range[1])
        self.slider.setSingleStep(1)

        self.slider.setValue(int(self.init_val/self.max_val*self.range[1]))

        self.slider.valueChanged.connect(self.slider_cb)

    def slider_cb(self, i):
        val = self.max_val*(i/self.range[1])
        self.slider_label.setText(self.label+f"{val:05.2f}") 

        self.controller.publish_dist_params()
    
    def disable_slider(self):
        self.slider.setEnabled(False)
    
    def enable_slider(self):
        self.slider.setEnabled(True)
    
    def get_current_value(self):
        i = self.slider.value()
        cur_val = self.max_val*(i/self.range[1])
        return cur_val

class SlidersBlock:
    def __init__(self):
        self.grid_layout = QGridLayout()
        self.sliders_list: List[CustomSlider] = []

        self.sliders_list.append(CustomSlider(self, "a=", (0, 100), 7.0, 15.0))
        self.sliders_list.append(CustomSlider(self, "b=", (0, 100), 10.0, 20.0))
        self.sliders_list.append(CustomSlider(self, "c=", (0, 100),3, 10.0))
        self.sliders_list.append(CustomSlider(self, "eye_speration=", (0, 100),30, 70))

        for idx, custom_slider in enumerate(self.sliders_list):
            self.grid_layout.addWidget(custom_slider.slider_label, idx, 0)
            self.grid_layout.addWidget(custom_slider.slider, idx, 1)

    def add_publisher(self, publisher: ParamPublisher):
        self.publisher = publisher
    
    def enable_all_sliders(self):
        for slider in self.sliders_list:
            slider.enable_slider()
    
    def disable_all_sliders(self):
        for slider in self.sliders_list:
            slider.disable_slider()
    
    def publish_dist_params(self):
        a = self.sliders_list[0].get_current_value()
        b = self.sliders_list[1].get_current_value()
        c = self.sliders_list[2].get_current_value()
        d = self.sliders_list[3].get_current_value()
        self.publisher.pub_dist_params(a, b, c, d) 

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Parameter publisher")

        # QWigets config
        self.button = QPushButton("Publish parameters")
        self.button.setCheckable(True)
        self.button.clicked.connect(self.pub_button_cb)

        self.combo_label = QLabel("Operation mode")
        self.fmt_label(self.combo_label)
        self.combo_label.setFixedHeight(25)

        self.combo_box = QComboBox()
        self.combo_box.addItems(["mode 1", "mode 2", "mode 3", "mode 4"])
        self.combo_box.setEnabled(False)
        # self.combo_box.currentIndexChanged.connect( self.index_changed )
        self.combo_box.currentTextChanged.connect( self.text_changed )

        self.label = QLabel("Operation mode")

        self.input = QLineEdit()
        self.input.textChanged.connect(self.label.setText)

        # Create a QGridLayout
        self.dist_label = QLabel("Distortion params")
        self.fmt_label(self.dist_label)
        self.dist_label.setFixedHeight(25)

        self.slider_block = SlidersBlock()
        self.slider_block.disable_all_sliders()

        self.text_edit_label = QLabel("Console")
        self.fmt_label(self.text_edit_label)
        self.text_edit = QTextEdit()
        self.text_edit.setReadOnly(True)

        # Layout config
        layout = QVBoxLayout()
        layout.addWidget(self.button)
        layout.addWidget(self.combo_label)
        layout.addWidget(self.combo_box)
        layout.addWidget(self.dist_label)
        layout.addLayout(self.slider_block.grid_layout)
        layout.addWidget(self.text_edit_label)
        layout.addWidget(self.text_edit)

        container = QWidget()
        container.setLayout(layout)

        # Window config.
        self.setCentralWidget(container)
        self.setFixedSize(QSize(400, 300))

    def fmt_label(self, widget):
        font = widget.font()
        font.setBold(True)
        widget.setFont(font)

    def pub_button_cb(self):
        # Init param publishers
        self.param_pub = ParamPublisher()
        time.sleep(0.2)

        self.slider_block.add_publisher(self.param_pub)

        self.button.setText("publishers activated")
        self.button.setEnabled(False)
        self.button.setStyleSheet("QPushButton:checked { background-color: green; }")

        self.combo_box.setEnabled(True)
        self.slider_block.enable_all_sliders()

        self.text_edit.append("Publishers activated")
        self.text_edit.append("Publishing to topic: /volumetric/camera_params")

    # def index_changed(self, i): # i is an int
    #     print(i)

    def text_changed(self, selected_mode): # s is a str
        self.param_pub.pub_op_mode(selected_mode)
        print(selected_mode)
        

class ParamPublisher:
    def __init__(self) -> None:
        base_topic = "/volumetric/camera_params/"
        #Operation mode publisher 
        #self.mode_pub = rospy.Publisher(base_topic+'op_mode', KeyValue, queue_size=10)
        #self.dist_pub = rospy.Publisher(base_topic+'dist_params1', Joy, queue_size=10)
        self.mode_pub = rospy.Publisher(base_topic+'op_mode', Int32, queue_size=10)
        self.dist_pub1 = rospy.Publisher(base_topic+'dist_params1', Float32, queue_size=10)
        self.dist_pub2 = rospy.Publisher(base_topic+'dist_params2', Float32, queue_size=10)
        self.dist_pub3 = rospy.Publisher(base_topic+'dist_params3', Float32, queue_size=10)
        self.dist_pub4 = rospy.Publisher(base_topic+'dist_params4', Float32, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
    
    def pub_op_mode(self, mode):
        number=int(mode[-1])
        message = Int32()
        message.data = number
        self.mode_pub.publish(message)
        #self.mode_pub.publish(KeyValue(key="op_mode", value=mode))
    
    def pub_dist_params(self, a, b, c, d):
        #self.dist_pub.publish(Joy(header=None, axes=[a, b, c], buttons=[]))
        self.dist_pub1.publish(a)
        self.dist_pub2.publish(b)
        self.dist_pub3.publish(c)
        self.dist_pub4.publish(d)

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec()

if __name__ == "__main__":
    main()