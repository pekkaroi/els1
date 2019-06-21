import sys
from PyQt4 import QtGui, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import QTimer
import serial
import serial.tools.list_ports
import threading
import queue
import time
modes = ["BASIC", "THREAD", "INDEX"]
statuses = ["IDLE", "PENDING", "JOG", "ACCELERATE", "FOLLOW", "DECELERATE", "DONE", "RETURN"]
STEPS_PER_MM = 200
class SerialThread(threading.Thread):
    def __init__(self, queue, writequeue, ser):
        threading.Thread.__init__(self)
        self.ser = ser
        self.queue = queue
        self.writequeue = writequeue
        self.buffer = ""

    def stop(self):
        self.running = False

    def run(self):
        self.running = True
        while self.running:
            while self.ser.inWaiting():
                ch = self.ser.read(1)


                if ch == b'\n' or ch == b'\r':

                    self.queue.put(self.buffer)
                    self.buffer = ""
                else:
                    self.buffer += ch.decode("utf-8")
            while self.writequeue.qsize():
                try:
                    line = self.writequeue.get()
                    self.ser.write(bytearray(line, "utf-8"))
                except queue.Empty:
                    pass


class MyWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()
        uic.loadUi('mainwindow.ui', self)
        self.btnSetPitch.clicked.connect(self.setPitch)
        self.btnSetLength.clicked.connect(self.setLength)
        self.btnSetRefPos.clicked.connect(self.setRefPos)
        self.btnSTART.clicked.connect(self.sendStart)
        self.btnSTARTBasic.clicked.connect(self.sendStart)
        self.btnSTOP.clicked.connect(self.sendStop)
        self.btnRETURN.clicked.connect(self.sendReturn)
        self.btnOpen.clicked.connect(self.openSerial)
        self.tabWidget.currentChanged.connect(self.tabChanged)
        self.queue = queue.Queue()
        self.writequeue = queue.Queue()

        ports = self.getComPorts()
        for p in ports:
            self.serPort.addItem(p)
        self.show()

    def tabChanged(self, index):


        st = "SET_MODE," + modes[index]+",\n"
        print(st)
        self.writequeue.put(st)

    def setPitch(self):
        pitch, ok = QInputDialog.getDouble(self, "select pitch", "enter a number")
        st = "SET_PITCH,{},\n".format(pitch)
        self.writequeue.put(st)

    def setLength(self):
        leng, ok = QInputDialog.getDouble(self, "select length", "enter a number")
        st = "SET_LENGTH,{},\n".format(leng)
        self.writequeue.put(st)
    def setRefPos(self):
        offs, ok = QInputDialog.getDouble(self, "select offset from current pos", "enter offset")
        st = "SET_REF_POS,{},\n".format(offs)
        self.writequeue.put(st)
    def sendStart(self):
        self.writequeue.put("START,\n")
    def sendReturn(self):
        self.writequeue.put("RETURN,\n")
    def sendStop(self):
        self.writequeue.put("STOP,\n")

    def openSerial(self):
        try:
            self.ser = serial.Serial(self.serPort.currentText(), 115200, timeout=0)
        except serial.SerialException:
            print ("unable to open")
            return

        self.thread = SerialThread(self.queue, self.writequeue, self.ser)
        self.thread.daemon = True
        self.thread.start()
        self.process_serial()

    def process_serial(self):
        while self.queue.qsize():
            try:
                line = self.queue.get()
                self.handleLine(line)
            except queue.Empty:
                pass
        #self.after(100, self.process_serial)

        QTimer.singleShot(100, self.process_serial)

    def closeSerial(self):
        pass
    def handleLine(self, line):
        vals = line.split(',')
        if len(vals) > 4:
            mode = int(vals[0])
            if mode != self.tabWidget.currentIndex():
                self.tabWidget.currentChanged.disconnect()
                self.tabWidget.setCurrentIndex(mode)
                self.tabWidget.currentChanged.connect(self.tabChanged)
            status = int(vals[1])
            self.lblStatus.setText(statuses[status])
            self.lblPosition.setText(str((int(vals[2])-int(vals[6])) / STEPS_PER_MM))
            self.lblPitch.setText(vals[3])
            self.lblDistance.setText(vals[4])
            self.lblRPM.setText(str(float(vals[5]) * 60.0))
    def getComPorts(self):
        ports = serial.tools.list_ports.comports()
        portNames = []
        for port in ports:
            portNames.append(port[0])
        return portNames


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    sys.exit(app.exec_())
