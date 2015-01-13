#!/usr/bin/env python
import sys
import signal
from PyQt4 import QtGui, QtCore

HEIGHT = 500
WIDTH = 500

class Main(QtGui.QMainWindow):

    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        self.initUI()

    def initUI(self):
        
        '''Window Settings'''
        screen = QtGui.QDesktopWidget().screenGeometry()
        self.setGeometry((screen.width() - self.geometry().width())/2,
                (screen.height() - self.geometry().height())/2,  WIDTH,
                HEIGHT)
        self.setWindowTitle('I see what you did there')


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal.SIG_DFL) #bypass any finally blocks
    app = QtGui.QApplication(sys.argv)
    main = Main()
    main.show()
    sys.exit(app.exec_())
