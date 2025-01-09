import sys

from IHM.IHM import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *


if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = StarterCode()
    widget.show()
    sys.exit(app.exec())