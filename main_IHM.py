####################################################################
#                       BEI EasyMile                               #
#   Moez CHAGRAOUI, Rayen YADIR, Yassine ABDELILLAH, Drissa SAGNON #
####################################################################
# main_IHM.py

import sys
from IHM.IHM import *

if __name__ == "__main__":
    app = QApplication(sys.argv)
    widget = StarterCode()
    widget.show()
    sys.exit(app.exec())

