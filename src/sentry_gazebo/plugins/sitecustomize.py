import os
import sys

sys.path.insert(0, '/usr/lib/python3/dist-packages')
_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.append(_here)
