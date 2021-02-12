import sys
import os
path = os.path.dirname(__file__)
path = "/".join(path.split("\\")[:-1])
sys.path.append(path)
from robot.utils import *

class Scene1(object):
    def __init__(self):
        super().__init__()

class Scene2(object):
    def __init__(self):
        super().__init__()

class Scene3(object):
    def __init__(self):
        super().__init__()
