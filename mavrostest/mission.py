from mavrostest.flight_control import FlightControl
from mavrostest.visual import Visual

class Mission:
    def __init__(self, controller:FlightControl) -> None:
        self.controller = controller

    def landedOnPlatform(self):
        
