from utils import *
from routines import *
from tools import *
from strategy import Brain

class ApexBot(GoslingAgent):
    def initialize_agent(self):
        super().initialize_agent()
        self.brain = Brain()

    def run(self):
        # Debug drawing
        self.renderer.draw_string_3d(self.me.location, 2, 2, f"Speed: {round(self.me.velocity.magnitude(), 1)}", self.renderer.white())

        # If no routine is active, decide on the next one
        if len(self.stack) < 1:
            self.brain.run(self)
