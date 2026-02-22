from utils import *
from routines import *
from tools import *
from strategy import Brain

class ApexBot(GoslingAgent):
    def initialize_agent(self):
        super().initialize_agent()
        self.brain = Brain(self)

    def run(self):
        # Debug drawing
        loc = RLBotVector3(self.me.location[0], self.me.location[1], self.me.location[2])
        self.renderer.draw_string_3d(loc, 2, 2, f"Speed: {round(magnitude(self.me.velocity), 1)}", self.renderer.white())

        # If no routine is active, decide on the next one
        if len(self.stack) < 1:
            self.brain.execute()
