from utils import GoslingAgent, np_to_rlbot, magnitude
from strategy import Brain

class ApexBot(GoslingAgent):
    """
    Agente ApexBot para Rocket League.
    Combina as melhores lógicas de diversos bots em uma arquitetura refinada.
    Utiliza NumPy para todos os cálculos matemáticos e uma classe Brain para estratégia.
    """
    def initialize_agent(self):
        # Chama a inicialização da classe base GoslingAgent
        super().initialize_agent()
        # Instancia o cérebro estratégico
        self.brain = Brain(self)

    def run(self):
        """
        Método principal executado a cada tick do jogo.
        Responsável por depuração visual e delegação da estratégia ao Brain.
        """
        # 1. Renderização de Depuração (Debug)
        # Exibe a velocidade atual do carro acima dele no mundo 3D
        velocity_mag = magnitude(self.me.velocity)
        self.renderer.draw_string_3d(
            np_to_rlbot(self.me.location),
            2, 2,
            f"Speed: {round(velocity_mag, 1)}",
            self.renderer.white()
        )

        # Exibe a rotina atual no topo da pilha para acompanhamento
        if len(self.stack) > 0:
            current_routine = self.stack[-1].__class__.__name__
            self.renderer.draw_string_2d(10, 50, 2, 2, f"Routine: {current_routine}", self.renderer.yellow())
        else:
            self.renderer.draw_string_2d(10, 50, 2, 2, "Strategy: Idle", self.renderer.cyan())

        # 2. Execução da Estratégia
        # O Brain decide se deve adicionar novas rotinas à pilha (self.stack)
        self.brain.execute()

        # 3. Linha de alvo (se houver uma rotina ativa com alvo)
        if len(self.stack) > 0 and hasattr(self.stack[-1], 'target'):
            try:
                target_loc = self.stack[-1].target
                if target_loc is not None:
                    self.line(self.me.location, target_loc, [0, 255, 0])
            except:
                pass
