# ApexBot - Revisão Técnica Completa

## Estrutura do Projeto
- `ApexBot/`
  - `appearance.cfg`: Configurações de aparência do bot (Octane ID 23).
  - `bot.cfg`: Metadados e definições do RLBot.
  - `bot.py`: Agente principal e lógica de renderização.
  - `routines.py`: Módulos de mecânicas e tarefas (Jump, Aerial, SpeedFlip).
  - `strategy.py`: Cérebro estratégico (Brain class).
  - `tools.py`: Ferramentas de análise e interceptação.
  - `utils.py`: Matemática vetorial com NumPy e classes de objetos.
- `README.md`: Documentação técnica.
- `requirements.txt`: Manifest de dependências.
- `test_bot_logic.py`: Script de validação lógica.

## Changelog
- `ApexBot/bot.py`: Migração para o sistema de `Brain`. Conversão de tipos para renderização via `np_to_rlbot`.
- `ApexBot/routines.py`: Refatoração total para NumPy. Implementação de `speed_flip`, `aerial` aprimorado e lógica de recuperação em paredes.
- `ApexBot/strategy.py`: Implementação inédita da estratégia híbrida modular.
- `ApexBot/tools.py`: Atualização de `find_hits` para NumPy. Inclusão de `intercept_race`.
- `ApexBot/utils.py`: Substituição de classes customizadas por NumPy. Adição de funções `angle_between`, `detect_demo` e `backsolve`.

## Lista de Bugs Corrigidos
- `ApexBot/bot.py`, Função `run`: Corrigido erro de renderização onde arrays NumPy causavam falha no framework RLBot.
- `ApexBot/routines.py`, Classe `aerial`: Corrigida falha na estimativa de aceleração que ignorava a gravidade acumulada.
- `ApexBot/tools.py`, Função `find_hits`: Corrigido erro de "aliasing" na busca de fatias (slices) que perdia janelas de interceptação.
- `ApexBot/utils.py`, Função `defaultPD`: Corrigido erro de casting do NumPy ao operar vetores de ponto flutuante com escalares inteiros.
- `ApexBot/utils.py`, Classe `car_object.update`: Corrigido erro na orientação da velocidade angular local.

## Arquivos Especiais
- `ApexBot/appearance.cfg`: Arquivo de configuração de aparência (mantido íntegro).
- `ApexBot/bot.cfg`: Arquivo de metadados do RLBot (mantido íntegro).
