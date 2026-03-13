import sys
import os
import numpy as np

# Adiciona o diretório ApexBot ao path para poder importar os módulos
sys.path.append(os.path.join(os.getcwd(), 'ApexBot'))

def test_math_utils():
    print("Testando utilitários matemáticos...")
    import utils

    # Teste de magnitude
    v1 = np.array([3, 4, 0])
    assert utils.magnitude(v1) == 5.0
    print("  - magnitude: OK")

    # Teste de normalize
    v2 = np.array([10, 0, 0])
    v2_n = utils.normalize(v2)
    assert np.array_equal(v2_n, np.array([1, 0, 0]))
    print("  - normalize: OK")

    # Teste de angle_between
    va = np.array([1, 0, 0])
    vb = np.array([0, 1, 0])
    angle = utils.angle_between(va, vb)
    assert np.isclose(angle, np.pi/2)
    print("  - angle_between: OK")

    # Teste de cap
    assert utils.cap(10, 0, 5) == 5
    assert utils.cap(-10, 0, 5) == 0
    assert utils.cap(3, 0, 5) == 3
    print("  - cap: OK")

def test_routines_instantiation():
    print("Testando instanciação de rotinas...")
    import routines

    try:
        r1 = routines.atba()
        r2 = routines.recovery()
        r3 = routines.goto(np.array([0,0,0]))
        r4 = routines.speed_flip(np.array([100,100,100]))
        print("  - Instanciação básica: OK")
    except Exception as e:
        print(f"  - Erro na instanciação: {e}")
        sys.exit(1)

def test_tools_logic():
    print("Testando lógica de ferramentas...")
    import tools
    # Apenas verifica se a função existe e pode ser importada
    assert callable(tools.find_hits)
    print("  - find_hits: OK")

if __name__ == "__main__":
    try:
        test_math_utils()
        test_routines_instantiation()
        test_tools_logic()
        print("\nTodos os testes lógicos passaram com sucesso!")
    except Exception as e:
        print(f"\nFalha nos testes: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
