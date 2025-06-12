pontuacao_atual = 0
vidas_restantes = 3
bola_em_jogo = False
jogo_ativo = True
nome_jogador = ""

def reset_jogo():
    global pontuacao_atual, vidas_restantes, bola_em_jogo, jogo_ativo
    pontuacao_atual = 0
    vidas_restantes = 3
    bola_em_jogo = False
    jogo_ativo = True