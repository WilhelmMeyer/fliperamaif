import threading
import serial
import time
import game_state
import score_db

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)

def processar_linha(linha):
    linha = linha.strip()
    
    if linha.startswith("SCORE"):
        if game_state.bola_em_jogo and game_state.jogo_ativo:
            try:
                msg=linha.split(":")
                ovelha=msg[1]
                valor = int(msg[2])
                game_state.pontuacao_atual += valor
                print(f"[Ovelha {ovelha} +{valor} pts] Total: {game_state.pontuacao_atual}")
            except:
                print("Erro ao interpretar SCORE")
        else:
            print(f"[IGNORADO] SCORE recebido sem bola em jogo: {linha}")

    elif linha == "NEW_BALL":
        if game_state.jogo_ativo:
            game_state.bola_em_jogo = True
            print("[NOVA BOLA] Bola em jogo!")

    elif linha == "BALL_OUT":
        if game_state.bola_em_jogo:
            game_state.vidas_restantes -= 1
            game_state.bola_em_jogo = False
            print(f"[BOLA PERDIDA] Vidas restantes: {game_state.vidas_restantes}")
            if game_state.vidas_restantes <= 0:
                game_state.jogo_ativo = False
                print("[FIM DE JOGO]")

def escutar_arduino():
    print("Escutando Arduino...")
    while True:
        if arduino.in_waiting:
            linha = arduino.readline().decode()
            processar_linha(linha)
        time.sleep(0.05)

def iniciar_listener():
    t = threading.Thread(target=escutar_arduino, daemon=True)
    t.start()
