from flask import Flask, render_template, request
from datetime import datetime
from flask import Flask, render_template, request, jsonify
import arduino_listener
import game_state
import score_db
import logging


log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__)

score_db.inicializar_banco()
arduino_listener.iniciar_listener()

@app.route('/')
def index():
    agora = datetime.now().strftime("%H:%M:%S")
    return render_template('index.html', hora=agora)

@app.route('/status')
def status():
    return jsonify({
        "pontuacao": game_state.pontuacao_atual,
        "vidas": game_state.vidas_restantes,
        "bola_em_jogo": game_state.bola_em_jogo,
        "jogo_ativo": game_state.jogo_ativo
    })

@app.route('/reset', methods=['POST'])
def reset():
    game_state.reset_jogo()
    return "Jogo reiniciado com sucesso."

@app.route('/salvar_score', methods=['POST'])
def salvar_score():
    data = request.get_json()
    nome = data.get("nome", "").strip()[:10].upper()
    pontos = game_state.pontuacao_atual
    score_db.salvar_score(nome, pontos)
    return jsonify({"status": "ok", "salvo": True, "nome": nome, "pontos": pontos})

@app.route('/hall')
def hall():
    top_scores = score_db.obter_top_scores()
    return jsonify(top_scores)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
