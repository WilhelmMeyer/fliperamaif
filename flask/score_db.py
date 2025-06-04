import sqlite3

DB_PATH = "scores.db"

def inicializar_banco():
    with sqlite3.connect(DB_PATH) as conn:
        cursor = conn.cursor()
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS scores (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                nome TEXT NOT NULL,
                pontos INTEGER NOT NULL
            )
        ''')
        conn.commit()

def salvar_score(nome, pontos):
    with sqlite3.connect(DB_PATH) as conn:
        cursor = conn.cursor()
        cursor.execute('INSERT INTO scores (nome, pontos) VALUES (?, ?)', (nome[:10], pontos))
        conn.commit()

def obter_top_scores(limit=5):
    with sqlite3.connect(DB_PATH) as conn:
        cursor = conn.cursor()
        cursor.execute('SELECT nome, pontos FROM scores ORDER BY pontos DESC LIMIT ?', (limit,))
        return cursor.fetchall()
