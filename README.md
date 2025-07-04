# Fliperama com ESP32 e Raspberry Pi

Este projeto implementa um fliperama físico interativo utilizando um ESP32 para controle de hardware e um Raspberry Pi para a exibição da interface do jogo. O sistema é dividido entre tarefas em tempo real, sensores e motores físicos, e um servidor web para interface com o jogador.

## Descrição do Sistema

- O ESP32 é responsável pelo controle dos sensores de pontuação e dos motores de passo que acionam os flippers (aletas laterais do jogo). A programação é baseada em FreeRTOS para garantir responsividade e modularidade.
- Os sensores detectam movimentos da bola e registram os pontos. A lógica de controle inclui filtragem e persistência de calibração dos sensores, utilizando a NVS (memória não volátil) do ESP32.
- O Raspberry Pi executa um servidor Flask que gerencia a interface gráfica do jogo, incluindo a pontuação atual e um hall da fama com os melhores desempenhos. A comunicação entre o ESP32 e o servidor ocorre via porta serial.

## Tecnologias Utilizadas

- ESP32 DevKit V1 com FreeRTOS
- Sensores de movimento para registro de pontuação
- Motores de passo para controle dos flippers
- Raspberry Pi com servidor Flask
- HTML/CSS para a interface web
- Armazenamento NVS no ESP32 para calibração automática

## Créditos

Projeto desenvolvido por alunos do curso de Engenharia de Controle e Automação do IFPR — Instituto Federal do Paraná, sob orientação do professor Jefferson Wilhelm Meyer Soares:

- Adauto Eduardo Pereira de Souza  
- Guilherme da Silva Faganelli  
- Jonatas Bernardino dos Santos  
- Luciano Diz da Silva  
- Miguel Ferreira Baliego  
- Ronald Jose Contijo  
- Winderson Nascimento da Cruz  

**Colaboração adicional:**  
- Luiz Eduardo Virgínio de Souza

Ano: 2025
