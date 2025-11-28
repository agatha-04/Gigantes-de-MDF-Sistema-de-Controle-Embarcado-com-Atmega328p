# Gigantes-de-MDF-Sistema-de-Controle-Embarcado-com-ATmega328p
Firmware, Eletrônica e Documentação Técnica do projeto final da disciplina de Programação de Hardware - 2025.

## Descrição Geral
Esse projeto implementa o sistema eletrônico e programação em C ao robô do projeto final da disciplina Programação de Hardware (2025).
Ele deve:
* Se mover de acordo com a posição do Joystick;
* Controlar os motores por via PWM;
* Detectar impactos do laser inimigo através de um LDR;
* Disparar sua própria arma laser de 1 em 1 segundo;
* Registrar danos (causados pelo laser inimigo) através de LEDs que representam as vidas;
* Reagir ao ser atingido (giro de 180º, perda de uma vida, pausa de 5 segundos);
* Parar o sistema inteiro após a perda das três vidas, reiniciando após apertar o botão de reset.

## Objetivos
* Aplicar conceitos de programação de hardware usando o microcontrolador ATmega328p.
* Projetar um circuito completo de acionamento de motores.
* Implementar controle PWM para movimentação.
* Implementar timers para disparo do laser e rotina de 1 segundo.
* Programar rotinas de interrupção para o botão reset das vidas.
* Criar documentação com o Doxygen.
* Disponilizar todo o código em repositório GitHub.

## Componentes Eletrônicos Usados no Projeto
* 2 microcontroladores do tipo ATmega328p (1 para o circuito que controla o sistema de vidas e motores, e o segundo para o circuito do controle remoto).
* 1 diodo laser de 5V/5mW.
* 1 transistor Mosfet IRLZ44n (para controle do diodo laser).
* 1 LDR de 20 milímetros.
* 3 LEDs azuis 10 milímetros.
* 1 CI de ponte H L293d.
* 2 motores DC com faixa de operação de 3V-6V.
* 2 módulos wireless NRF24L01 (transceptores de rádio, que agem como a comunicação entre o circuito do robô e o circuito do controle).
* 2 módulos Joystick (um para controle de direção, e o segundo agindo como o botão de reset).
* Regulador de tensão de 5V (LM7805).
* Regulador de tensão dd 6V (LM7806).
* Regulador de tensão de 3.3V (AMS1117).
* 3 baterias de 9V.

## Bibliotecas Usadas
* Biblioteca RF24.

## Como Testar
**1.** Gravar os arquivos transmissor.ino.hex (controle) e receptor.ino.hex (robô) nos respectivos microcontroladores.
**2.** Conectar as baterias ao robô.
**3.** Testar as respectivas funções:
  * Giro no eixo;
  * PWM dos motores;
  * Disparo do laser:
  * Resposta ao laser inimigo;
  * LEDs de vida;
  * Botão de reset.

## Autores
* **Autor(a):** Agatha Nicole Marques Fávaro - **RA:** 231335
* **Autor(a):** Mykaella Scarllet de Lemos Santana - **RA:** 233508
* **Autor(a):** Rafael Ribeiro de Lima - **RA:** 226326
* Universidade Santa Cecília - Engenharia da Computação
* **Disciplina:** Programação de Hardware - 2/2025
* **Professor(a):** Sérgio Schina de Andrade
