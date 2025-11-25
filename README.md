# CleanBot_Project
Repositório para o desenvolvimento de código e recursos do projeto CleanBot, criado para a disciplina Elementos da Robótica do curso de Engenharia da Computação - UPE.

## Equipe
- Ana Karla
- Arthur Carvalho
- Carlson Vasconcelos
- Letícia Fontenelle
- Mayanne Gomes

## O Projeto
O projeto consiste em desenvolver um robô autônomo para a limpeza de residências. O objetivo é que o robô consiga se deslocar em um ambiente delimitado, realizando a varredura do espaço e efetuando sua limpeza de forma eficiente.

## As Branchs
O projeto neste repositório está organizado em branchs. Cada branch corresponde a uma parte fundamental do desenvolvimento do projeto.

- main: branch inicial, contém o README de apresentação do projeto.
- HouseSimulation: contém o cenário (arquivo .ttt) da casa utilizado para exemplificar o funcionamento do robô em uma residência.
- coppeliaSim: contém os aquivos .ttt e .py da simulação oficial de testes, com o algoritmo mais atualizado do sistema de navegação
- develop: contém o arquivo .ino com o código do robô físico
- poc_cleanbot2: branch de testes do sistema de navegação

## O Design
O design do robô foi desenvolvido através da plataforma Tinkercad. Toda a estrutura de suporte do robô será impresso em 3D.

<img width="1176" height="780" alt="Design do CleanBot" src="https://github.com/user-attachments/assets/8b54c83c-0d4c-41b3-9225-885756222ff9" />
![Acesse o modelo no TinkerCad](https://www.tinkercad.com/things/cUoWfwwbVK0/edit?sharecode=X5aFOnu0VZ7V_3BggZSapVHzO-4BnrmAJRIiCxf0ZVU)

## A Simulação
A simulação do funcionamento do robô foi feita através do simulador CoppeliaSim v4.10.0. A simulação foi desenvolvida através da integração do CoppeliaSim com o Python, através da ZMQ Remote API. Abaixo segue um vídeo demonstrativo do sistema de navegação utilizado atualmente.


### O Sistema de Navegação
O robô utiliza uma máquina de estados reativa, inspirada no comportamento de um Roomba, para explorar o ambiente com autonomia. Cada estado define um padrão de movimentação baseado exclusivamente em motores, encoders e sensor ultrassônico. A troca de estados ocorre conforme a situação percebida ao redor.

### Estados Principais
#### 1) SPIRAL
O robô inicia descrevendo uma espiral crescente, ampliando gradualmente o raio de curva.
Objetivo: cobrir área livre e encontrar paredes/obstáculos.
Transição: quando detecta obstáculo frontal → backup + seleção de lado → WALL_FOLLOW.

#### 2) WALL_FOLLOW
Segue a parede:
- mede distância lateral com o sensor em ±60°;
- mantém distância-alvo (d_set) da parede;
- corrige direção ajustando apenas a diferença de velocidade entre as rodas;
- aplica escape caso aproxime demais (ré curta + giro para longe).

Transições:
- perdeu a parede por muito tempo → Random Walk;
- obstáculo frontal → escape e continua no follow.

#### 3) RANDOM_WALK
Exploração livre sem dependência de yaw.
O robô:
- anda reto com motores iguais;
- ao detectar obstáculo frontal → para, faz backup relativo por odometria e gira um ângulo aleatório usando somente encoders;
- retoma movimento em um novo heading.

#### 4) ESCAPE
Estratégia de saída quando o robô detecta falta de progresso:
- ré mais longa,
- varredura angular com o sensor,
- gira para a direção mais livre,
- retorna ao RANDOM_WALK.
