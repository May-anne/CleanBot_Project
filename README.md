# CleanBot_Project
1. **Estado inicial**

   * Antes: começava em `SPIRAL`, depois ia para `WALL_FOLLOW` ou `RANDOM_WALK`.
   * Agora: começa **direto em `RANDOM_WALK`**, que virou o modo **ZIGZAG**.
   * `SPIRAL` ficou só como função de referência, mas **não é mais usado no fluxo normal**.

2. **Mapa em grade (memória espacial)**

   * Foi criado um **grid 2D fixo de 50x50 células** com `CELL_SIZE = 0.10` → cobre aprox. **5m x 5m**.
   * O grid é **centrado na posição inicial** do robô (`map_origin_x`, `map_origin_y`).
   * Duas matrizes:

     * `visited[cx][cy]`: conta quantas vezes cada célula foi visitada.
     * `obstacles[cx][cy]`: marca células com obstáculo detectado na frente.
   * Funções novas:

     * `_world_to_cell(x, y)` → converte posição contínua para índice de célula.
     * `_update_visit_map()` → incrementa a célula atual.
     * `_update_obstacle_from_front_sensor(d_front)` → marca obstáculo à frente quando bate.

3. **Visualização: heatmap de visitas**

   * Criada a **segunda figura do Matplotlib** (`fig_map`, `ax_map`) com um `imshow` da matriz `visited`.
   * `colorbar` mostra **“Número de visitas”**.
   * `_update_heatmap()` atualiza a imagem e o `vmax` é ajustado dinamicamente para o máximo de visitas do mapa.

4. **Novo comportamento RANDOM_WALK → ZIGZAG**

   * `RANDOM_WALK` agora implementa um **padrão de varredura em zigzag (lawnmower)**:

     * `_init_zigzag_if_needed()`:

       * na primeira entrada no estado, alinha o robô com o **eixo X** (yaw 0 ou π, o mais próximo);
       * define a direção em X: `zigzag_dir = +1` (indo para +X) ou `-1` (indo para -X).
     * No laço:

       * o robô anda **reto** com `FORWARD_SPEED` enquanto:

         * não tiver obstáculo na frente (`d_front >= D_STOP`), e
         * não atingir a borda do grid em X (coluna 0 ou coluna 49).
       * Quando encontra obstáculo ou borda:

         1. Para e dá uma ré curta (`_backup()`).
         2. Move **1 célula em Y** (`_drive_forward_for(self.cell_size)`) e vira para a **direção oposta em X**:

            * se estava indo para +X: sobe 1 célula (+Y) e passa a ir para -X;
            * se estava indo para -X: desce 1 célula (-Y) e passa a ir para +X.
       * Com isso, forma-se um **padrão de linhas paralelas**, cobrindo o ambiente faixa a faixa.
   * Se o robô sair do grid (posição fora de [0,49] em X ou Y), entra num comportamento simples de recuperação com giro aleatório.

5. **Escape / destravamento**

   * `ESCAPE` continua existindo, mas agora:

     * sempre que fica preso (`_is_stuck()`), vai para `ESCAPE`, faz:

       * `_backup()`,
       * `_scan_best_dir()` para achar direção com maior distância livre,
       * `_rotate_relative(best_dir)`,
       * volta para `RANDOM_WALK` e **reinicializa o zigzag** (`zigzag_initialized = False`).

6. **Estados pouco usados**

   * `WALL_FOLLOW` permanece implementado, mas não é mais chamado pelo fluxo principal (não há transição para ele).
   * `SPIRAL` idem — mantido só como referência / fallback, mas você não cai nele na lógica normal.

---

## 2. Comportamento esperado na simulação

Na prática, com esse código, você deve ver:

1. **Trajetória**

   * Ele **começa já alinhado** em uma direção horizontal (para direita ou esquerda).
   * Anda **reto até bater em algo ou atingir a borda do mapa em X**.
   * Quando isso acontece:

     * ré curta,
     * sobe/desce uma “linha” (1 célula em Y),
     * gira 90° + 90° e volta andando reto, agora no sentido oposto em X.
       → Isso deve formar um desenho **de “ida e volta” em faixas**, bem diferente daquele random walk caótico.

2. **Heatmap**

   * O mapa de visitas deve mostrar:

     * **“faixas” de células visitadas** com intensidade relativamente uniforme, em vez de um “bolo” concentrado em um lugar só.
     * Com o tempo, você vai ver **linhas paralelas preenchendo o grid** à medida que o robô varre novas linhas em Y.

3. **Reação a obstáculos / paredes**

   * Ao encontrar um móvel/parede no meio de uma faixa, ele trata isso como se fosse o fim da faixa:

     * ré,
     * sobe/desce uma célula,
     * inverte a direção e continua o zigzag.
   * Em cantos complicados (por exemplo, preso entre parede e sofá), o detector de “stuck” entra em ação:

     * estado `ESCAPE`,
     * procura uma direção mais livre,
     * volta para o zigzag.

4. **Sem espiral reentrante**

   * Em nenhum momento ele deve “voltar a espiralar” sozinho.
   * Todo o movimento de cobertura depois do início é feito **somente com a lógica de zigzag**.
