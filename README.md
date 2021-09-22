# Central_alarme

## Descrição do projeto:

O propósito desse projeto é a produção da comunicação de hardware comumente usado na **eletrônica**, com o **ATmega 328P**, para produção de um Central de Alarme. Uma central de alarme é amplamente usada para monitorar um determinado local com o intuito de identificar falhas, o exemplo mais comum de uso desse sistema seria
uma central para identificação de incêndios. Foi usado o simulador de hardware Proteus para produção de esquemáticos e simulação do sistema eletrônico. Todo sistema do projeto RTL foi descrito em C, em concordância com o padrão do ATmega usado, descrito no datasheet da ATMEL. Para este projeto a central acionará uma
sirene sempre que algum sensor de movimento detectar um movimento em um ambiente. São usados: um display LCD para para exibição dos parâmetros e dados para a central, leds para sinalizações do modo configuração e ativação para zonas, também um canal de entrada para os sensores, um canal de saída para a sirene e um canal de **comunicação USART** para acesso ao Arquivo de Log, utilzando a **memória EEPROM** do microcontrolador. O projeto foi desenvolvido com o objetivo de ser **eficiente**, utilizando o mínimo de recurso de hardware possível, para **redução de custos** de uma possível produção mercadológica. De maneira didática, a central foi responsável por testar os **conhecimentos previamente adquiridos dos sistemas embarcados de terceiros**, fomentar o conhecimento em baixo nível das ferramentas comuns do meio da **Engenharia Mecatrônica e Automação** e criar uma interlocução entre a eletrônica, Software em baixo nível e limitações mercadológicas.

Esse projeto foi feito em conjunto com outros integrantes. Para maior detalhamento acesse o **Relatorio.pdf** desse repositório. Por fim, **script.c** apresenta o código em C que atua no ATmega 328P.

## Habilidades adquiridas:

A produção desse projeto possibilitou o aprendizado de:
* Aperfeiçoamento de produção de relatório tecnico;
* Conhecimento específico do ATmega 328P; 
* Entendimento da comunicação USART;
* Capacidade de utilização da memória EEPROM para Arquivo de Log;
* Entendimento acerca dos tempos e momentos de interrupções do microcontrolador;
* Compreensão acerca da escolha de Baud Rate adequado;
* Tratamento de sinal para economia na utilização de canais únicos de comunicação;
* Familiaridade com programação de Displays LCD, coversores analógico-digitais, teclados e sensores;
* Leitura e entendimento de datasheets para produção de projetos;
* Capacidade de avaliação crítica na alocação e utilização de recursos.

## Elementos Homem-Máquina do Alarme:

<p align="center">
  <img src="https://user-images.githubusercontent.com/48588172/134418631-1e6586d8-10ce-4a24-af51-7c7d98fb9773.png" />
</p>

## Hardware produzido:

<p align="center">
  <img src="https://user-images.githubusercontent.com/48588172/134428133-f914f6fd-1d2e-47fb-87d8-ad365cf5702c.png" />
</p>
