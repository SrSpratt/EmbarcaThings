# Embarcatech-Residência
## EmbarcaThings
#### Autor:
* Roberto Vítor Lima Gomes Rodrigues

### Servidor MQTT
Para o desenvolvimento da aplicação com servidor MQTT, foram utilizados o terminal wi-fi cyw43-arch, os LEDs RGB, a matriz de LEDs WS2818, os Buzzers, o Joystick com auxílio de ADC, e PWM para controlar a intensidade do buzzer e dos LEDs RGB.
Para abertura do servidor (broker) MQTT, foi utilizado o software mosquitto num celular com Android.
Par o monitoramento e controle dos periféricos da placa através do protocolo MQTT, foi utilizado o MQTT Explorer num laptop com Windows.

#### Vídeo de funcionamento
* https://youtu.be/ZxnqypBus3g


#### Instruções de compilação
Certifique-se de que você tem o ambiente configurado conforme abaixo:
* Pico SDK instalado e configurado.
* VSCode com todas as extensões configuradas, CMake e Make instalados.
* Clone o repositório e abra a pasta do projeto, a extensão Pi Pico criará a pasta build
* Altere as linhas referentes à conexão Wi-Fi
* Clique em Compile na barra inferior, do lado direito (ao lado esquerdo de RUN | PICO SDK)
* Verifique se gerou o arquivo .uf2
* Conecte a placa BitDogLab e ponha-a em modo BOOTSEL
* Arraste o arquivo até a placa, que o programa se iniciará

#### Manual do programa
###### Antes de executar o programa, é preciso alterar as linhas referentes à conexão Wi-Fi
Ao executar o programa, ele buscará conectar-se com as credenciais informadas de rede e servidor MQTT (broker):
   * É imprescindível ter iniciado o servidor mqtt em outro dispositivo (neste caso, foi utilizado o Mosquitto no celular).
   * É interessante abrir o monitor serial para monitorar as leituras de temperatura e umidade
* O LED RGB azul ficará acesso até que ele se conecte;
* Se o LED RGB vermelho se acender, quer dizer que ele não conseguiu se conectar; aperte RESET para tentar novamente (e verifique suas informações de conexão)
* Quando o LED RGB verde se acender, quer dizer que a conexão foi bem sucedida
    * Abra o monitor explorer, com as credenciais do servidor MQTT aberto.
    * Será possível ver os tópicos de temperatura  (/temperature) e umidade (/humidity) a partir do momento que a bitdoglab começar a enviá-los
       * É possível clicar nos tópicos e criar gráficos para acompanhar as mudanças de temperatura e umidade com o tempo
       * Além dos tópicos padrões de temperatura e umidade, a placa também enviará valores qualitativos para os tópicos /tempNotify e /humNotify, que classificarão a temperatura e umidade em, respectivamente: frio, bom e quente (para a temperatura); seco, bom e úmido (para a umidade) 
    * Será possível alterar o estado da luminária com os valores de on e off no tópico /led
    * Será possível ligar e desligar um buzzer de 5s com os valores de on e off no tópico /ring (muito embora tocá-lo uma vez não fará com que o toque seja contínuo, não havendo a necessidade de desligar depois)
    * Será possível ligar e desligar uma mangueira de água, simulada por uma animação na matriz de LEDs acompanhada de um toque curto intermitente no buzzer.
    * Para cada um desses tópicos, será possível atrelar um gráfico cujos pontos representarão ligado e desligado, tal que será possível monitorar a quantidade de vezes em cada estado lógico 
* As leituras de temperatura são feitas com a movimentação do joystick

