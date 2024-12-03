# ESP32-CAM Telegram Security Bot

Este projeto utiliza um **ESP32-CAM** para criar um sistema de monitoramento inteligente. Com um sensor de movimento, o dispositivo captura uma foto sempre que detecta movimento e a envia diretamente para o seu **Telegram**.

---

## 📋 Funcionalidades

- Detecta movimento utilizando um sensor PIR.
- Captura fotos com a câmera do ESP32-CAM.
- Envia as imagens para um chat ou grupo no Telegram.
- Sistema simples e eficiente para monitoramento remoto.

---

## 🛠️ Hardware Necessário

- ESP32-CAM.
- Sensor de movimento PIR (HC-SR501 ou similar).
- Fonte de alimentação (5V).
- Cabos jumper e outros componentes básicos para conexão.

---

## 🖥️ Pré-requisitos

Antes de começar, certifique-se de ter:

1. **Arduino IDE** configurado com suporte ao ESP32. [Configuração oficial aqui](https://github.com/espressif/arduino-esp32).
2. Biblioteca para conexão com o Telegram, como a [UniversalTelegramBot](https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot).
3. Credenciais do seu bot no Telegram:
   - Crie um bot com o [BotFather](https://core.telegram.org/bots#botfather).
   - Obtenha o token do bot.

---

## 🔧 Configuração e Instalação

### 1. Configuração do código
- Clone este repositório:
  ```bash
  git clone https://github.com/seu-usuario/esp32-cam-telegram-bot.git
