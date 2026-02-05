# TCC
TRABALHO DE CONCLUSÃO DE CURSO - BACHARELADO EM CIÊNCIA DA COMPUTAÇÃO

# Sistema de Irrigação Automatizada com ESP32 e Arduino Cloud

Projeto de TCC: **Irrigação automatizada em pomares e pequenas plantações: uma abordagem baseada em integração IoT na plataforma Arduino Cloud.**

## 📌 Sobre o projeto

Este trabalho apresenta o desenvolvimento de um sistema de irrigação automatizado voltado para pequenos e médios cultivos, utilizando o microcontrolador **ESP32 DevKit V1** e a plataforma **Arduino IoT Cloud**.

O sistema monitora em tempo real:

* Umidade do solo
* Temperatura do ar
* Umidade do ar
* Luminosidade

Com base nesses dados, o ESP32 toma decisões automáticas para acionar válvulas de irrigação, podendo também ser controlado remotamente via internet.

---

## 🎯 Objetivos do projeto

* Automatizar o processo de irrigação
* Reduzir deslocamentos no campo
* Economizar água e energia
* Permitir monitoramento remoto
* Integrar com assistentes virtuais

---

## ⚙️ Tecnologias utilizadas

* ESP32 DevKit V1
* Linguagem C/C++
* Arduino IoT Cloud
* Arduino Cloud Agent v1.7.0-87f097b
* Protocolo MQTT
* Sensores ambientais
* Relés e válvulas solenoides
* Google Sheets
* Google Apps Script (JavaScript)
* Power BI
* IDE Arduino v2.3.4 
* Energia solar (projeto conceitual)
* Asistente Virtual Alexa v2.2.626008.0 - Skill Arduino .
* PlantUML
* Fritzing v0.9.3b (Beta)

---

## 🧠 Funcionalidades

* Monitoramento ambiental em tempo real
* Irrigação automática baseada em sensores
* Controle remoto pela Arduino Cloud
* Integração com assistentes virtuais (ex: Alexa)
* Modo de economia de energia (deep sleep)
* Registro de dados em nuvem
* Visualização analítica no Power BI

---

## 🗂️ Estrutura do repositório

A organização das pastas segue a lógica utilizada durante o desenvolvimento do TCC, separando código, bibliotecas, hardware e dados.

```
/
├── src/                → Código principal do ESP32 e Scripts e configurações da Arduino Cloud
├── libraries/          → Bibliotecas utilizadas pelo projeto
├── docs/               → Documentação acadêmica (TCC, artigos, textos)
├── hardware/           → Esquemas, Fritzing, diagramas e Drivers
├── data/               → Arquivos de análise (Google Sheets, Power BI)
├── images/             → Fotos do protótipo e dashboards
```

### Observação importante

A pasta `libraries/` contém as versões das bibliotecas utilizadas durante o desenvolvimento para facilitar a reprodução do ambiente original do projeto.

Em ambientes de produção, essas bibliotecas podem ser instaladas diretamente pela **Arduino IDE**.

---

## 🔌 Hardware utilizado

| ID  | Nome da peça                                                                 | Número da peça       |
| --- | ---------------------------------------------------------------------------- | -------------------- |
| P1  | Placa ESP32 com Wi-Fi e Bluetooth (ESP32S Dual Core) – Dev Kit V1 + cabo USB | 1                    |
| P2  | Placa de expansão para ESP32 DevKit V1 (ESP32 Base Board – 30 pinos)         | 1                    |
| S1  | Sensores de umidade do solo                                                  | Conforme necessidade |
| S2  | Sensor de temperatura e umidade do ar (DHT11)                                | Conforme necessidade |
| S3  | Sensor de luminosidade (LDR – fotoresistor)                                  | Conforme necessidade |
| R1  | Módulo relé 1 canal – 5V                                                     | Conforme necessidade |
| V1  | Válvula solenóide 12V                                                        | Conforme necessidade |
| M1  | Multiplexador analógico CD74HC4067                                           | 1                    |
| PS1 | Painel solar 50W / 12V + regulador de carga 20A                              | Conforme necessidade |
| PS2 | Painel solar 50W / 12V (adicional)                                           | Conforme necessidade |
| B1  | Bateria selada 12V / 20Ah                                                    | 1                    |
| F1  | Fonte de alimentação 12V / 2A (sensores e atuadores)                         | Conforme necessidade |
| F2  | Fonte de alimentação 5V / 2A (sensores e lógica)                             | Conforme necessidade |
| D1  | Display LCD 16x2 com interface I2C                                           | 1                    |
| C1  | Caixa de proteção IP65 para uso externo                                      | Conforme necessidade |
| A1  | Conta ativa na plataforma Arduino Cloud                                      | 1                    |
| T1  | Ferramentas de desenvolvimento open-source                                   | 1                    |

---

## 🚀 Como executar o projeto

1. Instale a **Arduino IDE**.
2. Instale as bibliotecas necessárias.
3. Conecte o ESP32 ao computador.
4. Abra o código principal na pasta `src/`.
5. Configure:

   * Credenciais Wi-Fi
   * Arduino IoT Cloud
6. Faça o upload para o ESP32.

---

## 📊 Monitoramento de dados

Os dados coletados pelo sistema são enviados para:

* Arduino IoT Cloud (tempo real)
* Google Sheets (armazenamento)
* Power BI (análise e visualização)

---

## 📄 Documentação

O TCC completo pode ser encontrado na pasta:

```
docs/
```

---

## 👨‍💻 Autor

**André Oliveira Lopes**

---

## 📜 Licença

Este projeto está sob a licença MIT.

