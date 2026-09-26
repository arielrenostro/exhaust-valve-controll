# Exhaust Valve Driver

Firmware PlatformIO para Arduino Nano que controla uma válvula de escapamento
(borboleta) acionada por motor DC, pilotado por um módulo BTS7960, com
realimentação de posição por potenciômetro e alvo definido por PWM de 100Hz
vindo da ECU.

O desafio central do projeto: **o potenciômetro de posição e o motor
compartilham o mesmo fio físico**. Um MOSFET multiplexa esse fio entre
"energizar o motor" e "ler o sinal do potenciômetro", e as duas coisas nunca
podem acontecer ao mesmo tempo — fazer isso queima o microcontrolador.
Praticamente toda a arquitetura deste firmware existe para lidar com essa
restrição com segurança.

## Sumário

- [Hardware e pinout](#hardware-e-pinout)
- [O interlock D7/D9](#o-interlock-d7d9)
- [Arquitetura do firmware](#arquitetura-do-firmware)
- [Máquina de estados](#máquina-de-estados)
- [Simulação do alvo em bancada](#simulação-do-alvo-em-bancada)
- [Calibração (`include/config.h`)](#calibração-includeconfigh)
- [Build, upload e testes](#build-upload-e-testes)
- [Estrutura do projeto](#estrutura-do-projeto)
- [Status atual e próximos passos](#status-atual-e-próximos-passos)

## Hardware e pinout

| Pino | Função |
|------|--------|
| `D2` | Leitura do PWM de 100Hz da ECU (alvo de abertura), captura não-bloqueante |
| `D5` | `R_PWM` do BTS7960 |
| `D6` | `L_PWM` do BTS7960 |
| `D7` | `R_EN` + `L_EN` do BTS7960 (ligados juntos) |
| `D9` | Habilita o MOSFET que conecta `A0` ao potenciômetro |
| `A0` | Leitura analógica do potenciômetro de posição (fio compartilhado com o motor) |
| `A1` | `R_IS` do BTS7960 — corrente do sentido de abertura |
| `A2` | `L_IS` do BTS7960 — corrente do sentido de fechamento |
| `A3` | Tensão da bateria do carro, via divisor resistivo (30,1kΩ / 10kΩ) |
| `A4` | *(bancada, opcional)* Potenciômetro simulando o PWM da ECU — veja [Simulação do alvo em bancada](#simulação-do-alvo-em-bancada) |

> **Nunca habilitar `D9` junto com `D7`.** Habilitar o motor energiza o fio
> compartilhado, tornando a leitura de `A0` inválida (e potencialmente
> perigosa para o pino analógico). Essa regra é garantida em software por um
> único módulo (`ActuatorInterlock`) — nada mais no firmware tem permissão
> para tocar em `D5`, `D6`, `D7`, `D9` ou `A0` diretamente.

## O interlock D7/D9

Toda leitura de posição exige desligar o motor, ler o mais rápido possível e
só então religar — nunca os dois ao mesmo tempo:

```
|----- T_DRIVE_MS -----|-settle-|--read--|-settle-|----- proximo drive -----|
 D7=HIGH                D7=LOW    D9=HIGH  D9=LOW    D7=HIGH
 (motor girando)        (motor    (A0       (motor
                        parado)   lido)     ainda parado)
```

A corrente (`A1`/`A2`) não sofre dessa limitação — é lida direto do BTS7960,
então a detecção de batente por corrente continua funcionando com o motor
ligado, inclusive durante o homing.

## Arquitetura do firmware

```
┌─────────────────────┐     ┌───────────────────────┐
│   PwmTargetReader     │     │   ActuatorInterlock     │
│  (D2, interrupt +      │     │  (D7/D9 mutex, dead-time, │
│   watchdog Timer1)      │     │   direção reversa)         │
└──────────┬───────────┘     └────────────┬────────────┘
           │  duty% / validade                  │  drive() / readPositionRaw()
           v                                    v
┌─────────────────────────────────────────────────────────────┐
│                     ValvePositionControl                       │
│  PID (modo POSICAO) ou dead-reckoning (modo TEMPO)              │
│  holding-at-stops · fail-safe (sinal perdido) · falha de meio-curso │
└───────────┬───────────────────────────┬─────────────────────┘
            │ (re)homing                 │ eventos / telemetria
            v                            v
┌─────────────────────┐     ┌───────────────────────┐
│  HomingCalibration    │     │        Logger           │
│  (batente a batente,   │     │  Serial 115200, eventos  │
│   min/max, tempos)      │     │  + telemetria throttled   │
└─────────────────────┘     └───────────────────────┘
```

Lógica pura e testável nativamente (sem `Arduino.h`) vive em `lib/`:
`DutyMapping`, `CurrentSense`, `ModeSelection`, `DeadReckoning`,
`PositionDeadband`, `Pid`. Código que fala com o hardware vive em `src/`.

## Máquina de estados

```
        LIGA (power on)
              |
              v
      +-------------------+
      |      HOMING         |<------------------------+
      |  batente a batente    |                         |
      +-------------------+                          |
              |                                        |
   grava min/max(A0), t_abre, t_fecha                  |
   escolhe MODO: POSICAO (PID) ou TEMPO (dead-reckoning) |
              |                                        |
              v                                        |
      +-------------------+   sobrecorrente no meio     |
      |      RUNNING         |------ do curso ------------+
      |  alvo = duty(D2)       |   (corta motor, refaz
      |  holding nos batentes    |    homing automaticamente)
      +-------------------+
              |
       D2 sem sinal valido
              v
      +-------------------+
      |      FAILSAFE         |
      |  vai e segura           |
      |  TOTALMENTE ABERTA        |
      +-------------------+
```

Como a válvula não tem mola de retorno em nenhum extremo, alcançar um
batente nunca solta o motor — ele continua empurrando com força reduzida
(`HOLD_DUTY_PERCENT`), com proteção por corrente que reduz ainda mais essa
força (e loga um evento) se a corrente de sustentação ficar alta por tempo
demais.

## Simulação do alvo em bancada

Enquanto não houver um gerador de PWM (ou a própria ECU) disponível para
testar, o firmware pode ler o alvo de abertura de um **potenciômetro em
`A4`** em vez do sinal PWM real do `D2`. Controlado por uma única flag em
`include/config.h`:

```cpp
constexpr bool SIMULATE_TARGET_WITH_POTENTIOMETER = true; // false = usa o D2 real
constexpr int TARGET_SIM_POT_MAX_COUNTS = 900;             // leitura do A4 que representa 100%
```

A leitura do `A4` é mapeada de `0` a `900` (não a escala cheia de 0-1023 do
ADC) para `0-100%` de abertura — ajuste `TARGET_SIM_POT_MAX_COUNTS` se o seu
potenciômetro não bater exatamente nesses 900. Com a flag em `true`, o
sinal de fail-safe por perda de PWM (`D2` sem sinal) fica desativado, já
que não existe um "sinal perdido" equivalente para o potenciômetro — o
firmware sempre considera o alvo do `A4` válido. O módulo responsável é
`src/TargetSource.*`, que é o único ponto do código que decide qual das
duas fontes usar; o resto do firmware (`ValvePositionControl`) não sabe
nem precisa saber qual está ativa.

Antes de instalar no carro, lembre de voltar a flag para `false` (ou usar o
gerador de PWM de bancada / a ECU real).

## Calibração (`include/config.h`)

Todos os valores de calibração são constantes nomeadas e comentadas em
`include/config.h` — nenhum "número mágico" espalhado pela lógica. Já
populados com os valores medidos neste hardware:

- Escala de corrente do BTS7960: `0.00V = 0A`, `4.0V = 3.4A`
- Divisor de tensão da bateria: `20,05V = 5V` no pino (R1=30,1kΩ, R2=10kΩ)
- Mapeamento do duty do D2: `0% = aberta`, `100% = fechada` (`INVERT_TARGET_DUTY = true`)

Ainda precisam de calibração em bancada antes de ir para o carro:
`T_DRIVE_MS`, `T_BTS7960_TURNOFF_MS`/`T_READ_MOSFET_TURNOFF_US` (os dois
dead-times do interlock — o do BTS7960 em milissegundos pelo datasheet, o do
MOSFET 2N7000 em microssegundos, já que ele chaveia em ~10ns), limiares de
corrente de batente (`STALL_CURRENT_THRESHOLD_*_AMPS`), ganhos do PID,
`HOLD_DUTY_PERCENT`, `HOLD_CURRENT_THRESHOLD_AMPS` e o prescaler do ADC. Veja
`openspec/changes/exhaust-valve-controller/design.md` para o racional de
cada um.

## Build, upload e testes

Projeto PlatformIO padrão — via VS Code + extensão PlatformIO, ou CLI:

```bash
# compilar e gravar no Nano
pio run -e nanoatmega328new -t upload

# monitor serial (115200 baud)
pio device monitor -b 115200

# rodar os testes unitários da lógica pura (não precisa de hardware)
pio test -e native
```

## Estrutura do projeto

```
include/config.h          constantes de calibração (sem dependência de Arduino.h)
include/Pins.h             pinout fixo do hardware
lib/PwmTargetReader/        captura não-bloqueante do PWM em D2
lib/DutyMapping/            duty% -> alvo de abertura (0..1)
lib/CurrentSense/           ADC -> Amps (escala do BTS7960)
lib/ModeSelection/          escolhe modo POSICAO vs TEMPO
lib/DeadReckoning/          estimador de posição por tempo
lib/PositionDeadband/       banda-morta de posição
lib/Pid/                    controlador PID genérico
lib/AnalogTargetMapping/    ADC do A4 -> duty% (simulação de bancada)
src/ActuatorInterlock.*     o mutex D7/D9 (único lugar que toca no motor/sensor)
src/CurrentSensing.*        leitura de corrente (A1/A2) e tensão de bateria (A3)
src/TargetSource.*          alterna entre D2 (PWM real) e A4 (pot de bancada)
src/HomingCalibration.*     varredura batente-a-batente na inicialização
src/ValvePositionControl.*  loop de controle principal
src/Logger.*                 log serial (eventos + telemetria throttled)
src/main.cpp                 integração de tudo
test/test_native/            testes unitários nativos (Unity), sem hardware
openspec/                    proposta, specs, design e tasks desta implementação
```

## Status atual e próximos passos

O firmware está implementado e compila limpo (`pio run`), com 19 testes
unitários nativos passando (`pio test -e native`) cobrindo toda a lógica
pura. O que ainda falta é **verificação em bancada com hardware real**:
confirmar no osciloscópio/multímetro que `D7`/`D9` nunca se sobrepõem,
calibrar os tempos e limiares de corrente, e validar os dois modos de
controle, o holding nos batentes, a falha de meio-curso e o fail-safe do
sinal do D2. O checklist completo está em
`openspec/changes/exhaust-valve-controller/tasks.md` (item 8.2).
