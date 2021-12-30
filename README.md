# JKBMS
read data from JKBMS and send to grafana

use software as rour risk!
Connection data: directly to your esp:

                 RS485-TTL
┌──────────┐                ┌─────┐
│                3│<-TX--------RX->│         │
│  JK-BMS        2│<-RX--------TX->│ ESP32/  │
│                1│<----- GND ---->│ ESP8266 │
│                 │                │         │<-- 3.3V
└──────────┘                 ─────┘

# RS485-TTL jack (4 Pin, JST 1.25mm pinch)
┌─────────-
│ 1   2   3   4      │
│ O   O   O   O  │
│GND  RX  TX VBAT│
└──────────


 
