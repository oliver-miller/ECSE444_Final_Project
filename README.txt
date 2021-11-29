Operation:
Plug player boards into the host board

Pinouts:
1. Player
A0 -> ACC_OUT
A1 -> MIC_OUT
A2 -> BUTTON_OUT
D15 -> RESET_PLAYER
+ COMMON GROUND

A0, A1, A2 -> GPIO_OUT
D15 -> INTERRUPT

2. Host
A0 -> ACC_IN_1
A1 -> MIC_IN_1
A2 -> BUTTON_IN_1
A3 -> ACC_IN_2
A4 -> MIC_IN_2
A5 -> BUTTON_IN_2
D14 -> RESET_PLAYER_1
D15 -> RESET_PLAYER_2
+ COMMON GROUND

A0, A1, A2, A3, A4, A5 -> INTERRUPT
D14, D15 -> GPIO_OUT


NOTES:
- Microphone sometimes turns on when board is grabbed
for twisting. Idea to fix -> increases threshold when not
microphone task, but idk.
- UART is not configured on the host board. It is on the 
player board.