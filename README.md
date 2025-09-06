Pinout:
PC8 -> TRST
PC10 -> TCK (SWCLK)
PC11 -> TDO
PC12 -> TDI
PD2 -> SRST

PE2 -> TCK (SWCLK)
PE5 -> TMS (SWDIO)

It is MANDATORY to tie PE2 and PC10 together.
In addition, it is safer to add a 1Kohm series resistor onto SWDIO line when using SWD (to mitigate short circuit if target or host tries to drive the line at the same time). 

Please compile the project with "-ofast". It increases the performance A LOT.

Watch the demo:
https://www.youtube.com/watch?v=F0JFuAMPzhQ
https://www.youtube.com/watch?v=yzbKtbcyPDk

Read the Medium article:
https://medium.com/@sahin.duran.9275/9236206d2bf8


