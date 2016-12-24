EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:stm32
LIBS:Power_Management
LIBS:motor_drivers
LIBS:drv8305
LIBS:bsc016n06ns
LIBS:BMC2-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 3
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L DRV8305 U?
U 1 1 58539F2D
P 3200 3350
F 0 "U?" H 3300 3300 60  0000 C CNN
F 1 "DRV8305" H 3300 3150 60  0000 C CNN
F 2 "" H 3300 2850 60  0001 C CNN
F 3 "" H 3300 2850 60  0001 C CNN
	1    3200 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58539F8F
P 3350 5250
F 0 "#PWR?" H 3350 5000 50  0001 C CNN
F 1 "GND" H 3350 5100 50  0000 C CNN
F 2 "" H 3350 5250 50  0000 C CNN
F 3 "" H 3350 5250 50  0000 C CNN
	1    3350 5250
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5853A0AC
P 1750 2200
F 0 "R?" V 1830 2200 50  0000 C CNN
F 1 "10K" V 1750 2200 50  0000 C CNN
F 2 "" V 1680 2200 50  0000 C CNN
F 3 "" H 1750 2200 50  0000 C CNN
	1    1750 2200
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR?
U 1 1 5853A0E8
P 1750 1900
F 0 "#PWR?" H 1750 1750 50  0001 C CNN
F 1 "+3V3" H 1750 2040 50  0000 C CNN
F 2 "" H 1750 1900 50  0000 C CNN
F 3 "" H 1750 1900 50  0000 C CNN
	1    1750 1900
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5853A124
P 1550 2200
F 0 "R?" V 1630 2200 50  0000 C CNN
F 1 "10K" V 1550 2200 50  0000 C CNN
F 2 "" V 1480 2200 50  0000 C CNN
F 3 "" H 1550 2200 50  0000 C CNN
	1    1550 2200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5853A1B9
P 3850 1800
F 0 "C?" H 3875 1900 50  0000 L CNN
F 1 "0.047uf 45v" H 3875 1700 50  0000 L CNN
F 2 "" H 3888 1650 50  0000 C CNN
F 3 "" H 3850 1800 50  0000 C CNN
	1    3850 1800
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5854472D
P 3550 1500
F 0 "C?" H 3575 1600 50  0000 L CNN
F 1 "0.047uf 90V" H 3575 1400 50  0000 L CNN
F 2 "" H 3588 1350 50  0000 C CNN
F 3 "" H 3550 1500 50  0000 C CNN
	1    3550 1500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58544790
P 2600 1750
F 0 "C?" H 2625 1850 50  0000 L CNN
F 1 "1uf 6.3V" V 2750 1600 50  0000 L CNN
F 2 "" H 2638 1600 50  0000 C CNN
F 3 "" H 2600 1750 50  0000 C CNN
	1    2600 1750
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 585447EA
P 2600 1500
F 0 "C?" H 2625 1600 50  0000 L CNN
F 1 "1uf 6.3V" V 2750 1400 50  0000 L CNN
F 2 "" H 2638 1350 50  0000 C CNN
F 3 "" H 2600 1500 50  0000 C CNN
	1    2600 1500
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 58544849
P 2600 1200
F 0 "C?" H 2625 1300 50  0000 L CNN
F 1 "1uf 16V" V 2750 1050 50  0000 L CNN
F 2 "" H 2638 1050 50  0000 C CNN
F 3 "" H 2600 1200 50  0000 C CNN
	1    2600 1200
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 58544FF0
P 2350 1850
F 0 "#PWR?" H 2350 1600 50  0001 C CNN
F 1 "GND" H 2350 1700 50  0000 C CNN
F 2 "" H 2350 1850 50  0000 C CNN
F 3 "" H 2350 1850 50  0000 C CNN
	1    2350 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5854508D
P 5250 1600
F 0 "C?" H 5275 1700 50  0000 L CNN
F 1 "2.2uf 16V" H 5275 1500 50  0000 L CNN
F 2 "" H 5288 1450 50  0000 C CNN
F 3 "" H 5250 1600 50  0000 C CNN
	1    5250 1600
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 585450DB
P 5250 2050
F 0 "C?" H 5275 2150 50  0000 L CNN
F 1 "4.7uF 45V" H 5275 1950 50  0000 L CNN
F 2 "" H 5288 1900 50  0000 C CNN
F 3 "" H 5250 2050 50  0000 C CNN
	1    5250 2050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 585451CE
P 5250 2250
F 0 "#PWR?" H 5250 2000 50  0001 C CNN
F 1 "GND" H 5250 2100 50  0000 C CNN
F 2 "" H 5250 2250 50  0000 C CNN
F 3 "" H 5250 2250 50  0000 C CNN
	1    5250 2250
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5854522C
P 5250 2550
F 0 "R?" V 5330 2550 50  0000 C CNN
F 1 "100R" V 5250 2550 50  0000 C CNN
F 2 "" V 5180 2550 50  0000 C CNN
F 3 "" H 5250 2550 50  0000 C CNN
	1    5250 2550
	0    1    1    0   
$EndComp
$Comp
L +36V #PWR?
U 1 1 58545328
P 5950 1600
F 0 "#PWR?" H 5950 1450 50  0001 C CNN
F 1 "+36V" H 5950 1740 50  0000 C CNN
F 2 "" H 5950 1600 50  0000 C CNN
F 3 "" H 5950 1600 50  0000 C CNN
	1    5950 1600
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 585456A3
P 8700 5200
F 0 "R?" V 8780 5200 50  0000 C CNN
F 1 "R001" V 8700 5200 50  0000 C CNN
F 2 "" V 8630 5200 50  0000 C CNN
F 3 "" H 8700 5200 50  0000 C CNN
	1    8700 5200
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 585456EE
P 7950 5650
F 0 "R?" V 8030 5650 50  0000 C CNN
F 1 "R001" V 7950 5650 50  0000 C CNN
F 2 "" V 7880 5650 50  0000 C CNN
F 3 "" H 7950 5650 50  0000 C CNN
	1    7950 5650
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 58545738
P 6950 6050
F 0 "R?" V 7030 6050 50  0000 C CNN
F 1 "R001" V 6950 6050 50  0000 C CNN
F 2 "" V 6880 6050 50  0000 C CNN
F 3 "" H 6950 6050 50  0000 C CNN
	1    6950 6050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 585465EE
P 7950 6000
F 0 "#PWR?" H 7950 5750 50  0001 C CNN
F 1 "GND" H 7950 5850 50  0000 C CNN
F 2 "" H 7950 6000 50  0000 C CNN
F 3 "" H 7950 6000 50  0000 C CNN
	1    7950 6000
	1    0    0    -1  
$EndComp
$Comp
L BSC016N06NS Q?
U 1 1 5855359D
P 7800 2800
F 0 "Q?" H 7600 3100 50  0000 L CNN
F 1 "BSC016N06NS" H 7600 2550 50  0000 L CNN
F 2 "" H 7800 3000 50  0000 C CIN
F 3 "" V 7800 2800 50  0000 L CNN
	1    7800 2800
	1    0    0    -1  
$EndComp
$Comp
L BSC016N06NS Q?
U 1 1 58553698
P 7800 3450
F 0 "Q?" H 7600 3750 50  0000 L CNN
F 1 "BSC016N06NS" H 7600 3200 50  0000 L CNN
F 2 "" H 7800 3650 50  0000 C CIN
F 3 "" V 7800 3450 50  0000 L CNN
	1    7800 3450
	1    0    0    -1  
$EndComp
$Comp
L BSC016N06NS Q?
U 1 1 5855385A
P 8650 1850
F 0 "Q?" H 8450 2150 50  0000 L CNN
F 1 "BSC016N06NS" H 8450 1600 50  0000 L CNN
F 2 "" H 8650 2050 50  0000 C CIN
F 3 "" V 8650 1850 50  0000 L CNN
	1    8650 1850
	1    0    0    -1  
$EndComp
$Comp
L BSC016N06NS Q?
U 1 1 585538BC
P 8650 1150
F 0 "Q?" H 8450 1450 50  0000 L CNN
F 1 "BSC016N06NS" H 8450 900 50  0000 L CNN
F 2 "" H 8650 1350 50  0000 C CIN
F 3 "" V 8650 1150 50  0000 L CNN
	1    8650 1150
	1    0    0    -1  
$EndComp
$Comp
L BSC016N06NS Q?
U 1 1 5855390C
P 6850 4250
F 0 "Q?" H 6650 4550 50  0000 L CNN
F 1 "BSC016N06NS" H 6650 4000 50  0000 L CNN
F 2 "" H 6850 4450 50  0000 C CIN
F 3 "" V 6850 4250 50  0000 L CNN
	1    6850 4250
	1    0    0    -1  
$EndComp
$Comp
L BSC016N06NS Q?
U 1 1 58553A23
P 6850 4900
F 0 "Q?" H 6650 5200 50  0000 L CNN
F 1 "BSC016N06NS" H 6650 4650 50  0000 L CNN
F 2 "" H 6850 5100 50  0000 C CIN
F 3 "" V 6850 4900 50  0000 L CNN
	1    6850 4900
	1    0    0    -1  
$EndComp
$Comp
L +36V #PWR?
U 1 1 58554223
P 8950 700
F 0 "#PWR?" H 8950 550 50  0001 C CNN
F 1 "+36V" H 8950 840 50  0000 C CNN
F 2 "" H 8950 700 50  0000 C CNN
F 3 "" H 8950 700 50  0000 C CNN
	1    8950 700 
	1    0    0    -1  
$EndComp
$Comp
L +36V #PWR?
U 1 1 5855565E
P 8100 2350
F 0 "#PWR?" H 8100 2200 50  0001 C CNN
F 1 "+36V" H 8100 2490 50  0000 C CNN
F 2 "" H 8100 2350 50  0000 C CNN
F 3 "" H 8100 2350 50  0000 C CNN
	1    8100 2350
	1    0    0    -1  
$EndComp
$Comp
L +36V #PWR?
U 1 1 585563B7
P 7150 3950
F 0 "#PWR?" H 7150 3800 50  0001 C CNN
F 1 "+36V" H 7150 4090 50  0000 C CNN
F 2 "" H 7150 3950 50  0000 C CNN
F 3 "" H 7150 3950 50  0000 C CNN
	1    7150 3950
	1    0    0    -1  
$EndComp
Text HLabel 9450 1500 2    60   Output ~ 0
PhaseA
Text HLabel 9450 3100 2    60   Output ~ 0
PhaseB
Text HLabel 9500 4550 2    60   Output ~ 0
PhaseC
$Comp
L R R?
U 1 1 5855C63C
P 9350 1700
F 0 "R?" V 9430 1700 50  0000 C CNN
F 1 "39K" V 9350 1700 50  0000 C CNN
F 2 "" V 9280 1700 50  0000 C CNN
F 3 "" H 9350 1700 50  0000 C CNN
	1    9350 1700
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5855C6A2
P 9350 2100
F 0 "R?" V 9430 2100 50  0000 C CNN
F 1 "3K" V 9350 2100 50  0000 C CNN
F 2 "" V 9280 2100 50  0000 C CNN
F 3 "" H 9350 2100 50  0000 C CNN
	1    9350 2100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5855C999
P 9350 2350
F 0 "#PWR?" H 9350 2100 50  0001 C CNN
F 1 "GND" H 9350 2200 50  0000 C CNN
F 2 "" H 9350 2350 50  0000 C CNN
F 3 "" H 9350 2350 50  0000 C CNN
	1    9350 2350
	1    0    0    -1  
$EndComp
Text HLabel 9600 1900 2    60   Output ~ 0
VSenseA
$Comp
L R R?
U 1 1 5855E442
P 9350 3400
F 0 "R?" V 9430 3400 50  0000 C CNN
F 1 "39K" V 9350 3400 50  0000 C CNN
F 2 "" V 9280 3400 50  0000 C CNN
F 3 "" H 9350 3400 50  0000 C CNN
	1    9350 3400
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5855E4AB
P 9350 3800
F 0 "R?" V 9430 3800 50  0000 C CNN
F 1 "3K" V 9350 3800 50  0000 C CNN
F 2 "" V 9280 3800 50  0000 C CNN
F 3 "" H 9350 3800 50  0000 C CNN
	1    9350 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5855E51E
P 9350 4100
F 0 "#PWR?" H 9350 3850 50  0001 C CNN
F 1 "GND" H 9350 3950 50  0000 C CNN
F 2 "" H 9350 4100 50  0000 C CNN
F 3 "" H 9350 4100 50  0000 C CNN
	1    9350 4100
	1    0    0    -1  
$EndComp
Text HLabel 9650 3600 2    60   Output ~ 0
VSenseB
$Comp
L C C?
U 1 1 5855EF88
P 9600 3850
F 0 "C?" H 9625 3950 50  0000 L CNN
F 1 "30pF 5V" H 9625 3750 50  0000 L CNN
F 2 "" H 9638 3700 50  0000 C CNN
F 3 "" H 9600 3850 50  0000 C CNN
	1    9600 3850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5855F356
P 9600 2150
F 0 "C?" H 9625 2250 50  0000 L CNN
F 1 "30pF 5V" H 9625 2050 50  0000 L CNN
F 2 "" H 9638 2000 50  0000 C CNN
F 3 "" H 9600 2150 50  0000 C CNN
	1    9600 2150
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5855F677
P 9250 4950
F 0 "R?" V 9330 4950 50  0000 C CNN
F 1 "39K" V 9250 4950 50  0000 C CNN
F 2 "" V 9180 4950 50  0000 C CNN
F 3 "" H 9250 4950 50  0000 C CNN
	1    9250 4950
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5855F6E8
P 9250 5400
F 0 "R?" V 9330 5400 50  0000 C CNN
F 1 "3K" V 9250 5400 50  0000 C CNN
F 2 "" V 9180 5400 50  0000 C CNN
F 3 "" H 9250 5400 50  0000 C CNN
	1    9250 5400
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5855F762
P 9550 5350
F 0 "C?" H 9575 5450 50  0000 L CNN
F 1 "30pF 5V" H 9575 5250 50  0000 L CNN
F 2 "" H 9588 5200 50  0000 C CNN
F 3 "" H 9550 5350 50  0000 C CNN
	1    9550 5350
	1    0    0    -1  
$EndComp
Text HLabel 9750 5100 2    60   Output ~ 0
VSenseC
$Comp
L GND #PWR?
U 1 1 5855F833
P 9250 5650
F 0 "#PWR?" H 9250 5400 50  0001 C CNN
F 1 "GND" H 9250 5500 50  0000 C CNN
F 2 "" H 9250 5650 50  0000 C CNN
F 3 "" H 9250 5650 50  0000 C CNN
	1    9250 5650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 585560F6
P 8700 5500
F 0 "#PWR?" H 8700 5250 50  0001 C CNN
F 1 "GND" H 8700 5350 50  0000 C CNN
F 2 "" H 8700 5500 50  0000 C CNN
F 3 "" H 8700 5500 50  0000 C CNN
	1    8700 5500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58556274
P 6950 6300
F 0 "#PWR?" H 6950 6050 50  0001 C CNN
F 1 "GND" H 6950 6150 50  0000 C CNN
F 2 "" H 6950 6300 50  0000 C CNN
F 3 "" H 6950 6300 50  0000 C CNN
	1    6950 6300
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 585569F3
P 7500 4200
F 0 "C?" H 7525 4300 50  0000 L CNN
F 1 "0.1uf 50V" H 7525 4100 50  0000 L CNN
F 2 "" H 7538 4050 50  0000 C CNN
F 3 "" H 7500 4200 50  0000 C CNN
	1    7500 4200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 58556BEB
P 7500 4400
F 0 "#PWR?" H 7500 4150 50  0001 C CNN
F 1 "GND" H 7500 4250 50  0000 C CNN
F 2 "" H 7500 4400 50  0000 C CNN
F 3 "" H 7500 4400 50  0000 C CNN
	1    7500 4400
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 58556F1D
P 8350 2650
F 0 "C?" H 8375 2750 50  0000 L CNN
F 1 "0.1uf 50V" H 8375 2550 50  0000 L CNN
F 2 "" H 8388 2500 50  0000 C CNN
F 3 "" H 8350 2650 50  0000 C CNN
	1    8350 2650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5855728A
P 8350 2900
F 0 "#PWR?" H 8350 2650 50  0001 C CNN
F 1 "GND" H 8350 2750 50  0000 C CNN
F 2 "" H 8350 2900 50  0000 C CNN
F 3 "" H 8350 2900 50  0000 C CNN
	1    8350 2900
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5855751A
P 9300 1000
F 0 "C?" H 9325 1100 50  0000 L CNN
F 1 "0.1uf 50V" H 9325 900 50  0000 L CNN
F 2 "" H 9338 850 50  0000 C CNN
F 3 "" H 9300 1000 50  0000 C CNN
	1    9300 1000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5855769F
P 9300 1250
F 0 "#PWR?" H 9300 1000 50  0001 C CNN
F 1 "GND" H 9300 1100 50  0000 C CNN
F 2 "" H 9300 1250 50  0000 C CNN
F 3 "" H 9300 1250 50  0000 C CNN
	1    9300 1250
	1    0    0    -1  
$EndComp
Text HLabel 1550 4350 0    60   Output ~ 0
ISenseA
Text HLabel 1550 4450 0    60   Output ~ 0
ISenseB
Text HLabel 1550 4550 0    60   Output ~ 0
ISenseC
Text HLabel 1450 4050 0    60   Input ~ 0
SCLK
Text HLabel 1450 3950 0    60   Output ~ 0
MISO
Text HLabel 1450 3850 0    60   Input ~ 0
MOSI
Text HLabel 1450 3750 0    60   Input ~ 0
sSCS
Text HLabel 1350 2600 0    60   Output ~ 0
PWRGD
Text HLabel 1300 3550 0    60   Output ~ 0
nFAULT
Text HLabel 1350 2500 0    60   Input ~ 0
WAKE
Text HLabel 1350 2800 0    60   Input ~ 0
EN_GATE
Text HLabel 1350 2950 0    60   Input ~ 0
INHA
Text HLabel 1350 3050 0    60   Input ~ 0
INLA
Text HLabel 1350 3150 0    60   Input ~ 0
INHB
Text HLabel 1350 3250 0    60   Input ~ 0
INLB
Text HLabel 1350 3350 0    60   Input ~ 0
INHC
Text HLabel 1350 3450 0    60   Input ~ 0
INLC
$Comp
L R R?
U 1 1 5855A71B
P 3350 6050
F 0 "R?" V 3430 6050 50  0000 C CNN
F 1 "39K" V 3350 6050 50  0000 C CNN
F 2 "" V 3280 6050 50  0000 C CNN
F 3 "" H 3350 6050 50  0000 C CNN
	1    3350 6050
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5855A7D6
P 3350 6500
F 0 "R?" V 3430 6500 50  0000 C CNN
F 1 "3K" V 3350 6500 50  0000 C CNN
F 2 "" V 3280 6500 50  0000 C CNN
F 3 "" H 3350 6500 50  0000 C CNN
	1    3350 6500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5855A877
P 3350 6750
F 0 "#PWR?" H 3350 6500 50  0001 C CNN
F 1 "GND" H 3350 6600 50  0000 C CNN
F 2 "" H 3350 6750 50  0000 C CNN
F 3 "" H 3350 6750 50  0000 C CNN
	1    3350 6750
	1    0    0    -1  
$EndComp
$Comp
L +36V #PWR?
U 1 1 5855A8E8
P 3350 5800
F 0 "#PWR?" H 3350 5650 50  0001 C CNN
F 1 "+36V" H 3350 5940 50  0000 C CNN
F 2 "" H 3350 5800 50  0000 C CNN
F 3 "" H 3350 5800 50  0000 C CNN
	1    3350 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 5050 3200 5150
Wire Wire Line
	3200 5150 3500 5150
Wire Wire Line
	3350 5050 3350 5250
Wire Wire Line
	3500 5150 3500 5050
Connection ~ 3350 5150
Wire Wire Line
	1750 1900 1750 2050
Wire Wire Line
	1750 2350 1750 2600
Wire Wire Line
	1350 2600 2250 2600
Wire Wire Line
	1550 2350 1550 3550
Wire Wire Line
	1300 3550 2250 3550
Wire Wire Line
	1550 2050 1550 1900
Wire Wire Line
	1550 1900 2200 1900
Wire Wire Line
	3850 1900 3850 1950
Wire Wire Line
	3850 1650 3700 1650
Wire Wire Line
	3700 1650 3700 1900
Wire Wire Line
	3550 1900 3550 1650
Wire Wire Line
	3550 1350 3400 1350
Wire Wire Line
	3400 1350 3400 1900
Wire Wire Line
	3000 1500 3000 1900
Wire Wire Line
	2850 1900 2850 1750
Wire Wire Line
	2850 1750 2750 1750
Wire Wire Line
	2750 1500 3000 1500
Wire Wire Line
	2750 1200 3150 1200
Wire Wire Line
	3150 1200 3150 1900
Wire Wire Line
	2450 1200 2350 1200
Wire Wire Line
	2350 1200 2350 1850
Wire Wire Line
	2350 1500 2450 1500
Wire Wire Line
	2350 1750 2450 1750
Connection ~ 2350 1500
Connection ~ 2350 1750
Wire Wire Line
	4450 2200 4600 2200
Wire Wire Line
	4600 2200 4600 1350
Wire Wire Line
	4600 1350 5250 1350
Wire Wire Line
	5250 1350 5250 1450
Wire Wire Line
	5250 1750 5250 1900
Wire Wire Line
	4900 1850 5950 1850
Wire Wire Line
	4900 1850 4900 2300
Wire Wire Line
	4900 2300 4450 2300
Connection ~ 5250 1850
Wire Wire Line
	5250 2250 5250 2200
Wire Wire Line
	5100 2550 4800 2550
Wire Wire Line
	4800 2550 4800 2450
Wire Wire Line
	4800 2450 4450 2450
Wire Wire Line
	5400 2550 5500 2550
Wire Wire Line
	5500 2550 5500 1850
Wire Wire Line
	5950 1850 5950 1600
Connection ~ 5500 1850
Wire Wire Line
	4450 4050 5600 4050
Wire Wire Line
	5600 4050 5600 5250
Wire Wire Line
	8350 5050 8700 5050
Wire Wire Line
	4450 4150 5250 4150
Wire Wire Line
	5250 4150 5250 5350
Wire Wire Line
	5250 5350 8700 5350
Wire Wire Line
	4450 4300 5150 4300
Wire Wire Line
	5150 4300 5150 5500
Wire Wire Line
	5150 5500 7950 5500
Wire Wire Line
	4450 4400 4900 4400
Wire Wire Line
	4900 4400 4900 5800
Wire Wire Line
	4900 5800 7950 5800
Wire Wire Line
	4450 4550 4800 4550
Wire Wire Line
	4800 4550 4800 5900
Wire Wire Line
	4800 5900 6950 5900
Wire Wire Line
	4500 6200 6950 6200
Wire Wire Line
	4500 4650 4500 6200
Wire Wire Line
	4500 4650 4450 4650
Wire Wire Line
	7950 5800 7950 6000
Wire Wire Line
	6950 6200 6950 6300
Wire Wire Line
	8350 1150 8150 1150
Wire Wire Line
	8150 1150 8150 1500
Wire Wire Line
	6500 1500 9450 1500
Wire Wire Line
	8950 1500 8950 1950
Connection ~ 8950 1650
Connection ~ 8950 1750
Connection ~ 8950 1850
Wire Wire Line
	8350 950  8350 1150
Connection ~ 8350 1050
Wire Wire Line
	8950 700  8950 1250
Connection ~ 8950 1150
Connection ~ 8950 1050
Connection ~ 8950 950 
Wire Wire Line
	4450 2650 6350 2650
Wire Wire Line
	6350 2650 6350 1250
Wire Wire Line
	6350 1250 8350 1250
Wire Wire Line
	6500 1500 6500 2750
Wire Wire Line
	6500 2750 4450 2750
Connection ~ 8150 1500
Wire Wire Line
	4450 2850 6650 2850
Wire Wire Line
	6650 2850 6650 1950
Wire Wire Line
	6650 1950 8350 1950
Wire Wire Line
	8350 1650 8350 2200
Connection ~ 8350 1750
Wire Wire Line
	8700 5050 8650 2200
Wire Wire Line
	8650 2200 8350 2200
Connection ~ 8350 1850
Wire Wire Line
	4450 2950 6800 2950
Wire Wire Line
	6800 2950 6800 2100
Wire Wire Line
	6800 2100 8350 2100
Connection ~ 8350 2100
Wire Wire Line
	5600 5250 8350 5250
Wire Wire Line
	8350 5250 8350 5050
Wire Wire Line
	8100 2350 8100 2900
Connection ~ 8100 2600
Connection ~ 8100 2700
Connection ~ 8100 2800
Wire Wire Line
	7500 2600 7500 3100
Connection ~ 7500 2700
Wire Wire Line
	4450 3100 6900 3100
Wire Wire Line
	6900 3100 6900 2900
Wire Wire Line
	6900 2900 7500 2900
Wire Wire Line
	4450 3200 7250 3200
Wire Wire Line
	7250 3200 7250 3100
Wire Wire Line
	7250 3100 9450 3100
Wire Wire Line
	8100 3100 8100 3550
Connection ~ 8100 3250
Connection ~ 8100 3350
Connection ~ 8100 3450
Wire Wire Line
	7500 3250 7500 3800
Connection ~ 7500 3350
Connection ~ 7500 3100
Connection ~ 7500 2800
Wire Wire Line
	4450 3300 7250 3300
Wire Wire Line
	7250 3300 7250 3550
Wire Wire Line
	7250 3550 7500 3550
Wire Wire Line
	7500 3800 7950 3800
Wire Wire Line
	7950 3800 7950 5500
Connection ~ 7500 3450
Wire Wire Line
	4450 3400 7150 3400
Wire Wire Line
	7150 3400 7150 3650
Wire Wire Line
	7150 3650 7500 3650
Connection ~ 7500 3650
Wire Wire Line
	7150 3950 7150 4350
Connection ~ 7150 4050
Connection ~ 7150 4150
Connection ~ 7150 4250
Wire Wire Line
	6550 4050 6550 4250
Connection ~ 6550 4150
Wire Wire Line
	6550 4350 6400 4350
Wire Wire Line
	6400 4350 6400 3550
Wire Wire Line
	6400 3550 4450 3550
Wire Wire Line
	6550 4250 6500 4250
Wire Wire Line
	6500 4250 6500 4550
Wire Wire Line
	6250 4550 9500 4550
Wire Wire Line
	7150 4550 7150 5000
Connection ~ 7150 4700
Connection ~ 7150 4800
Connection ~ 7150 4900
Wire Wire Line
	6550 4700 6550 5650
Connection ~ 6550 4800
Wire Wire Line
	4450 3650 6250 3650
Wire Wire Line
	6250 3650 6250 4550
Connection ~ 6500 4550
Wire Wire Line
	6550 5000 6100 5000
Wire Wire Line
	6100 5000 6100 3750
Wire Wire Line
	6100 3750 4450 3750
Wire Wire Line
	6550 5650 6950 5650
Wire Wire Line
	6950 5650 6950 5900
Connection ~ 6550 4900
Wire Wire Line
	6550 5150 5750 5150
Wire Wire Line
	5750 5150 5750 3850
Wire Wire Line
	5750 3850 4450 3850
Connection ~ 6550 5150
Connection ~ 8950 1500
Connection ~ 8100 3100
Connection ~ 7150 4550
Wire Wire Line
	9350 1550 9350 1500
Connection ~ 9350 1500
Wire Wire Line
	9350 1850 9350 1950
Wire Wire Line
	9350 2250 9350 2350
Wire Wire Line
	9350 1900 9600 1900
Connection ~ 9350 1900
Wire Wire Line
	9350 3950 9350 4100
Wire Wire Line
	9350 3550 9350 3650
Wire Wire Line
	9350 3250 9350 3100
Connection ~ 9350 3100
Wire Wire Line
	9350 3600 9650 3600
Connection ~ 9350 3600
Wire Wire Line
	9350 4000 9350 4050
Wire Wire Line
	9350 4050 9600 4050
Wire Wire Line
	9600 4050 9600 4000
Connection ~ 9350 4000
Wire Wire Line
	9600 3700 9600 3600
Connection ~ 9600 3600
Wire Wire Line
	9350 2300 9600 2300
Connection ~ 9350 2300
Wire Wire Line
	9600 2000 9500 2000
Wire Wire Line
	9500 2000 9500 1900
Connection ~ 9500 1900
Wire Wire Line
	9250 5550 9250 5650
Wire Wire Line
	9250 5100 9250 5250
Wire Wire Line
	9250 5200 9550 5200
Connection ~ 9250 5200
Wire Wire Line
	9550 5200 9550 5100
Wire Wire Line
	9550 5100 9750 5100
Wire Wire Line
	9550 5500 9550 5600
Wire Wire Line
	9550 5600 9250 5600
Connection ~ 9250 5600
Wire Wire Line
	9250 4800 9250 4550
Connection ~ 9250 4550
Wire Wire Line
	8700 5350 8700 5500
Wire Wire Line
	7500 4050 7150 4050
Wire Wire Line
	7500 4400 7500 4350
Wire Wire Line
	8350 2500 8350 2400
Wire Wire Line
	8350 2400 8100 2400
Connection ~ 8100 2400
Wire Wire Line
	8350 2800 8350 2900
Wire Wire Line
	9300 850  9300 750 
Wire Wire Line
	9300 750  8950 750 
Wire Wire Line
	8950 750  8950 800 
Connection ~ 8950 800 
Wire Wire Line
	9300 1150 9300 1250
Wire Wire Line
	2250 2400 2200 2400
Wire Wire Line
	2200 2400 2200 1900
Connection ~ 1750 1900
Wire Wire Line
	1550 4350 1900 4350
Wire Wire Line
	1900 4350 1900 4300
Wire Wire Line
	1900 4300 2250 4300
Wire Wire Line
	1550 4450 1950 4450
Wire Wire Line
	1950 4450 1950 4400
Wire Wire Line
	1950 4400 2250 4400
Wire Wire Line
	1550 4550 2000 4550
Wire Wire Line
	2000 4550 2000 4500
Wire Wire Line
	2000 4500 2250 4500
Wire Wire Line
	1450 3750 2250 3750
Wire Wire Line
	1450 3850 2250 3850
Wire Wire Line
	2250 3950 1450 3950
Wire Wire Line
	1450 4050 2250 4050
Connection ~ 1750 2600
Connection ~ 1550 3550
Wire Wire Line
	1350 2500 2250 2500
Wire Wire Line
	1350 2800 2250 2800
Wire Wire Line
	2250 2800 2250 2850
Wire Wire Line
	1350 3450 2250 3450
Wire Wire Line
	1350 2950 2250 2950
Wire Wire Line
	2250 3050 1350 3050
Wire Wire Line
	1350 3150 2250 3150
Wire Wire Line
	1350 3250 2250 3250
Wire Wire Line
	1350 3350 2250 3350
Wire Wire Line
	3350 5800 3350 5900
Wire Wire Line
	3350 6350 3350 6200
Wire Wire Line
	3350 6650 3350 6750
$Comp
L C C?
U 1 1 5855B06D
P 3100 6500
F 0 "C?" H 3125 6600 50  0000 L CNN
F 1 "0.1uF 5V" H 3125 6400 50  0000 L CNN
F 2 "" H 3138 6350 50  0000 C CNN
F 3 "" H 3100 6500 50  0000 C CNN
	1    3100 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 6350 3350 6350
Wire Wire Line
	3100 6650 3100 6700
Wire Wire Line
	3100 6700 3350 6700
Connection ~ 3350 6700
Text HLabel 2850 6350 0    60   Output ~ 0
VSupplySense
Connection ~ 3100 6350
$Comp
L C C?
U 1 1 5855C2E8
P 2000 2100
F 0 "C?" H 2025 2200 50  0000 L CNN
F 1 "1uF 4V" H 2025 2000 50  0000 L CNN
F 2 "" H 2038 1950 50  0000 C CNN
F 3 "" H 2000 2100 50  0000 C CNN
	1    2000 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 1950 2000 1900
Connection ~ 2000 1900
$Comp
L GND #PWR?
U 1 1 5855C539
P 2000 2300
F 0 "#PWR?" H 2000 2050 50  0001 C CNN
F 1 "GND" H 2000 2150 50  0000 C CNN
F 2 "" H 2000 2300 50  0000 C CNN
F 3 "" H 2000 2300 50  0000 C CNN
	1    2000 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 2300 2000 2250
$Comp
L C C?
U 1 1 585D5E19
P 4650 4750
F 0 "C?" H 4675 4850 50  0000 L CNN
F 1 "1000pf" H 4675 4650 50  0000 L CNN
F 2 "" H 4688 4600 50  0000 C CNN
F 3 "" H 4650 4750 50  0000 C CNN
	1    4650 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 4900 4650 5000
Wire Wire Line
	4650 5000 4500 5000
Connection ~ 4500 5000
Wire Wire Line
	4650 4600 4650 4550
Connection ~ 4650 4550
$Comp
L C C?
U 1 1 585D645B
P 5050 4550
F 0 "C?" H 5075 4650 50  0000 L CNN
F 1 "1000pf" H 5075 4450 50  0000 L CNN
F 2 "" H 5088 4400 50  0000 C CNN
F 3 "" H 5050 4550 50  0000 C CNN
	1    5050 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 4700 5050 4800
Wire Wire Line
	5050 4800 4900 4800
Connection ~ 4900 4800
Wire Wire Line
	5050 4400 5050 4300
Connection ~ 5050 4300
$Comp
L C C?
U 1 1 585D6827
P 5450 4300
F 0 "C?" H 5475 4400 50  0000 L CNN
F 1 "1000pf" H 5475 4200 50  0000 L CNN
F 2 "" H 5488 4150 50  0000 C CNN
F 3 "" H 5450 4300 50  0000 C CNN
	1    5450 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 4450 5250 4450
Connection ~ 5250 4450
Wire Wire Line
	5450 4150 5450 4050
Connection ~ 5450 4050
$EndSCHEMATC
