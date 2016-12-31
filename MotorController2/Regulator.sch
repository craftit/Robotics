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
LIBS:dc-dc
LIBS:IFX91041EJV33
LIBS:BMC2-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
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
L GND #PWR?
U 1 1 585F6F4B
P 2850 3350
F 0 "#PWR?" H 2850 3100 50  0001 C CNN
F 1 "GND" H 2850 3200 50  0000 C CNN
F 2 "" H 2850 3350 50  0000 C CNN
F 3 "" H 2850 3350 50  0000 C CNN
	1    2850 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 2800 2850 3350
Wire Wire Line
	2850 2950 3000 2950
Wire Wire Line
	2850 2800 3000 2800
Connection ~ 2850 2950
$Comp
L C C?
U 1 1 585F6F73
P 2500 2700
F 0 "C?" H 2525 2800 50  0000 L CNN
F 1 "22nF 6V" H 2525 2600 50  0000 L CNN
F 2 "" H 2538 2550 50  0000 C CNN
F 3 "" H 2500 2700 50  0000 C CNN
	1    2500 2700
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 585F6FA4
P 2500 3050
F 0 "R?" V 2580 3050 50  0000 C CNN
F 1 "22k" V 2500 3050 50  0000 C CNN
F 2 "" V 2430 3050 50  0000 C CNN
F 3 "" H 2500 3050 50  0000 C CNN
	1    2500 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 3250 2500 3250
Wire Wire Line
	2500 3250 2500 3200
Connection ~ 2850 3250
Wire Wire Line
	2500 2900 2500 2850
Wire Wire Line
	2500 2550 2500 2500
Wire Wire Line
	2500 2500 2850 2500
Wire Wire Line
	2850 2500 2850 2650
Wire Wire Line
	2850 2650 3000 2650
$Comp
L +36V #PWR?
U 1 1 585F700E
P 950 1450
F 0 "#PWR?" H 950 1300 50  0001 C CNN
F 1 "+36V" H 950 1590 50  0000 C CNN
F 2 "" H 950 1450 50  0000 C CNN
F 3 "" H 950 1450 50  0000 C CNN
	1    950  1450
	1    0    0    -1  
$EndComp
$Comp
L D D?
U 1 1 585F7042
P 1300 1600
F 0 "D?" H 1300 1700 50  0000 C CNN
F 1 "D" H 1300 1500 50  0000 C CNN
F 2 "" H 1300 1600 50  0000 C CNN
F 3 "" H 1300 1600 50  0000 C CNN
	1    1300 1600
	-1   0    0    1   
$EndComp
$Comp
L C C?
U 1 1 585F709D
P 1550 1800
F 0 "C?" H 1575 1900 50  0000 L CNN
F 1 "220nF 60V" H 1575 1700 50  0000 L CNN
F 2 "" H 1588 1650 50  0000 C CNN
F 3 "" H 1550 1800 50  0000 C CNN
	1    1550 1800
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR L?
U 1 1 585F70E9
P 2000 1600
F 0 "L?" V 1950 1600 50  0000 C CNN
F 1 "47uH" V 2100 1600 50  0000 C CNN
F 2 "" H 2000 1600 50  0000 C CNN
F 3 "" H 2000 1600 50  0000 C CNN
	1    2000 1600
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 585F7125
P 2700 1850
F 0 "C?" H 2725 1950 50  0000 L CNN
F 1 "220nF 60V" H 2725 1750 50  0000 L CNN
F 2 "" H 2738 1700 50  0000 C CNN
F 3 "" H 2700 1850 50  0000 C CNN
	1    2700 1850
	1    0    0    -1  
$EndComp
$Comp
L CP C?
U 1 1 585F715C
P 2400 1850
F 0 "C?" H 2425 1950 50  0000 L CNN
F 1 "100uF 60V" H 1900 1750 50  0000 L CNN
F 2 "" H 2438 1700 50  0000 C CNN
F 3 "" H 2400 1850 50  0000 C CNN
	1    2400 1850
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 585F72AD
P 4350 2250
F 0 "C?" H 4375 2350 50  0000 L CNN
F 1 "220nF 6V" H 4375 2150 50  0000 L CNN
F 2 "" H 4388 2100 50  0000 C CNN
F 3 "" H 4350 2250 50  0000 C CNN
	1    4350 2250
	1    0    0    -1  
$EndComp
$Comp
L D_Schottky D?
U 1 1 585F731A
P 4350 2700
F 0 "D?" H 4350 2800 50  0000 C CNN
F 1 "D_Schottky" H 4350 2600 50  0000 C CNN
F 2 "" H 4350 2700 50  0000 C CNN
F 3 "" H 4350 2700 50  0000 C CNN
	1    4350 2700
	0    1    1    0   
$EndComp
$Comp
L CP C?
U 1 1 585F737F
P 5300 2700
F 0 "C?" H 5325 2800 50  0000 L CNN
F 1 "100uF 6V" H 4900 2600 50  0000 L CNN
F 2 "" H 5338 2550 50  0000 C CNN
F 3 "" H 5300 2700 50  0000 C CNN
	1    5300 2700
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 585F73CE
P 5550 2700
F 0 "C?" H 5575 2800 50  0000 L CNN
F 1 "220nF 6V" H 5400 2600 50  0000 L CNN
F 2 "" H 5588 2550 50  0000 C CNN
F 3 "" H 5550 2700 50  0000 C CNN
	1    5550 2700
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR L?
U 1 1 585F74CD
P 4800 2450
F 0 "L?" V 4750 2450 50  0000 C CNN
F 1 "47uH" V 4900 2450 50  0000 C CNN
F 2 "" H 4800 2450 50  0000 C CNN
F 3 "" H 4800 2450 50  0000 C CNN
	1    4800 2450
	0    1    1    0   
$EndComp
Wire Wire Line
	950  1450 950  1600
Wire Wire Line
	950  1600 1150 1600
Wire Wire Line
	1450 1600 1700 1600
Wire Wire Line
	1550 1600 1550 1650
Connection ~ 1550 1600
Wire Wire Line
	2300 1600 3550 1600
Wire Wire Line
	2400 1600 2400 1700
Wire Wire Line
	2700 1600 2700 1700
Connection ~ 2400 1600
Wire Wire Line
	4350 2400 4350 2550
Wire Wire Line
	4100 2450 4500 2450
Connection ~ 4350 2450
Wire Wire Line
	5300 2050 5300 2550
Wire Wire Line
	5550 2450 5550 2550
Connection ~ 5300 2450
Wire Wire Line
	5100 2450 5100 3200
Wire Wire Line
	4200 2950 3900 2950
Wire Wire Line
	3900 2650 3900 2100
Wire Wire Line
	3900 2100 4350 2100
Wire Wire Line
	3900 2800 4100 2800
Wire Wire Line
	4100 2800 4100 2450
$Comp
L GND #PWR?
U 1 1 585F787D
P 4350 2950
F 0 "#PWR?" H 4350 2700 50  0001 C CNN
F 1 "GND" H 4350 2800 50  0000 C CNN
F 2 "" H 4350 2950 50  0000 C CNN
F 3 "" H 4350 2950 50  0000 C CNN
	1    4350 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 2950 4350 2850
$Comp
L GND #PWR?
U 1 1 585F7983
P 5450 2950
F 0 "#PWR?" H 5450 2700 50  0001 C CNN
F 1 "GND" H 5450 2800 50  0000 C CNN
F 2 "" H 5450 2950 50  0000 C CNN
F 3 "" H 5450 2950 50  0000 C CNN
	1    5450 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 2850 5300 2950
Wire Wire Line
	5550 2950 5550 2850
Connection ~ 5450 2950
Wire Wire Line
	4200 2950 4200 3200
Wire Wire Line
	4200 3200 5100 3200
Wire Wire Line
	3550 1600 3550 2200
Connection ~ 2700 1600
$Comp
L GND #PWR?
U 1 1 585F7C36
P 1550 2050
F 0 "#PWR?" H 1550 1800 50  0001 C CNN
F 1 "GND" H 1550 1900 50  0000 C CNN
F 2 "" H 1550 2050 50  0000 C CNN
F 3 "" H 1550 2050 50  0000 C CNN
	1    1550 2050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 585F7C68
P 2400 2050
F 0 "#PWR?" H 2400 1800 50  0001 C CNN
F 1 "GND" H 2400 1900 50  0000 C CNN
F 2 "" H 2400 2050 50  0000 C CNN
F 3 "" H 2400 2050 50  0000 C CNN
	1    2400 2050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 585F7C9A
P 2700 2050
F 0 "#PWR?" H 2700 1800 50  0001 C CNN
F 1 "GND" H 2700 1900 50  0000 C CNN
F 2 "" H 2700 2050 50  0000 C CNN
F 3 "" H 2700 2050 50  0000 C CNN
	1    2700 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 2050 1550 1950
Wire Wire Line
	2400 2050 2400 2000
Wire Wire Line
	2700 2050 2700 2000
Wire Wire Line
	3350 2200 3350 1600
Connection ~ 3350 1600
$Comp
L ZENER D?
U 1 1 585F8179
P 5900 2700
F 0 "D?" H 5900 2800 50  0000 C CNN
F 1 "5.4V" H 5900 2600 50  0000 C CNN
F 2 "" H 5900 2700 50  0000 C CNN
F 3 "" H 5900 2700 50  0000 C CNN
	1    5900 2700
	0    1    1    0   
$EndComp
Wire Wire Line
	5900 2950 5900 2900
Connection ~ 5550 2950
Wire Wire Line
	5900 2450 5900 2500
Connection ~ 5550 2450
$Comp
L LED D?
U 1 1 585F871A
P 8550 2850
F 0 "D?" H 8550 2950 50  0000 C CNN
F 1 "LED" H 8550 2750 50  0000 C CNN
F 2 "" H 8550 2850 50  0000 C CNN
F 3 "" H 8550 2850 50  0000 C CNN
	1    8550 2850
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 585F876D
P 8550 2400
F 0 "R?" V 8630 2400 50  0000 C CNN
F 1 "100R" V 8550 2400 50  0000 C CNN
F 2 "" V 8480 2400 50  0000 C CNN
F 3 "" H 8550 2400 50  0000 C CNN
	1    8550 2400
	1    0    0    -1  
$EndComp
Connection ~ 5900 2950
Connection ~ 5900 2450
$Comp
L IFX91041EJV50 U?
U 1 1 58659107
P 3450 3100
F 0 "U?" H 3450 3000 60  0000 C CNN
F 1 "IFX91041EJV50" H 3450 3100 60  0000 C CNN
F 2 "" H 3450 3100 60  0001 C CNN
F 3 "" H 3450 3100 60  0001 C CNN
	1    3450 3100
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR?
U 1 1 586591BB
P 5300 2050
F 0 "#PWR?" H 5300 1900 50  0001 C CNN
F 1 "+5V" H 5300 2190 50  0000 C CNN
F 2 "" H 5300 2050 50  0000 C CNN
F 3 "" H 5300 2050 50  0000 C CNN
	1    5300 2050
	1    0    0    -1  
$EndComp
$Comp
L D D?
U 1 1 58659257
P 1600 1300
F 0 "D?" H 1600 1400 50  0000 C CNN
F 1 "D" H 1600 1200 50  0000 C CNN
F 2 "" H 1600 1300 50  0000 C CNN
F 3 "" H 1600 1300 50  0000 C CNN
	1    1600 1300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1600 1450 1600 1600
Connection ~ 1600 1600
$Comp
L +8V #PWR?
U 1 1 5865931B
P 1600 1000
F 0 "#PWR?" H 1600 850 50  0001 C CNN
F 1 "+8V" H 1600 1140 50  0000 C CNN
F 2 "" H 1600 1000 50  0000 C CNN
F 3 "" H 1600 1000 50  0000 C CNN
	1    1600 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 1000 1600 1150
$Comp
L GND #PWR?
U 1 1 586595C6
P 6600 3150
F 0 "#PWR?" H 6600 2900 50  0001 C CNN
F 1 "GND" H 6600 3000 50  0000 C CNN
F 2 "" H 6600 3150 50  0000 C CNN
F 3 "" H 6600 3150 50  0000 C CNN
	1    6600 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 3150 6600 2950
$Comp
L +3V3 #PWR?
U 1 1 58659680
P 7300 2300
F 0 "#PWR?" H 7300 2150 50  0001 C CNN
F 1 "+3V3" H 7300 2440 50  0000 C CNN
F 2 "" H 7300 2300 50  0000 C CNN
F 3 "" H 7300 2300 50  0000 C CNN
	1    7300 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 2450 7900 2450
Wire Wire Line
	7300 2450 7300 2300
$Comp
L C C?
U 1 1 5865982D
P 7250 2750
F 0 "C?" H 7275 2850 50  0000 L CNN
F 1 "1uF 4V min" H 7275 2650 50  0000 L CNN
F 2 "" H 7288 2600 50  0000 C CNN
F 3 "" H 7250 2750 50  0000 C CNN
	1    7250 2750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 586598C9
P 7250 3000
F 0 "#PWR?" H 7250 2750 50  0001 C CNN
F 1 "GND" H 7250 2850 50  0000 C CNN
F 2 "" H 7250 3000 50  0000 C CNN
F 3 "" H 7250 3000 50  0000 C CNN
	1    7250 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 3000 7250 2900
Wire Wire Line
	7250 2600 7250 2450
Connection ~ 7250 2450
$Comp
L ZENER D?
U 1 1 58659A6D
P 7900 2800
F 0 "D?" H 7900 2900 50  0000 C CNN
F 1 "3.6V" H 7900 2700 50  0000 C CNN
F 2 "" H 7900 2800 50  0000 C CNN
F 3 "" H 7900 2800 50  0000 C CNN
	1    7900 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	7900 2150 7900 2600
Connection ~ 7300 2450
Wire Wire Line
	8550 2550 8550 2650
Wire Wire Line
	7900 2150 8550 2150
Wire Wire Line
	8550 2150 8550 2250
Connection ~ 7900 2450
Wire Wire Line
	5300 2950 5900 2950
$Comp
L TC2117-3.3V U?
U 1 1 5865A268
P 6600 2550
F 0 "U?" H 6600 2950 60  0000 C CNN
F 1 "TC2117-3.3V" H 6600 2850 60  0000 C CNN
F 2 "" H 6600 2550 60  0001 C CNN
F 3 "" H 6600 2550 60  0001 C CNN
	1    6600 2550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5865A3E7
P 7900 3100
F 0 "#PWR?" H 7900 2850 50  0001 C CNN
F 1 "GND" H 7900 2950 50  0000 C CNN
F 2 "" H 7900 3100 50  0000 C CNN
F 3 "" H 7900 3100 50  0000 C CNN
	1    7900 3100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5865A42E
P 8550 3150
F 0 "#PWR?" H 8550 2900 50  0001 C CNN
F 1 "GND" H 8550 3000 50  0000 C CNN
F 2 "" H 8550 3150 50  0000 C CNN
F 3 "" H 8550 3150 50  0000 C CNN
	1    8550 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 3050 8550 3150
Wire Wire Line
	7900 3000 7900 3100
Wire Wire Line
	6200 2450 5100 2450
$EndSCHEMATC
