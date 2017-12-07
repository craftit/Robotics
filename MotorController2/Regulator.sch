EESchema Schematic File Version 2
LIBS:BMC2-rescue
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
LIBS:MIC2009A-1YM6TR
LIBS:BMC2-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 4
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
L GND #PWR053
U 1 1 585F6F4B
P 2850 3350
F 0 "#PWR053" H 2850 3100 50  0001 C CNN
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
L C C38
U 1 1 585F6F73
P 2500 2700
F 0 "C38" H 2525 2800 50  0000 L CNN
F 1 "22nF 50V" H 2525 2600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2538 2550 50  0001 C CNN
F 3 "" H 2500 2700 50  0000 C CNN
F 4 "1414682" H 2500 2700 60  0001 C CNN "Farnell"
F 5 "6V" H 2500 2700 60  0001 C CNN "Notes"
F 6 "0.0246" H 2500 2700 60  0001 C CNN "Price"
	1    2500 2700
	1    0    0    -1  
$EndComp
$Comp
L R R33
U 1 1 585F6FA4
P 2500 3050
F 0 "R33" V 2580 3050 50  0000 C CNN
F 1 "22k" V 2500 3050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2430 3050 50  0001 C CNN
F 3 "" H 2500 3050 50  0000 C CNN
F 4 "0.01" V 2500 3050 60  0001 C CNN "Price"
F 5 "9238646" V 2500 3050 60  0001 C CNN "Farnell"
	1    2500 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 3250 3500 3250
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
L +36V #PWR054
U 1 1 585F700E
P 950 1450
F 0 "#PWR054" H 950 1300 50  0001 C CNN
F 1 "+36V" H 950 1590 50  0000 C CNN
F 2 "" H 950 1450 50  0000 C CNN
F 3 "" H 950 1450 50  0000 C CNN
	1    950  1450
	1    0    0    -1  
$EndComp
$Comp
L D D4
U 1 1 585F7042
P 1300 1600
F 0 "D4" H 1300 1700 50  0000 C CNN
F 1 "60V 2A" H 1300 1500 50  0000 C CNN
F 2 "Diodes_SMD:SOD-123" H 1300 1600 50  0001 C CNN
F 3 "" H 1300 1600 50  0000 C CNN
F 4 "http://uk.farnell.com/panasonic-electronic-components/db2w60400l/diode-schottky-60v-mini2-f3-b/dp/2284958" H 1300 1600 60  0001 C CNN "Order"
	1    1300 1600
	-1   0    0    1   
$EndComp
$Comp
L C C36
U 1 1 585F709D
P 1550 1800
F 0 "C36" H 1575 1900 50  0000 L CNN
F 1 "220nF 100V" H 1300 1700 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 1588 1650 50  0001 C CNN
F 3 "" H 1550 1800 50  0000 C CNN
F 4 "1856615" H 1550 1800 60  0001 C CNN "Farnell"
F 5 "0.205" H 1550 1800 60  0001 C CNN "Price"
	1    1550 1800
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR L2
U 1 1 585F70E9
P 2000 1600
F 0 "L2" V 1950 1600 50  0000 C CNN
F 1 "47uH" V 2100 1600 50  0000 C CNN
F 2 "Inductors:SELF-WE-PD3S" H 2000 1600 50  0001 C CNN
F 3 "" H 2000 1600 50  0000 C CNN
F 4 "http://uk.farnell.com/tdk/clf7045t-470m/inductor-47uh-1-3a-20-smd/dp/2345091" V 2000 1600 60  0001 C CNN "Order"
F 5 "2345091" V 2000 1600 60  0001 C CNN "Farnell"
F 6 "0.765" V 2000 1600 60  0001 C CNN "Price"
	1    2000 1600
	0    1    1    0   
$EndComp
$Comp
L C C39
U 1 1 585F7125
P 2700 1850
F 0 "C39" H 2725 1950 50  0000 L CNN
F 1 "220nF 100V" H 2725 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 2738 1700 50  0001 C CNN
F 3 "" H 2700 1850 50  0000 C CNN
F 4 "60V" H 2700 1850 60  0001 C CNN "Note"
F 5 "1856615" H 2700 1850 60  0001 C CNN "Farnell"
	1    2700 1850
	1    0    0    -1  
$EndComp
$Comp
L CP C37
U 1 1 585F715C
P 2400 1850
F 0 "C37" H 2425 1950 50  0000 L CNN
F 1 "100uF 63V" H 2150 1750 50  0000 L CNN
F 2 "Capacitors_SMD:c_elec_10x10.5" H 2438 1700 50  0001 C CNN
F 3 "" H 2400 1850 50  0000 C CNN
F 4 "9696040" H 2400 1850 60  0001 C CNN "Farnell"
F 5 "0.43" H 2400 1850 60  0001 C CNN "Price"
	1    2400 1850
	1    0    0    -1  
$EndComp
$Comp
L C C40
U 1 1 585F72AD
P 4350 2250
F 0 "C40" H 4375 2350 50  0000 L CNN
F 1 "220nF 25V" H 4375 2150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4388 2100 50  0001 C CNN
F 3 "" H 4350 2250 50  0000 C CNN
F 4 "6V" H 4350 2250 60  0001 C CNN "Notes"
F 5 "1856615" H 4350 2250 60  0001 C CNN "Farnell"
F 6 "0.205" H 4350 2250 60  0001 C CNN "Price"
	1    4350 2250
	1    0    0    -1  
$EndComp
$Comp
L D_Schottky D6
U 1 1 585F731A
P 4350 2700
F 0 "D6" H 4350 2800 50  0000 C CNN
F 1 "60V 2A" H 4350 2600 50  0000 C CNN
F 2 "Diodes_SMD:SOD-123" H 4350 2700 50  0001 C CNN
F 3 "" H 4350 2700 50  0000 C CNN
F 4 "2284958" H 4350 2700 60  0001 C CNN "Farnell"
	1    4350 2700
	0    1    1    0   
$EndComp
$Comp
L CP C41
U 1 1 585F737F
P 5300 2700
F 0 "C41" H 5325 2800 50  0000 L CNN
F 1 "100uF 6.3V" H 4900 2600 50  0000 L CNN
F 2 "Capacitors_SMD:c_elec_6.3x5.8" H 5338 2550 50  0001 C CNN
F 3 "" H 5300 2700 50  0000 C CNN
F 4 "2611349" H 5300 2700 60  0001 C CNN "Farnell"
F 5 "0.138" H 5300 2700 60  0001 C CNN "Price"
	1    5300 2700
	1    0    0    -1  
$EndComp
$Comp
L C C42
U 1 1 585F73CE
P 5550 2700
F 0 "C42" H 5575 2800 50  0000 L CNN
F 1 "220nF 100V" H 5400 2600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 5588 2550 50  0001 C CNN
F 3 "" H 5550 2700 50  0000 C CNN
F 4 "6V" H 5550 2700 60  0001 C CNN "Notes"
F 5 "1856615" H 5550 2700 60  0001 C CNN "Farnell"
F 6 "0.205" H 5550 2700 60  0001 C CNN "Price"
	1    5550 2700
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR L3
U 1 1 585F74CD
P 4800 2450
F 0 "L3" V 4750 2450 50  0000 C CNN
F 1 "47uH" V 4900 2450 50  0000 C CNN
F 2 "Inductors:SELF-WE-PD3S" H 4800 2450 50  0001 C CNN
F 3 "" H 4800 2450 50  0000 C CNN
F 4 "http://uk.farnell.com/tdk/clf7045t-470m/inductor-47uh-1-3a-20-smd/dp/2345091" V 4800 2450 60  0001 C CNN "Order"
F 5 "https://www.digikey.co.uk/product-detail/en/tdk-corporation/CLF7045T-470M/445-9527-1-ND/3909700" V 4800 2450 60  0001 C CNN "Order2"
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
L GND #PWR055
U 1 1 585F787D
P 4350 2950
F 0 "#PWR055" H 4350 2700 50  0001 C CNN
F 1 "GND" H 4350 2800 50  0000 C CNN
F 2 "" H 4350 2950 50  0000 C CNN
F 3 "" H 4350 2950 50  0000 C CNN
	1    4350 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 2950 4350 2850
$Comp
L GND #PWR056
U 1 1 585F7983
P 5450 2950
F 0 "#PWR056" H 5450 2700 50  0001 C CNN
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
L GND #PWR057
U 1 1 585F7C36
P 1550 2050
F 0 "#PWR057" H 1550 1800 50  0001 C CNN
F 1 "GND" H 1550 1900 50  0000 C CNN
F 2 "" H 1550 2050 50  0000 C CNN
F 3 "" H 1550 2050 50  0000 C CNN
	1    1550 2050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR058
U 1 1 585F7C68
P 2400 2050
F 0 "#PWR058" H 2400 1800 50  0001 C CNN
F 1 "GND" H 2400 1900 50  0000 C CNN
F 2 "" H 2400 2050 50  0000 C CNN
F 3 "" H 2400 2050 50  0000 C CNN
	1    2400 2050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR059
U 1 1 585F7C9A
P 2700 2050
F 0 "#PWR059" H 2700 1800 50  0001 C CNN
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
	3350 1450 3350 1600
Connection ~ 3350 1600
$Comp
L ZENER D7
U 1 1 585F8179
P 5900 2700
F 0 "D7" H 5900 2800 50  0000 C CNN
F 1 "5.6V Zener" H 5900 2900 50  0000 C CNN
F 2 "Diodes_SMD:SOD-123" H 5900 2700 50  0001 C CNN
F 3 "" H 5900 2700 50  0000 C CNN
F 4 "MMSZ4690T1G" H 5900 2700 60  0001 C CNN "Part"
F 5 "http://uk.farnell.com/on-semiconductor/mmsz5v6t1g/diode-zener-5-6v-0-5w/dp/1431292" H 5900 2700 60  0001 C CNN "Order"
F 6 "1459122" H 5900 2700 60  0001 C CNN "Farnell"
F 7 "863-SZMMSZ5V6T1G" H 5900 2700 60  0001 C CNN "Mouser"
F 8 "?" H 5900 2700 60  0001 C CNN "Supplier"
F 9 "0.16" H 5900 2700 60  0001 C CNN "Price"
F 10 "MMSZ5V6T1G" H 5900 2700 60  0001 C CNN "OldPart"
	1    5900 2700
	0    1    1    0   
$EndComp
Wire Wire Line
	5900 2950 5900 2900
Connection ~ 5550 2950
Wire Wire Line
	5900 2450 5900 2500
Connection ~ 5550 2450
Connection ~ 5900 2950
Connection ~ 5900 2450
$Comp
L IFX91041EJV50 U4
U 1 1 58659107
P 3450 3100
F 0 "U4" H 3450 3000 60  0000 C CNN
F 1 "IFX91041EJV50" H 3450 3100 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-8-1EP_3.9x4.9mm_Pitch1.27mm" H 3450 3100 60  0001 C CNN
F 3 "http://www.infineon.com/dgdl/Infineon-IFX91041-DS-v01_01-en.pdf?fileId=db3a304320d39d59012153afb91a0548" H 3450 3100 60  0001 C CNN
F 4 "https://www.digikey.co.uk/product-detail/en/infineon-technologies/IFX91041EJ-V50/IFX91041EJ-V50INCT-ND/3300395" H 3450 3100 60  0001 C CNN "Order"
F 5 "2432504" H 3450 3100 60  0001 C CNN "Farnell"
F 6 "1.10" H 3450 3100 60  0001 C CNN "Price"
	1    3450 3100
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR060
U 1 1 586591BB
P 5300 2050
F 0 "#PWR060" H 5300 1900 50  0001 C CNN
F 1 "+5V" H 5300 2190 50  0000 C CNN
F 2 "" H 5300 2050 50  0000 C CNN
F 3 "" H 5300 2050 50  0000 C CNN
	1    5300 2050
	1    0    0    -1  
$EndComp
$Comp
L D D5
U 1 1 58659257
P 1600 1300
F 0 "D5" H 1600 1400 50  0000 C CNN
F 1 "60V 2A" H 1600 1200 50  0000 C CNN
F 2 "Diodes_SMD:SOD-123" H 1600 1300 50  0001 C CNN
F 3 "http://www.farnell.com/datasheets/1912070.pdf?_ga=1.4639027.1808009743.1471515981" H 1600 1300 50  0001 C CNN
F 4 "http://uk.farnell.com/panasonic-electronic-components/db2w60400l/diode-schottky-60v-mini2-f3-b/dp/2284958" H 1600 1300 60  0001 C CNN "Order"
F 5 "DB2W60400L" H 1600 1300 60  0001 C CNN "Type"
F 6 "2284958" H 1600 1300 60  0001 C CNN "Farnell"
	1    1600 1300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1600 1450 1600 1600
Connection ~ 1600 1600
$Comp
L +8V #PWR061
U 1 1 5865931B
P 1600 1000
F 0 "#PWR061" H 1600 850 50  0001 C CNN
F 1 "+8V" H 1600 1140 50  0000 C CNN
F 2 "" H 1600 1000 50  0000 C CNN
F 3 "" H 1600 1000 50  0000 C CNN
	1    1600 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 1000 1600 1150
$Comp
L GND #PWR062
U 1 1 586595C6
P 6600 3150
F 0 "#PWR062" H 6600 2900 50  0001 C CNN
F 1 "GND" H 6600 3000 50  0000 C CNN
F 2 "" H 6600 3150 50  0000 C CNN
F 3 "" H 6600 3150 50  0000 C CNN
	1    6600 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 3150 6600 2950
$Comp
L +3V3 #PWR063
U 1 1 58659680
P 7300 2300
F 0 "#PWR063" H 7300 2150 50  0001 C CNN
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
L C C43
U 1 1 5865982D
P 7250 2750
F 0 "C43" H 7275 2850 50  0000 L CNN
F 1 "1uF 50V" H 7275 2650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7288 2600 50  0001 C CNN
F 3 "" H 7250 2750 50  0000 C CNN
F 4 "4V" H 7250 2750 60  0001 C CNN "Notes"
	1    7250 2750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR064
U 1 1 586598C9
P 7250 3000
F 0 "#PWR064" H 7250 2750 50  0001 C CNN
F 1 "GND" H 7250 2850 50  0000 C CNN
F 2 "" H 7250 3000 50  0000 C CNN
F 3 "" H 7250 3000 50  0000 C CNN
	1    7250 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 3000 7250 2900
Wire Wire Line
	7250 2450 7250 2600
Connection ~ 7250 2450
$Comp
L ZENER D8
U 1 1 58659A6D
P 7900 2800
F 0 "D8" H 7900 2900 50  0000 C CNN
F 1 "3.6V Zener" H 7900 2700 50  0000 C CNN
F 2 "Diodes_SMD:SOD-123" H 7900 2800 50  0001 C CNN
F 3 "" H 7900 2800 50  0000 C CNN
F 4 "http://uk.farnell.com/on-semiconductor/mmsz4685t1g/diode-zener-vz-3-6v/dp/1651592" H 7900 2800 60  0001 C CNN "Order"
F 5 "MMSZ5227B" H 7900 2800 60  0001 C CNN "Part"
F 6 "1700854" H 7900 2800 60  0001 C CNN "Farnell"
F 7 "MMSZ4685T1G" H 7900 2800 60  0001 C CNN "PartPrevious"
F 8 "0.155" H 7900 2800 60  0001 C CNN "Price"
	1    7900 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	7900 2150 7900 2600
Connection ~ 7300 2450
Connection ~ 7900 2450
Wire Wire Line
	5300 2950 5900 2950
$Comp
L GND #PWR065
U 1 1 5865A3E7
P 7900 3100
F 0 "#PWR065" H 7900 2850 50  0001 C CNN
F 1 "GND" H 7900 2950 50  0000 C CNN
F 2 "" H 7900 3100 50  0000 C CNN
F 3 "" H 7900 3100 50  0000 C CNN
	1    7900 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 3000 7900 3100
Wire Wire Line
	5100 2450 6200 2450
$Comp
L TEST_1P W12
U 1 1 586E8A7C
P 5750 2200
F 0 "W12" H 5750 2470 50  0000 C CNN
F 1 "5V" H 5750 2400 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 5950 2200 50  0001 C CNN
F 3 "" H 5950 2200 50  0000 C CNN
	1    5750 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 2200 5750 2450
Connection ~ 5750 2450
$Comp
L TEST_1P W13
U 1 1 586E8BDF
P 8250 1950
F 0 "W13" H 8250 2220 50  0000 C CNN
F 1 "3V3" H 8250 2150 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 8450 1950 50  0001 C CNN
F 3 "" H 8450 1950 50  0000 C CNN
	1    8250 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 2150 8250 1950
$Comp
L TEST_1P W10
U 1 1 586EC5DC
P 3000 1400
F 0 "W10" H 3000 1670 50  0000 C CNN
F 1 "DSUP" H 3000 1600 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 3200 1400 50  0001 C CNN
F 3 "" H 3200 1400 50  0000 C CNN
	1    3000 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1400 3000 1600
Connection ~ 3000 1600
$Comp
L PWR_FLAG #FLG066
U 1 1 5874647E
P 6000 2250
F 0 "#FLG066" H 6000 2345 50  0001 C CNN
F 1 "PWR_FLAG" H 6000 2430 50  0000 C CNN
F 2 "" H 6000 2250 50  0000 C CNN
F 3 "" H 6000 2250 50  0000 C CNN
	1    6000 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 2250 6000 2450
Connection ~ 6000 2450
$Comp
L PWR_FLAG #FLG067
U 1 1 58746947
P 1900 1050
F 0 "#FLG067" H 1900 1145 50  0001 C CNN
F 1 "PWR_FLAG" H 1900 1230 50  0000 C CNN
F 2 "" H 1900 1050 50  0000 C CNN
F 3 "" H 1900 1050 50  0000 C CNN
	1    1900 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 1050 1900 1150
Wire Wire Line
	1600 1150 2050 1150
$Comp
L PWR_FLAG #FLG068
U 1 1 58746A06
P 900 2000
F 0 "#FLG068" H 900 2095 50  0001 C CNN
F 1 "PWR_FLAG" H 900 2180 50  0000 C CNN
F 2 "" H 900 2000 50  0000 C CNN
F 3 "" H 900 2000 50  0000 C CNN
	1    900  2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  2000 1100 2000
Wire Wire Line
	1100 2000 1100 1600
Connection ~ 1100 1600
$Comp
L PWR_FLAG #FLG069
U 1 1 58764812
P 3350 1450
F 0 "#FLG069" H 3350 1545 50  0001 C CNN
F 1 "PWR_FLAG" H 3350 1630 50  0000 C CNN
F 2 "" H 3350 1450 50  0000 C CNN
F 3 "" H 3350 1450 50  0000 C CNN
	1    3350 1450
	1    0    0    -1  
$EndComp
Connection ~ 7250 2550
$Comp
L TC2117-3.3V U5
U 1 1 5865A268
P 6600 2550
F 0 "U5" H 6600 2950 60  0000 C CNN
F 1 "TC2117-3.3V" H 6600 2850 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-223" H 6600 2550 60  0001 C CNN
F 3 "http://www.farnell.com/datasheets/1669439.pdf?_ga=1.267823246.1808009743.1471515981" H 6600 2550 60  0001 C CNN
F 4 "http://uk.farnell.com/microchip/tc2117-3-3vdbtr/ic-cmso-ldo-3-3v-800ma-sot223/dp/1605581" H 6600 2550 60  0001 C CNN "Order"
F 5 "1605581" H 6600 2550 60  0001 C CNN "Farnell"
F 6 "0.669" H 6600 2550 60  0001 C CNN "Price "
	1    6600 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 2550 7250 2550
$Comp
L R R15
U 1 1 59C4B75A
P 2300 1050
F 0 "R15" V 2380 1050 50  0000 C CNN
F 1 "10K" V 2300 1050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2230 1050 50  0001 C CNN
F 3 "" H 2300 1050 50  0000 C CNN
	1    2300 1050
	0    1    1    0   
$EndComp
$Comp
L C C51
U 1 1 59C4B98C
P 2600 1200
F 0 "C51" H 2625 1300 50  0000 L CNN
F 1 "0.1uF 50V" H 2350 1100 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2638 1050 50  0001 C CNN
F 3 "" H 2600 1200 50  0000 C CNN
F 4 "+- 10%" H 2600 1200 60  0001 C CNN "Notes"
F 5 "0.116" H 2600 1200 60  0001 C CNN "Price"
	1    2600 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 1150 2050 1050
Wire Wire Line
	2050 1050 2150 1050
Connection ~ 1900 1150
Wire Wire Line
	2450 1050 3150 1050
$Comp
L GND #PWR070
U 1 1 59C4BBA7
P 2600 1400
F 0 "#PWR070" H 2600 1150 50  0001 C CNN
F 1 "GND" H 2600 1250 50  0000 C CNN
F 2 "" H 2600 1400 50  0000 C CNN
F 3 "" H 2600 1400 50  0000 C CNN
	1    2600 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 1400 2600 1350
Wire Wire Line
	3150 1050 3150 2100
Wire Wire Line
	3150 2100 3350 2100
Wire Wire Line
	3350 2100 3350 2200
Connection ~ 2600 1050
$Comp
L Jumper_NO_Small JP3
U 1 1 59C4C18F
P 3350 1800
F 0 "JP3" H 3350 1880 50  0000 C CNN
F 1 "Jumper_NO_Small" H 3360 1740 50  0001 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 3350 1800 50  0001 C CNN
F 3 "" H 3350 1800 50  0000 C CNN
F 4 "None" H 3350 1800 60  0001 C CNN "Supplier"
	1    3350 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 1800 3550 1800
Connection ~ 3550 1800
Wire Wire Line
	3250 1800 3150 1800
Connection ~ 3150 1800
Wire Wire Line
	7900 2150 8250 2150
Text Notes 3500 1550 0    60   ~ 0
When JP3 is not fitted the external +8V line acts as \na master power switch for all the boards.
$EndSCHEMATC
