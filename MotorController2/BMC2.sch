EESchema Schematic File Version 3
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
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 6850 2050 1400 3200
U 5851C864
F0 "power" 60
F1 "power.sch" 60
F2 "PhaseA" O R 8250 2400 60 
F3 "PhaseB" O R 8250 2550 60 
F4 "PhaseC" O R 8250 2700 60 
F5 "VSenseA" O L 6850 2200 60 
F6 "VSenseB" O L 6850 2300 60 
F7 "VSenseC" O L 6850 2400 60 
F8 "ISenseA" O L 6850 2550 60 
F9 "ISenseB" O L 6850 2650 60 
F10 "ISenseC" O L 6850 2750 60 
F11 "SCLK" I L 6850 2950 60 
F12 "MISO" O L 6850 3050 60 
F13 "MOSI" I L 6850 3150 60 
F14 "sSCS" I L 6850 3250 60 
F15 "PWRGD" O L 6850 3400 60 
F16 "nFAULT" O L 6850 3500 60 
F17 "WAKE" I L 6850 3700 60 
F18 "EN_GATE" I L 6850 3800 60 
F19 "INHA" I L 6850 4250 60 
F20 "INLA" I L 6850 4350 60 
F21 "INHB" I L 6850 4450 60 
F22 "INLB" I L 6850 4550 60 
F23 "INHC" I L 6850 4650 60 
F24 "INLC" I L 6850 4750 60 
F25 "VSupplySense" O L 6850 2850 60 
F26 "TempDrv" I L 6850 5000 60 
$EndSheet
$Sheet
S 5550 850  500  700 
U 585F64FD
F0 "Regulator" 60
F1 "Regulator.sch" 60
$EndSheet
$Comp
L TJA1042T/3 U1
U 1 1 586549EB
P 1900 4000
F 0 "U1" H 1500 4350 50  0000 L CNN
F 1 "TJA1042T/3" H 1950 4350 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 1900 3500 50  0001 C CIN
F 3 "" H 1900 4000 50  0000 C CNN
F 4 "http://uk.farnell.com/nxp/tja1042t-3-1j/can-transceiver-aec-q100-2mbps/dp/2574957" H 1900 4000 60  0001 C CNN "Order"
	1    1900 4000
	-1   0    0    -1  
$EndComp
$Comp
L +5V #PWR01
U 1 1 58654E2C
P 1900 3450
F 0 "#PWR01" H 1900 3300 50  0001 C CNN
F 1 "+5V" H 1900 3590 50  0000 C CNN
F 2 "" H 1900 3450 50  0000 C CNN
F 3 "" H 1900 3450 50  0000 C CNN
	1    1900 3450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 5865537A
P 1900 4500
F 0 "#PWR02" H 1900 4250 50  0001 C CNN
F 1 "GND" H 1900 4350 50  0000 C CNN
F 2 "" H 1900 4500 50  0000 C CNN
F 3 "" H 1900 4500 50  0000 C CNN
	1    1900 4500
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR03
U 1 1 586556AA
P 2550 4100
F 0 "#PWR03" H 2550 3950 50  0001 C CNN
F 1 "+3V3" H 2550 4240 50  0000 C CNN
F 2 "" H 2550 4100 50  0000 C CNN
F 3 "" H 2550 4100 50  0000 C CNN
	1    2550 4100
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P2
U 1 1 58656D5A
P 750 4000
F 0 "P2" H 750 4250 50  0000 C CNN
F 1 "CAN" V 850 4000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x04" H 750 4000 50  0001 C CNN
F 3 "" H 750 4000 50  0000 C CNN
F 4 "588738" H 750 4000 60  0001 C CNN "Farnell"
F 5 "1098455" H 750 4000 60  0001 C CNN "Farnell Related"
	1    750  4000
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 58657281
P 1100 3600
F 0 "#PWR04" H 1100 3350 50  0001 C CNN
F 1 "GND" H 1100 3450 50  0000 C CNN
F 2 "" H 1100 3600 50  0000 C CNN
F 3 "" H 1100 3600 50  0000 C CNN
	1    1100 3600
	1    0    0    -1  
$EndComp
$Comp
L +8V #PWR05
U 1 1 5865817E
P 1300 3650
F 0 "#PWR05" H 1300 3500 50  0001 C CNN
F 1 "+8V" H 1300 3790 50  0000 C CNN
F 2 "" H 1300 3650 50  0000 C CNN
F 3 "" H 1300 3650 50  0000 C CNN
	1    1300 3650
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5865BC3F
P 1150 5250
F 0 "C1" H 1175 5350 50  0000 L CNN
F 1 "0.1uF 50V" H 1175 5150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1188 5100 50  0001 C CNN
F 3 "" H 1150 5250 50  0000 C CNN
F 4 "1759265" H 1150 5250 60  0001 C CNN "Farnell"
	1    1150 5250
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR06
U 1 1 5865BDDB
P 1150 5050
F 0 "#PWR06" H 1150 4900 50  0001 C CNN
F 1 "+5V" H 1150 5190 50  0000 C CNN
F 2 "" H 1150 5050 50  0000 C CNN
F 3 "" H 1150 5050 50  0000 C CNN
	1    1150 5050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 5865BF5D
P 1150 5500
F 0 "#PWR07" H 1150 5250 50  0001 C CNN
F 1 "GND" H 1150 5350 50  0000 C CNN
F 2 "" H 1150 5500 50  0000 C CNN
F 3 "" H 1150 5500 50  0000 C CNN
	1    1150 5500
	1    0    0    -1  
$EndComp
$Comp
L USB_OTG-RESCUE-BMC2 P4
U 1 1 5865C76F
P 2000 5350
F 0 "P4" H 2325 5225 50  0000 C CNN
F 1 "USB_OTG" H 2000 5550 50  0000 C CNN
F 2 "Connect:USB_Mini-B" V 1950 5250 50  0001 C CNN
F 3 "" V 1950 5250 50  0000 C CNN
F 4 "http://uk.farnell.com/molex/67503-1020/mini-usb-2-0-otg-type-b-rcpt-smt/dp/1125348" H 2000 5350 60  0001 C CNN "Order"
	1    2000 5350
	0    -1   1    0   
$EndComp
$Comp
L R R9
U 1 1 5865DF1E
P 2800 5150
F 0 "R9" V 2880 5150 50  0000 C CNN
F 1 "22R" V 2800 5150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2730 5150 50  0001 C CNN
F 3 "" H 2800 5150 50  0000 C CNN
	1    2800 5150
	0    1    1    0   
$EndComp
$Comp
L R R10
U 1 1 5865E23C
P 2800 5300
F 0 "R10" V 2880 5300 50  0000 C CNN
F 1 "22R" V 2800 5300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2730 5300 50  0001 C CNN
F 3 "" H 2800 5300 50  0000 C CNN
	1    2800 5300
	0    1    1    0   
$EndComp
$Comp
L GND #PWR08
U 1 1 5865E9C4
P 2350 5900
F 0 "#PWR08" H 2350 5650 50  0001 C CNN
F 1 "GND" H 2350 5750 50  0000 C CNN
F 2 "" H 2350 5900 50  0000 C CNN
F 3 "" H 2350 5900 50  0000 C CNN
	1    2350 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 2500 5200 2500
Wire Wire Line
	5200 2500 5200 2200
Wire Wire Line
	5200 2200 6850 2200
Wire Wire Line
	5000 2600 5300 2600
Wire Wire Line
	5300 2600 5300 2300
Wire Wire Line
	5300 2300 6850 2300
Wire Wire Line
	5000 2700 5400 2700
Wire Wire Line
	5400 2700 5400 2400
Wire Wire Line
	5400 2400 6850 2400
Wire Wire Line
	6850 2550 5500 2550
Wire Wire Line
	5500 2550 5500 2850
Wire Wire Line
	5500 2850 5000 2850
Wire Wire Line
	5000 2950 5600 2950
Wire Wire Line
	5600 2950 5600 2650
Wire Wire Line
	5600 2650 6850 2650
Wire Wire Line
	6850 2750 5700 2750
Wire Wire Line
	5700 2750 5700 3050
Wire Wire Line
	5700 3050 5000 3050
Wire Wire Line
	6850 2850 5800 2850
Wire Wire Line
	5800 2850 5800 3150
Wire Wire Line
	5800 3150 5000 3150
Wire Wire Line
	5000 3250 5900 3250
Wire Wire Line
	5900 3250 5900 2950
Wire Wire Line
	5900 2950 6850 2950
Wire Wire Line
	6850 3050 6000 3050
Wire Wire Line
	6000 3050 6000 3350
Wire Wire Line
	6000 3350 5000 3350
Wire Wire Line
	5000 3450 6100 3450
Wire Wire Line
	6100 3450 6100 3150
Wire Wire Line
	6100 3150 6850 3150
Wire Wire Line
	5000 3550 6250 3550
Wire Wire Line
	6250 3550 6250 3250
Wire Wire Line
	6250 3250 6850 3250
Wire Wire Line
	6850 3400 6350 3400
Wire Wire Line
	6350 3400 6350 3750
Wire Wire Line
	6350 3750 5000 3750
Wire Wire Line
	5000 3850 6450 3850
Wire Wire Line
	6450 3850 6450 3500
Wire Wire Line
	6450 3500 6850 3500
Wire Wire Line
	6850 3700 6550 3700
Wire Wire Line
	6550 3700 6550 4000
Wire Wire Line
	6550 4000 5000 4000
Wire Wire Line
	5000 4100 6650 4100
Wire Wire Line
	6650 4100 6650 3800
Wire Wire Line
	6650 3800 6850 3800
Wire Wire Line
	5000 4350 5100 4350
Wire Wire Line
	5100 4350 5100 4250
Wire Wire Line
	5100 4250 6850 4250
Wire Wire Line
	6850 4350 5200 4350
Wire Wire Line
	5200 4350 5200 4450
Wire Wire Line
	5200 4450 5000 4450
Wire Wire Line
	5000 4550 5300 4550
Wire Wire Line
	5300 4550 5300 4450
Wire Wire Line
	5300 4450 6850 4450
Wire Wire Line
	5400 4550 6850 4550
Wire Wire Line
	5400 4550 5400 4650
Wire Wire Line
	5400 4650 5000 4650
Wire Wire Line
	5000 4750 5500 4750
Wire Wire Line
	5500 4750 5500 4650
Wire Wire Line
	5500 4650 6850 4650
Wire Wire Line
	6850 4750 5600 4750
Wire Wire Line
	5600 4750 5600 4850
Wire Wire Line
	5600 4850 5000 4850
Wire Wire Line
	2400 3800 2800 3800
Wire Wire Line
	2800 3800 2800 3950
Wire Wire Line
	2800 3950 3500 3950
Wire Wire Line
	2700 4050 3500 4050
Wire Wire Line
	2700 4050 2700 3900
Wire Wire Line
	2700 3900 2400 3900
Wire Wire Line
	1900 3450 1900 3600
Wire Wire Line
	1900 4500 1900 4400
Wire Wire Line
	2550 4100 2400 4100
Wire Wire Line
	3500 4200 2400 4200
Wire Wire Line
	1400 4100 1300 4100
Wire Wire Line
	1300 4100 1300 4700
Wire Wire Line
	1300 4150 950  4150
Wire Wire Line
	950  4050 1200 4050
Wire Wire Line
	1200 3900 1200 4600
Wire Wire Line
	1200 3900 1400 3900
Wire Wire Line
	950  3950 1100 3950
Wire Wire Line
	1100 3800 1100 4500
Wire Wire Line
	1100 3800 1300 3800
Wire Wire Line
	1300 3800 1300 3650
Wire Wire Line
	1150 5500 1150 5400
Wire Wire Line
	1150 5100 1150 5050
Wire Wire Line
	2950 5150 3150 5150
Wire Wire Line
	3150 5150 3150 5200
Wire Wire Line
	3150 5200 3500 5200
Wire Wire Line
	3500 5300 2950 5300
Wire Wire Line
	2300 5250 2500 5250
Wire Wire Line
	2500 5250 2500 5150
Wire Wire Line
	2500 5150 2650 5150
Wire Wire Line
	2300 5350 2550 5350
Wire Wire Line
	2550 5350 2550 5300
Wire Wire Line
	2550 5300 2650 5300
Wire Wire Line
	2350 5550 2350 5900
Wire Wire Line
	2350 5550 2300 5550
Wire Wire Line
	1900 5750 2350 5750
Connection ~ 2350 5750
NoConn ~ 2300 5450
$Comp
L D_Schottky D1
U 1 1 58660759
P 2350 4900
F 0 "D1" H 2350 5000 50  0000 C CNN
F 1 "60V 2A" H 2350 4800 50  0000 C CNN
F 2 "Diodes_SMD:SOD-123" H 2350 4900 50  0001 C CNN
F 3 "" H 2350 4900 50  0000 C CNN
F 4 "2284958" H 2350 4900 60  0001 C CNN "Farnell"
	1    2350 4900
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR09
U 1 1 58660AEB
P 2350 4550
F 0 "#PWR09" H 2350 4400 50  0001 C CNN
F 1 "+5V" H 2350 4690 50  0000 C CNN
F 2 "" H 2350 4550 50  0000 C CNN
F 3 "" H 2350 4550 50  0000 C CNN
	1    2350 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 4750 2350 4550
$Comp
L GND #PWR010
U 1 1 58665413
P 1100 1650
F 0 "#PWR010" H 1100 1400 50  0001 C CNN
F 1 "GND" H 1100 1500 50  0000 C CNN
F 2 "" H 1100 1650 50  0000 C CNN
F 3 "" H 1100 1650 50  0000 C CNN
	1    1100 1650
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 58665E66
P 1800 2400
F 0 "R4" V 1880 2400 50  0000 C CNN
F 1 "1k" V 1800 2400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1730 2400 50  0001 C CNN
F 3 "" H 1800 2400 50  0000 C CNN
	1    1800 2400
	0    1    1    0   
$EndComp
$Comp
L R R5
U 1 1 5866600D
P 1800 2550
F 0 "R5" V 1880 2550 50  0000 C CNN
F 1 "1k" V 1800 2550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1730 2550 50  0001 C CNN
F 3 "" H 1800 2550 50  0000 C CNN
	1    1800 2550
	0    1    1    0   
$EndComp
$Comp
L R R7
U 1 1 586680AE
P 1400 3000
F 0 "R7" V 1480 3000 50  0000 C CNN
F 1 "10K" V 1400 3000 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1330 3000 50  0001 C CNN
F 3 "" H 1400 3000 50  0000 C CNN
	1    1400 3000
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5866831A
P 1650 3000
F 0 "C2" H 1675 3100 50  0000 L CNN
F 1 "0.1uF 50V" H 1450 2900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1688 2850 50  0001 C CNN
F 3 "" H 1650 3000 50  0000 C CNN
	1    1650 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 2850 2600 2850
Wire Wire Line
	1400 3150 1650 3150
$Comp
L GND #PWR012
U 1 1 586688E0
P 1500 3250
F 0 "#PWR012" H 1500 3000 50  0001 C CNN
F 1 "GND" H 1500 3100 50  0000 C CNN
F 2 "" H 1500 3250 50  0000 C CNN
F 3 "" H 1500 3250 50  0000 C CNN
	1    1500 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 3250 1500 3150
Connection ~ 1500 3150
Wire Wire Line
	950  2650 1350 2650
Wire Wire Line
	1350 2650 1350 2850
Connection ~ 1400 2850
Connection ~ 1650 2850
Wire Wire Line
	2600 3400 3500 3400
Wire Wire Line
	2600 2850 2600 3400
$Comp
L R R6
U 1 1 5866A88A
P 1800 2700
F 0 "R6" V 1880 2700 50  0000 C CNN
F 1 "1k" V 1800 2700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1730 2700 50  0001 C CNN
F 3 "" H 1800 2700 50  0000 C CNN
	1    1800 2700
	0    1    1    0   
$EndComp
Wire Wire Line
	1950 2700 2650 2700
Wire Wire Line
	2650 2700 2650 3300
Wire Wire Line
	2650 3300 3500 3300
Wire Wire Line
	3500 3200 2700 3200
Wire Wire Line
	2700 3200 2700 2550
Wire Wire Line
	2700 2550 1950 2550
Wire Wire Line
	3500 3100 2750 3100
Wire Wire Line
	2750 3100 2750 2400
Wire Wire Line
	2750 2400 1950 2400
Wire Wire Line
	2800 2250 2800 3000
Wire Wire Line
	950  2250 2800 2250
Wire Wire Line
	2850 2150 2850 2900
Wire Wire Line
	950  2150 2850 2150
Wire Wire Line
	2900 2050 2900 2800
Wire Wire Line
	1650 2700 1450 2700
Wire Wire Line
	1450 2700 1450 2550
Wire Wire Line
	1450 2550 950  2550
Wire Wire Line
	1650 2550 1550 2550
Wire Wire Line
	1550 2550 1550 2450
Wire Wire Line
	1550 2450 950  2450
Wire Wire Line
	1650 2400 1650 2400
Wire Wire Line
	1650 2400 1650 2350
Wire Wire Line
	1650 2350 950  2350
$Comp
L CONN_01X05 P3
U 1 1 586CDC2C
P 850 900
F 0 "P3" H 850 1200 50  0000 C CNN
F 1 "DATA" V 950 900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x05" H 850 900 50  0001 C CNN
F 3 "" H 850 900 50  0000 C CNN
F 4 "588740" H 850 900 60  0001 C CNN "Farnell"
	1    850  900 
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 586CE0C4
P 1350 1200
F 0 "#PWR013" H 1350 950 50  0001 C CNN
F 1 "GND" H 1350 1050 50  0000 C CNN
F 2 "" H 1350 1200 50  0000 C CNN
F 3 "" H 1350 1200 50  0000 C CNN
	1    1350 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 1100 1350 1100
Wire Wire Line
	1350 1100 1350 1200
Wire Wire Line
	1050 1000 3550 1000
Wire Wire Line
	3500 2500 3150 2500
Wire Wire Line
	3150 2500 3150 900 
Wire Wire Line
	3150 900  1050 900 
Wire Wire Line
	3500 2400 3300 2400
Wire Wire Line
	3300 2400 3300 800 
Wire Wire Line
	3300 800  1050 800 
Wire Wire Line
	3500 2300 3400 2300
Wire Wire Line
	3400 2300 3400 700 
Wire Wire Line
	3400 700  1050 700 
$Comp
L CONN_01X01 P5
U 1 1 586D399A
P 8750 2400
F 0 "P5" H 8750 2500 50  0000 C CNN
F 1 "PhaseA" H 8850 2400 50  0000 C CNN
F 2 "Connect:1pin" H 8750 2400 50  0001 C CNN
F 3 "" H 8750 2400 50  0000 C CNN
	1    8750 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 2400 8550 2400
$Comp
L CONN_01X01 P6
U 1 1 586D4187
P 8750 2650
F 0 "P6" H 8750 2750 50  0000 C CNN
F 1 "PhaseB" H 8850 2650 50  0000 C CNN
F 2 "Connect:1pin" H 8750 2650 50  0001 C CNN
F 3 "" H 8750 2650 50  0000 C CNN
	1    8750 2650
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P7
U 1 1 586D4351
P 8750 2900
F 0 "P7" H 8750 3000 50  0000 C CNN
F 1 "PhaseC" H 8850 2900 50  0000 C CNN
F 2 "Connect:1pin" H 8750 2900 50  0001 C CNN
F 3 "" H 8750 2900 50  0000 C CNN
	1    8750 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 2650 8450 2650
Wire Wire Line
	8450 2650 8450 2550
Wire Wire Line
	8450 2550 8250 2550
Wire Wire Line
	8250 2700 8300 2700
Wire Wire Line
	8300 2700 8300 2900
Wire Wire Line
	8300 2900 8550 2900
$Comp
L TEST_1P W1
U 1 1 586E727E
P 2850 3850
F 0 "W1" H 2850 4120 50  0000 C CNN
F 1 "CAN_TX" H 2850 4050 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 3050 3850 50  0001 C CNN
F 3 "" H 3050 3850 50  0000 C CNN
	1    2850 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 3850 2850 3950
Connection ~ 2850 3950
$Comp
L TEST_1P W2
U 1 1 586E75E8
P 3150 3850
F 0 "W2" H 3150 4120 50  0000 C CNN
F 1 "CAN_RX" H 3150 4050 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 3350 3850 50  0001 C CNN
F 3 "" H 3350 3850 50  0000 C CNN
	1    3150 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 3850 3150 4050
Connection ~ 3150 4050
Wire Wire Line
	5000 5050 5900 5050
Wire Wire Line
	5900 5050 5900 5000
Wire Wire Line
	5900 5000 6850 5000
Wire Wire Line
	2300 5150 2350 5150
Wire Wire Line
	2350 5150 2350 5050
$Comp
L CONN_01X09 P1
U 1 1 58736653
P 750 2250
F 0 "P1" H 750 2750 50  0000 C CNN
F 1 "Sensor" V 850 2250 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x09" H 750 2250 50  0001 C CNN
F 3 "" H 750 2250 50  0000 C CNN
F 4 "2468365" H 750 2250 60  0001 C CNN "Farnell"
F 5 "1098695" H 750 2250 60  0001 C CNN "Plug Farnell"
	1    750  2250
	-1   0    0    1   
$EndComp
$Comp
L +36V #PWR015
U 1 1 58747DA2
P 7750 1350
F 0 "#PWR015" H 7750 1200 50  0001 C CNN
F 1 "+36V" H 7750 1490 50  0000 C CNN
F 2 "" H 7750 1350 50  0000 C CNN
F 3 "" H 7750 1350 50  0000 C CNN
	1    7750 1350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 58747F64
P 7800 1650
F 0 "#PWR016" H 7800 1400 50  0001 C CNN
F 1 "GND" H 7800 1500 50  0000 C CNN
F 2 "" H 7800 1650 50  0000 C CNN
F 3 "" H 7800 1650 50  0000 C CNN
	1    7800 1650
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P9
U 1 1 5874BB2E
P 8400 1350
F 0 "P9" H 8400 1450 50  0000 C CNN
F 1 "VSUPPLY" H 8500 1350 50  0000 C CNN
F 2 "Connect:1pin" H 8400 1350 50  0001 C CNN
F 3 "" H 8400 1350 50  0000 C CNN
	1    8400 1350
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P10
U 1 1 5874BD40
P 8400 1650
F 0 "P10" H 8400 1750 50  0000 C CNN
F 1 "GND" H 8500 1650 50  0000 C CNN
F 2 "Connect:1pin" H 8400 1650 50  0001 C CNN
F 3 "" H 8400 1650 50  0000 C CNN
	1    8400 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 1650 8200 1650
Wire Wire Line
	8200 1350 7750 1350
$Comp
L PWR_FLAG #FLG017
U 1 1 5874C5D3
P 8150 1950
F 0 "#FLG017" H 8150 2045 50  0001 C CNN
F 1 "PWR_FLAG" H 8150 2130 50  0000 C CNN
F 2 "" H 8150 1950 50  0000 C CNN
F 3 "" H 8150 1950 50  0000 C CNN
	1    8150 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 1950 7950 1950
Wire Wire Line
	7950 1950 7950 1650
Connection ~ 7950 1650
$Comp
L CONN_01X04 P12
U 1 1 58700494
P 750 4550
F 0 "P12" H 750 4800 50  0000 C CNN
F 1 "CAN" V 850 4550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x04" H 750 4550 50  0001 C CNN
F 3 "" H 750 4550 50  0000 C CNN
F 4 "588738" H 750 4550 60  0001 C CNN "Farnell"
	1    750  4550
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1200 4600 950  4600
Connection ~ 1200 4050
Wire Wire Line
	1300 4700 950  4700
Connection ~ 1300 4150
Wire Wire Line
	1100 4500 950  4500
Connection ~ 1100 3950
Wire Wire Line
	950  3850 1000 3850
Wire Wire Line
	1000 3600 1000 4400
Wire Wire Line
	1000 4400 950  4400
Wire Wire Line
	1000 3600 1100 3600
Connection ~ 1000 3850
Wire Wire Line
	1300 1950 950  1950
Wire Wire Line
	1000 1650 1000 1850
Wire Wire Line
	1000 1850 950  1850
Wire Wire Line
	1000 1650 1100 1650
Wire Wire Line
	2900 2800 3500 2800
Wire Wire Line
	2850 2900 3500 2900
Wire Wire Line
	2800 3000 3500 3000
$Comp
L +3V3 #PWR020
U 1 1 587B9A34
P 1600 4650
F 0 "#PWR020" H 1600 4500 50  0001 C CNN
F 1 "+3V3" H 1600 4790 50  0000 C CNN
F 2 "" H 1600 4650 50  0000 C CNN
F 3 "" H 1600 4650 50  0000 C CNN
	1    1600 4650
	1    0    0    -1  
$EndComp
$Comp
L C C44
U 1 1 587B9C10
P 1600 4850
F 0 "C44" H 1625 4950 50  0000 L CNN
F 1 "0.1uF 50V" H 1625 4750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1638 4700 50  0001 C CNN
F 3 "" H 1600 4850 50  0000 C CNN
	1    1600 4850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR021
U 1 1 587B9DFF
P 1600 5100
F 0 "#PWR021" H 1600 4850 50  0001 C CNN
F 1 "GND" H 1600 4950 50  0000 C CNN
F 2 "" H 1600 5100 50  0000 C CNN
F 3 "" H 1600 5100 50  0000 C CNN
	1    1600 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 5100 1600 5000
Wire Wire Line
	1600 4700 1600 4650
$Comp
L C C45
U 1 1 588C67F6
P 1950 3050
F 0 "C45" H 1975 3150 50  0000 L CNN
F 1 "1nF 50V" H 1750 2850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1988 2900 50  0001 C CNN
F 3 "" H 1950 3050 50  0000 C CNN
F 4 "9406344" H 1950 3050 60  0001 C CNN "Farnell"
	1    1950 3050
	1    0    0    -1  
$EndComp
$Comp
L C C46
U 1 1 588C7228
P 2200 3050
F 0 "C46" H 2225 3150 50  0000 L CNN
F 1 "1nF 50V" H 2050 2950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2238 2900 50  0001 C CNN
F 3 "" H 2200 3050 50  0000 C CNN
F 4 "9406344" H 2200 3050 60  0001 C CNN "Farnell"
	1    2200 3050
	1    0    0    -1  
$EndComp
$Comp
L C C47
U 1 1 588C7422
P 2450 3050
F 0 "C47" H 2475 3150 50  0000 L CNN
F 1 "1nF 50V" H 2350 2850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2488 2900 50  0001 C CNN
F 3 "" H 2450 3050 50  0000 C CNN
F 4 "9406344" H 2450 3050 60  0001 C CNN "Farnell"
	1    2450 3050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 588C7613
P 2250 3300
F 0 "#PWR022" H 2250 3050 50  0001 C CNN
F 1 "GND" H 2250 3150 50  0000 C CNN
F 2 "" H 2250 3300 50  0000 C CNN
F 3 "" H 2250 3300 50  0000 C CNN
	1    2250 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3200 2450 3200
Connection ~ 2200 3200
Wire Wire Line
	2200 3200 2200 3300
Wire Wire Line
	2200 3300 2250 3300
Wire Wire Line
	1950 2900 1950 2900
Wire Wire Line
	1950 2900 1950 2700
Connection ~ 1950 2700
Wire Wire Line
	2200 2900 2200 2900
Wire Wire Line
	2200 2900 2200 2550
Connection ~ 2200 2550
Wire Wire Line
	2450 2400 2450 2900
Connection ~ 2450 2400
Wire Wire Line
	1300 1600 1300 1950
Wire Wire Line
	2950 2700 3500 2700
Wire Wire Line
	2950 1450 2950 2700
$Comp
L MIC2009A-1YM6TR U6
U 1 1 59C62C29
P 2150 1450
F 0 "U6" H 2125 1820 50  0000 C CNN
F 1 "MIC2009A-1YM6TR" H 2125 1729 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-6" H 1800 1800 50  0001 L BNN
F 3 "SOT-23-6 Microchip" H 2950 1350 50  0001 L BNN
F 4 "MIC2009A-1YM6-TR" H 2900 1400 50  0001 L BNN "MP"
F 5 "Good" H 3000 1700 50  0001 L BNN "Availability"
F 6 "0.37 USD" H 3050 1600 50  0001 L BNN "Price"
F 7 "Microchip" H 3100 1500 50  0001 L BNN "MF"
F 8 "MIC2009 Series 1 Channel 5.5 V Adj. High Side Power Distribution Switch SOT-23-6" H 2700 1100 50  0001 L BNN "Description"
	1    2150 1450
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 59C633FF
P 2700 1900
F 0 "#PWR011" H 2700 1650 50  0001 C CNN
F 1 "GND" H 2700 1750 50  0000 C CNN
F 2 "" H 2700 1900 50  0000 C CNN
F 3 "" H 2700 1900 50  0000 C CNN
	1    2700 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 1650 2700 1900
Wire Wire Line
	2700 1650 2650 1650
$Comp
L +3V3 #PWR017
U 1 1 59C64144
P 2750 1300
F 0 "#PWR017" H 2750 1150 50  0001 C CNN
F 1 "+3V3" H 2750 1440 50  0000 C CNN
F 2 "" H 2750 1300 50  0000 C CNN
F 3 "" H 2750 1300 50  0000 C CNN
	1    2750 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 1350 2750 1350
Wire Wire Line
	2750 1350 2750 1300
Wire Wire Line
	1300 1600 1700 1600
Wire Wire Line
	950  2050 2900 2050
Wire Wire Line
	2950 1450 2650 1450
$Comp
L MIC2009A-1YM6TR U7
U 1 1 59C66257
P 4250 1250
F 0 "U7" H 4225 1620 50  0000 C CNN
F 1 "MIC2009A-1YM6TR" H 4225 1529 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-6" H 3900 1600 50  0001 L BNN
F 3 "SOT-23-6 Microchip" H 5050 1150 50  0001 L BNN
F 4 "MIC2009A-1YM6-TR" H 5000 1200 50  0001 L BNN "MP"
F 5 "Good" H 5100 1500 50  0001 L BNN "Availability"
F 6 "0.37 USD" H 5150 1400 50  0001 L BNN "Price"
F 7 "Microchip" H 5200 1300 50  0001 L BNN "MF"
F 8 "MIC2009 Series 1 Channel 5.5 V Adj. High Side Power Distribution Switch SOT-23-6" H 4800 900 50  0001 L BNN "Description"
	1    4250 1250
	-1   0    0    -1  
$EndComp
$Comp
L +5V #PWR014
U 1 1 586CE33C
P 4800 1100
F 0 "#PWR014" H 4800 950 50  0001 C CNN
F 1 "+5V" H 4800 1240 50  0000 C CNN
F 2 "" H 4800 1100 50  0000 C CNN
F 3 "" H 4800 1100 50  0000 C CNN
	1    4800 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 1150 4800 1150
Wire Wire Line
	4800 1150 4800 1100
Wire Wire Line
	3550 1000 3550 1400
Wire Wire Line
	3550 1400 3800 1400
$Comp
L GND #PWR019
U 1 1 59C6AA3F
P 4800 1600
F 0 "#PWR019" H 4800 1350 50  0001 C CNN
F 1 "GND" H 4800 1450 50  0000 C CNN
F 2 "" H 4800 1600 50  0000 C CNN
F 3 "" H 4800 1600 50  0000 C CNN
	1    4800 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 1600 4800 1450
Wire Wire Line
	4800 1450 4750 1450
Wire Wire Line
	5150 1250 5150 2400
Wire Wire Line
	5150 1250 4750 1250
$Comp
L R R1
U 1 1 59C6B084
P 2850 1700
F 0 "R1" V 2930 1700 50  0000 C CNN
F 1 "1k" V 2850 1700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2780 1700 50  0001 C CNN
F 3 "" H 2850 1700 50  0000 C CNN
	1    2850 1700
	-1   0    0    1   
$EndComp
Wire Wire Line
	2650 1550 2850 1550
Text Notes 2500 1900 2    60   ~ 0
Output limited to 200ma\n
Wire Wire Line
	5150 2400 5000 2400
$Sheet
S 3500 2250 1500 3350
U 5851BC18
F0 "Controller" 59
F1 "Controller.sch" 59
F2 "VSenseA" I R 5000 2500 60 
F3 "VSenseB" I R 5000 2600 60 
F4 "VSenseC" I R 5000 2700 60 
F5 "ISenseA" I R 5000 2850 60 
F6 "ISenseB" I R 5000 2950 60 
F7 "ISenseC" I R 5000 3050 60 
F8 "VSenseSupply" I R 5000 3150 60 
F9 "CAN_TX" O L 3500 3950 60 
F10 "CAN_RX" I L 3500 4050 60 
F11 "INHA" O R 5000 4350 60 
F12 "INHB" O R 5000 4550 60 
F13 "INHC" O R 5000 4750 60 
F14 "INLA" O R 5000 4450 60 
F15 "INLB" O R 5000 4650 60 
F16 "INLC" O R 5000 4850 60 
F17 "EN_GATE" O R 5000 4100 60 
F18 "FAULT" I R 5000 3850 60 
F19 "HallA" I L 3500 3100 60 
F20 "HallB" I L 3500 3200 60 
F21 "HallC" I L 3500 3300 60 
F22 "UART_TX" O L 3500 2400 60 
F23 "UART_RX" I L 3500 2500 60 
F24 "EndStop1" I L 3500 3000 60 
F25 "EndStop2" I L 3500 2900 60 
F26 "Index" I L 3500 2800 60 
F27 "MOSI" O R 5000 3450 60 
F28 "MISO" I R 5000 3350 60 
F29 "SCLK" O R 5000 3250 60 
F30 "USB_DM" B L 3500 5200 60 
F31 "USB_DP" B L 3500 5300 60 
F32 "Temp" I L 3500 3400 60 
F33 "Wake" O R 5000 4000 60 
F34 "sSCS" O R 5000 3550 60 
F35 "PWRGD" I R 5000 3750 60 
F36 "CAN_STBY" O L 3500 4200 60 
F37 "NRST" I L 3500 2300 60 
F38 "TempDrv" I R 5000 5050 60 
F39 "FanCtrl" O R 5000 2400 60 
F40 "SensorPwrCtrl" O L 3500 2700 60 
F41 "SENSOR_PWR_FAULT" I L 3500 2600 60 
F42 "FAN_FAULT" I R 5000 2300 60 
$EndSheet
Wire Wire Line
	1700 1500 1700 1100
Wire Wire Line
	1700 1100 3050 1100
Wire Wire Line
	3050 1100 3050 2600
Wire Wire Line
	3050 2600 3500 2600
Wire Wire Line
	5000 2300 5050 2300
Wire Wire Line
	5050 2300 5050 2050
Wire Wire Line
	5050 2050 3650 2050
Wire Wire Line
	3650 2050 3650 1300
Wire Wire Line
	3650 1300 3800 1300
$Comp
L R R2
U 1 1 59C7B2C7
P 5000 1600
F 0 "R2" V 5080 1600 50  0000 C CNN
F 1 "300R" V 5000 1600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4930 1600 50  0001 C CNN
F 3 "" H 5000 1600 50  0000 C CNN
	1    5000 1600
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR018
U 1 1 59C7B6BD
P 2850 1900
F 0 "#PWR018" H 2850 1650 50  0001 C CNN
F 1 "GND" H 2850 1750 50  0000 C CNN
F 2 "" H 2850 1900 50  0000 C CNN
F 3 "" H 2850 1900 50  0000 C CNN
	1    2850 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 1900 2850 1850
$Comp
L GND #PWR044
U 1 1 59C7BEF7
P 5000 1800
F 0 "#PWR044" H 5000 1550 50  0001 C CNN
F 1 "GND" H 5000 1650 50  0000 C CNN
F 2 "" H 5000 1800 50  0000 C CNN
F 3 "" H 5000 1800 50  0000 C CNN
	1    5000 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 1800 5000 1750
Wire Wire Line
	5000 1450 5000 1350
Wire Wire Line
	5000 1350 4750 1350
Text Notes 4700 1750 2    60   ~ 0
Current limit to 700mA\n
$EndSCHEMATC
