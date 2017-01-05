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
S 3350 2250 1500 3350
U 5851BC18
F0 "Controller" 59
F1 "Controller.sch" 59
F2 "VSenseA" I R 4850 2400 60 
F3 "VSenseB" I R 4850 2500 60 
F4 "VSenseC" I R 4850 2600 60 
F5 "ISenseA" I R 4850 2750 60 
F6 "ISenseB" I R 4850 2850 60 
F7 "ISenseC" I R 4850 2950 60 
F8 "VSenseSupply" I R 4850 3100 60 
F9 "CAN_TX" O L 3350 3950 60 
F10 "CAN_RX" I L 3350 4050 60 
F11 "INHA" O R 4850 4350 60 
F12 "INHB" O R 4850 4550 60 
F13 "INHC" O R 4850 4750 60 
F14 "INLA" O R 4850 4450 60 
F15 "INLB" O R 4850 4650 60 
F16 "INLC" O R 4850 4850 60 
F17 "EN_GATE" O R 4850 4100 60 
F18 "FAULT" I R 4850 3850 60 
F19 "HallA" I L 3350 3100 60 
F20 "HallB" I L 3350 3200 60 
F21 "HallC" I L 3350 3300 60 
F22 "UART_TX" O L 3350 2400 60 
F23 "UART_RX" I L 3350 2500 60 
F24 "EndStop1" I L 3350 2750 60 
F25 "EndStop2" I L 3350 2850 60 
F26 "Index" I L 3350 2950 60 
F27 "MOSI" O R 4850 3400 60 
F28 "MISO" I R 4850 3300 60 
F29 "SCLK" O R 4850 3200 60 
F30 "USB_DM" B L 3350 5200 60 
F31 "USB_DP" B L 3350 5300 60 
F32 "I2C_SCL" B L 3350 4800 60 
F33 "I2C_SDA" B L 3350 4900 60 
F34 "Temp" I L 3350 3400 60 
F35 "Wake" O R 4850 4000 60 
F36 "sSCS" O R 4850 3500 60 
F37 "PWRGD" I R 4850 3750 60 
F38 "CAN_STBY" O L 3350 4200 60 
F39 "NRST" I L 3350 2300 60 
F40 "TempDrv" I R 4850 5050 60 
$EndSheet
$Sheet
S 6700 2050 1400 3200
U 5851C864
F0 "power" 60
F1 "power.sch" 60
F2 "PhaseA" O R 8100 2400 60 
F3 "PhaseB" O R 8100 2550 60 
F4 "PhaseC" O R 8100 2700 60 
F5 "VSenseA" O L 6700 2200 60 
F6 "VSenseB" O L 6700 2300 60 
F7 "VSenseC" O L 6700 2400 60 
F8 "ISenseA" O L 6700 2550 60 
F9 "ISenseB" O L 6700 2650 60 
F10 "ISenseC" O L 6700 2750 60 
F11 "SCLK" I L 6700 2950 60 
F12 "MISO" O L 6700 3050 60 
F13 "MOSI" I L 6700 3150 60 
F14 "sSCS" I L 6700 3250 60 
F15 "PWRGD" O L 6700 3400 60 
F16 "nFAULT" O L 6700 3500 60 
F17 "WAKE" I L 6700 3700 60 
F18 "EN_GATE" I L 6700 3800 60 
F19 "INHA" I L 6700 4250 60 
F20 "INLA" I L 6700 4350 60 
F21 "INHB" I L 6700 4450 60 
F22 "INLB" I L 6700 4550 60 
F23 "INHC" I L 6700 4650 60 
F24 "INLC" I L 6700 4750 60 
F25 "VSupplySense" O L 6700 2850 60 
F26 "TempDrv" I L 6700 5000 60 
$EndSheet
$Sheet
S 3700 950  500  700 
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
P 800 4000
F 0 "P2" H 800 4250 50  0000 C CNN
F 1 "CAN" V 900 4000 50  0000 C CNN
F 2 "Sockets_MOLEX_KK-System:Socket_MOLEX-KK-RM2-54mm_Lock_4pin_straight" H 800 4000 50  0001 C CNN
F 3 "" H 800 4000 50  0000 C CNN
	1    800  4000
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
P 1200 4650
F 0 "C1" H 1225 4750 50  0000 L CNN
F 1 "0.1uf (6V min)" H 1225 4550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1238 4500 50  0001 C CNN
F 3 "" H 1200 4650 50  0000 C CNN
	1    1200 4650
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR06
U 1 1 5865BDDB
P 1200 4450
F 0 "#PWR06" H 1200 4300 50  0001 C CNN
F 1 "+5V" H 1200 4590 50  0000 C CNN
F 2 "" H 1200 4450 50  0000 C CNN
F 3 "" H 1200 4450 50  0000 C CNN
	1    1200 4450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 5865BF5D
P 1200 4900
F 0 "#PWR07" H 1200 4650 50  0001 C CNN
F 1 "GND" H 1200 4750 50  0000 C CNN
F 2 "" H 1200 4900 50  0000 C CNN
F 3 "" H 1200 4900 50  0000 C CNN
	1    1200 4900
	1    0    0    -1  
$EndComp
$Comp
L USB_OTG P4
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
	4850 2400 5100 2400
Wire Wire Line
	5100 2400 5100 2200
Wire Wire Line
	5100 2200 6700 2200
Wire Wire Line
	4850 2500 5200 2500
Wire Wire Line
	5200 2500 5200 2300
Wire Wire Line
	5200 2300 6700 2300
Wire Wire Line
	4850 2600 5300 2600
Wire Wire Line
	5300 2600 5300 2400
Wire Wire Line
	5300 2400 6700 2400
Wire Wire Line
	6700 2550 5400 2550
Wire Wire Line
	5400 2550 5400 2750
Wire Wire Line
	5400 2750 4850 2750
Wire Wire Line
	4850 2850 5500 2850
Wire Wire Line
	5500 2850 5500 2650
Wire Wire Line
	5500 2650 6700 2650
Wire Wire Line
	6700 2750 5600 2750
Wire Wire Line
	5600 2750 5600 2950
Wire Wire Line
	5600 2950 4850 2950
Wire Wire Line
	6700 2850 5700 2850
Wire Wire Line
	5700 2850 5700 3100
Wire Wire Line
	5700 3100 4850 3100
Wire Wire Line
	4850 3200 5800 3200
Wire Wire Line
	5800 3200 5800 2950
Wire Wire Line
	5800 2950 6700 2950
Wire Wire Line
	6700 3050 5900 3050
Wire Wire Line
	5900 3050 5900 3300
Wire Wire Line
	5900 3300 4850 3300
Wire Wire Line
	4850 3400 6000 3400
Wire Wire Line
	6000 3400 6000 3150
Wire Wire Line
	6000 3150 6700 3150
Wire Wire Line
	4850 3500 6100 3500
Wire Wire Line
	6100 3500 6100 3250
Wire Wire Line
	6100 3250 6700 3250
Wire Wire Line
	6700 3400 6200 3400
Wire Wire Line
	6200 3400 6200 3750
Wire Wire Line
	6200 3750 4850 3750
Wire Wire Line
	4850 3850 6300 3850
Wire Wire Line
	6300 3850 6300 3500
Wire Wire Line
	6300 3500 6700 3500
Wire Wire Line
	6700 3700 6400 3700
Wire Wire Line
	6400 3700 6400 4000
Wire Wire Line
	6400 4000 4850 4000
Wire Wire Line
	4850 4100 6500 4100
Wire Wire Line
	6500 4100 6500 3800
Wire Wire Line
	6500 3800 6700 3800
Wire Wire Line
	4850 4350 5100 4350
Wire Wire Line
	5100 4350 5100 4250
Wire Wire Line
	5100 4250 6700 4250
Wire Wire Line
	6700 4350 5200 4350
Wire Wire Line
	5200 4350 5200 4450
Wire Wire Line
	5200 4450 4850 4450
Wire Wire Line
	4850 4550 5300 4550
Wire Wire Line
	5300 4550 5300 4450
Wire Wire Line
	5300 4450 6700 4450
Wire Wire Line
	5400 4550 6700 4550
Wire Wire Line
	5400 4550 5400 4650
Wire Wire Line
	5400 4650 4850 4650
Wire Wire Line
	4850 4750 5500 4750
Wire Wire Line
	5500 4750 5500 4650
Wire Wire Line
	5500 4650 6700 4650
Wire Wire Line
	6700 4750 5600 4750
Wire Wire Line
	5600 4750 5600 4850
Wire Wire Line
	5600 4850 4850 4850
Wire Wire Line
	2400 3800 2800 3800
Wire Wire Line
	2800 3800 2800 3950
Wire Wire Line
	2800 3950 3350 3950
Wire Wire Line
	2700 4050 3350 4050
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
	3350 4200 2400 4200
Wire Wire Line
	1400 4100 1250 4100
Wire Wire Line
	1250 4100 1250 4150
Wire Wire Line
	1250 4150 1000 4150
Wire Wire Line
	1000 4050 1200 4050
Wire Wire Line
	1200 4050 1200 3900
Wire Wire Line
	1200 3900 1400 3900
Wire Wire Line
	1000 3850 1000 3600
Wire Wire Line
	1000 3600 1100 3600
Wire Wire Line
	1000 3950 1100 3950
Wire Wire Line
	1100 3950 1100 3800
Wire Wire Line
	1100 3800 1300 3800
Wire Wire Line
	1300 3800 1300 3650
Wire Wire Line
	1200 4900 1200 4800
Wire Wire Line
	1200 4500 1200 4450
Wire Wire Line
	2950 5150 3150 5150
Wire Wire Line
	3150 5150 3150 5200
Wire Wire Line
	3150 5200 3350 5200
Wire Wire Line
	3350 5300 2950 5300
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
F 1 "10V" H 2350 4800 50  0000 C CNN
F 2 "Diodes_SMD:DO-214AB_Handsoldering" H 2350 4900 50  0001 C CNN
F 3 "" H 2350 4900 50  0000 C CNN
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
L R R1
U 1 1 586642C1
P 1950 1950
F 0 "R1" V 2030 1950 50  0000 C CNN
F 1 "1k" V 1950 1950 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1880 1950 50  0001 C CNN
F 3 "" H 1950 1950 50  0000 C CNN
	1    1950 1950
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 58664FE3
P 1950 2100
F 0 "R2" V 2030 2100 50  0000 C CNN
F 1 "1k" V 1950 2100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1880 2100 50  0001 C CNN
F 3 "" H 1950 2100 50  0000 C CNN
	1    1950 2100
	0    1    1    0   
$EndComp
$Comp
L GND #PWR010
U 1 1 58665413
P 1050 3050
F 0 "#PWR010" H 1050 2800 50  0001 C CNN
F 1 "GND" H 1050 2900 50  0000 C CNN
F 2 "" H 1050 3050 50  0000 C CNN
F 3 "" H 1050 3050 50  0000 C CNN
	1    1050 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 2850 950  2850
$Comp
L +3V3 #PWR011
U 1 1 58665925
P 1250 3050
F 0 "#PWR011" H 1250 2900 50  0001 C CNN
F 1 "+3V3" H 1250 3190 50  0000 C CNN
F 2 "" H 1250 3050 50  0000 C CNN
F 3 "" H 1250 3050 50  0000 C CNN
	1    1250 3050
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 58665CC6
P 1950 2250
F 0 "R3" V 2030 2250 50  0000 C CNN
F 1 "1k" V 1950 2250 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1880 2250 50  0001 C CNN
F 3 "" H 1950 2250 50  0000 C CNN
	1    1950 2250
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 58665E66
P 1950 2400
F 0 "R4" V 2030 2400 50  0000 C CNN
F 1 "1k" V 1950 2400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1880 2400 50  0001 C CNN
F 3 "" H 1950 2400 50  0000 C CNN
	1    1950 2400
	0    1    1    0   
$EndComp
$Comp
L R R5
U 1 1 5866600D
P 1950 2550
F 0 "R5" V 2030 2550 50  0000 C CNN
F 1 "1k" V 1950 2550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1880 2550 50  0001 C CNN
F 3 "" H 1950 2550 50  0000 C CNN
	1    1950 2550
	0    1    1    0   
$EndComp
$Comp
L R R8
U 1 1 586661B3
P 2800 3450
F 0 "R8" V 2880 3450 50  0000 C CNN
F 1 "1k" V 2800 3450 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2730 3450 50  0001 C CNN
F 3 "" H 2800 3450 50  0000 C CNN
	1    2800 3450
	0    1    1    0   
$EndComp
$Comp
L R R7
U 1 1 586680AE
P 2100 3000
F 0 "R7" V 2180 3000 50  0000 C CNN
F 1 "10k" V 2100 3000 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2030 3000 50  0001 C CNN
F 3 "" H 2100 3000 50  0000 C CNN
	1    2100 3000
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5866831A
P 2350 3000
F 0 "C2" H 2375 3100 50  0000 L CNN
F 1 "0.1uF" H 2375 2900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2388 2850 50  0001 C CNN
F 3 "" H 2350 3000 50  0000 C CNN
	1    2350 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 2850 2550 2850
Wire Wire Line
	2100 3150 2350 3150
$Comp
L GND #PWR012
U 1 1 586688E0
P 2200 3250
F 0 "#PWR012" H 2200 3000 50  0001 C CNN
F 1 "GND" H 2200 3100 50  0000 C CNN
F 2 "" H 2200 3250 50  0000 C CNN
F 3 "" H 2200 3250 50  0000 C CNN
	1    2200 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 3250 2200 3150
Connection ~ 2200 3150
Wire Wire Line
	950  2650 1350 2650
Wire Wire Line
	1350 2650 1350 2850
Connection ~ 2100 2850
Connection ~ 2350 2850
Wire Wire Line
	2950 3450 3100 3450
Wire Wire Line
	3100 3450 3100 3400
Wire Wire Line
	3100 3400 3350 3400
Wire Wire Line
	2550 2850 2550 3450
Wire Wire Line
	2550 3450 2650 3450
$Comp
L R R6
U 1 1 5866A88A
P 1950 2700
F 0 "R6" V 2030 2700 50  0000 C CNN
F 1 "1k" V 1950 2700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 1880 2700 50  0001 C CNN
F 3 "" H 1950 2700 50  0000 C CNN
	1    1950 2700
	0    1    1    0   
$EndComp
Wire Wire Line
	950  2750 1150 2750
Wire Wire Line
	2100 2700 2650 2700
Wire Wire Line
	2650 2700 2650 3300
Wire Wire Line
	2650 3300 3350 3300
Wire Wire Line
	3350 3200 2700 3200
Wire Wire Line
	2700 3200 2700 2550
Wire Wire Line
	2700 2550 2100 2550
Wire Wire Line
	3350 3100 2750 3100
Wire Wire Line
	2750 3100 2750 2400
Wire Wire Line
	2750 2400 2100 2400
Wire Wire Line
	3350 2950 2800 2950
Wire Wire Line
	2800 2950 2800 2250
Wire Wire Line
	2800 2250 2100 2250
Wire Wire Line
	3350 2850 2850 2850
Wire Wire Line
	2850 2850 2850 2100
Wire Wire Line
	2850 2100 2100 2100
Wire Wire Line
	2100 1950 2900 1950
Wire Wire Line
	2900 1950 2900 2750
Wire Wire Line
	2900 2750 3350 2750
Wire Wire Line
	1800 2700 1450 2700
Wire Wire Line
	1450 2700 1450 2550
Wire Wire Line
	1450 2550 950  2550
Wire Wire Line
	1800 2550 1550 2550
Wire Wire Line
	1550 2550 1550 2450
Wire Wire Line
	1550 2450 950  2450
Wire Wire Line
	1800 2400 1650 2400
Wire Wire Line
	1650 2400 1650 2350
Wire Wire Line
	1650 2350 950  2350
Wire Wire Line
	950  2250 1800 2250
Wire Wire Line
	1800 2100 1550 2100
Wire Wire Line
	1550 2100 1550 2150
Wire Wire Line
	1550 2150 950  2150
Wire Wire Line
	950  2050 1450 2050
Wire Wire Line
	1450 2050 1450 1950
Wire Wire Line
	1450 1950 1800 1950
$Comp
L CONN_01X05 P3
U 1 1 586CDC2C
P 850 1250
F 0 "P3" H 850 1550 50  0000 C CNN
F 1 "DATA" V 950 1250 50  0000 C CNN
F 2 "Sockets_MOLEX_KK-System:Socket_MOLEX-KK-RM2-54mm_Lock_5pin_straight" H 850 1250 50  0001 C CNN
F 3 "" H 850 1250 50  0000 C CNN
	1    850  1250
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 586CE0C4
P 1150 1600
F 0 "#PWR013" H 1150 1350 50  0001 C CNN
F 1 "GND" H 1150 1450 50  0000 C CNN
F 2 "" H 1150 1600 50  0000 C CNN
F 3 "" H 1150 1600 50  0000 C CNN
	1    1150 1600
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR014
U 1 1 586CE33C
P 1450 1450
F 0 "#PWR014" H 1450 1300 50  0001 C CNN
F 1 "+5V" H 1450 1590 50  0000 C CNN
F 2 "" H 1450 1450 50  0000 C CNN
F 3 "" H 1450 1450 50  0000 C CNN
	1    1450 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 1450 1150 1450
Wire Wire Line
	1150 1450 1150 1600
Wire Wire Line
	1050 1350 1250 1350
Wire Wire Line
	1250 1350 1250 1550
Wire Wire Line
	1250 1550 1450 1550
Wire Wire Line
	1450 1550 1450 1450
Wire Wire Line
	3350 2500 2950 2500
Wire Wire Line
	2950 2500 2950 1250
Wire Wire Line
	2950 1250 1050 1250
Wire Wire Line
	3350 2400 3050 2400
Wire Wire Line
	3050 2400 3050 1150
Wire Wire Line
	3050 1150 1050 1150
Wire Wire Line
	3350 2300 3150 2300
Wire Wire Line
	3150 2300 3150 1050
Wire Wire Line
	3150 1050 1050 1050
$Comp
L CONN_01X01 P5
U 1 1 586D399A
P 8750 2400
F 0 "P5" H 8750 2500 50  0000 C CNN
F 1 "PhaseA" H 8850 2400 50  0000 C CNN
F 2 "Connect:Banana_Jack_1Pin" H 8750 2400 50  0001 C CNN
F 3 "" H 8750 2400 50  0000 C CNN
	1    8750 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 2400 8550 2400
$Comp
L CONN_01X01 P6
U 1 1 586D4187
P 8750 2650
F 0 "P6" H 8750 2750 50  0000 C CNN
F 1 "PhaseB" H 8850 2650 50  0000 C CNN
F 2 "Connect:Banana_Jack_1Pin" H 8750 2650 50  0001 C CNN
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
F 2 "Connect:Banana_Jack_1Pin" H 8750 2900 50  0001 C CNN
F 3 "" H 8750 2900 50  0000 C CNN
	1    8750 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 2650 8450 2650
Wire Wire Line
	8450 2650 8450 2550
Wire Wire Line
	8450 2550 8100 2550
Wire Wire Line
	8100 2700 8300 2700
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
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Big" H 3050 3850 50  0001 C CNN
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
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Big" H 3350 3850 50  0001 C CNN
F 3 "" H 3350 3850 50  0000 C CNN
	1    3150 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 3850 3150 4050
Connection ~ 3150 4050
Wire Wire Line
	4850 5050 5900 5050
Wire Wire Line
	5900 5050 5900 5000
Wire Wire Line
	5900 5000 6700 5000
Wire Wire Line
	2300 5150 2350 5150
Wire Wire Line
	2350 5150 2350 5050
$Comp
L CONN_01X09 P1
U 1 1 58736653
P 750 2450
F 0 "P1" H 750 2950 50  0000 C CNN
F 1 "Sensor" V 850 2450 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Angled_1x09" H 750 2450 50  0001 C CNN
F 3 "" H 750 2450 50  0000 C CNN
	1    750  2450
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1250 3050 1150 3050
Wire Wire Line
	1150 3050 1150 2750
Wire Wire Line
	1050 3050 1050 2850
NoConn ~ 4050 1350
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
F 2 "Connect:Banana_Jack_1Pin" H 8400 1350 50  0001 C CNN
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
F 2 "Connect:Banana_Jack_1Pin" H 8400 1650 50  0001 C CNN
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
L CONN_01X04 P11
U 1 1 586E38C6
P 2650 4650
F 0 "P11" H 2650 4900 50  0000 C CNN
F 1 "I2C" V 2750 4650 50  0000 C CNN
F 2 "Sockets_MOLEX_KK-System:Socket_MOLEX-KK-RM2-54mm_Lock_4pin_straight" H 2650 4650 50  0001 C CNN
F 3 "" H 2650 4650 50  0000 C CNN
F 4 "http://uk.farnell.com/molex/171856-0004/connector-header-tht-2-54mm-4way/dp/2366182" H 2650 4650 60  0001 C CNN "Order"
	1    2650 4650
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 586E3AFE
P 3150 4300
F 0 "#PWR018" H 3150 4050 50  0001 C CNN
F 1 "GND" H 3150 4150 50  0000 C CNN
F 2 "" H 3150 4300 50  0000 C CNN
F 3 "" H 3150 4300 50  0000 C CNN
	1    3150 4300
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR019
U 1 1 586E3CD7
P 3200 4650
F 0 "#PWR019" H 3200 4500 50  0001 C CNN
F 1 "+3V3" H 3200 4790 50  0000 C CNN
F 2 "" H 3200 4650 50  0000 C CNN
F 3 "" H 3200 4650 50  0000 C CNN
	1    3200 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 4500 2900 4500
Wire Wire Line
	2900 4500 2900 4300
Wire Wire Line
	2900 4300 3150 4300
Wire Wire Line
	2850 4600 3050 4600
Wire Wire Line
	3050 4600 3050 4650
Wire Wire Line
	3050 4650 3200 4650
Wire Wire Line
	3350 4800 3000 4800
Wire Wire Line
	3000 4800 3000 4700
Wire Wire Line
	3000 4700 2850 4700
Wire Wire Line
	2850 4800 2950 4800
Wire Wire Line
	2950 4800 2950 4900
Wire Wire Line
	2950 4900 3350 4900
$EndSCHEMATC
