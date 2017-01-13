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
LIBS:BMC2-cache
EELAYER 26 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 2 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 13450 4250 2    60   Input ~ 0
VSenseA
Text HLabel 13450 4350 2    60   Input ~ 0
VSenseB
Text HLabel 13450 4450 2    60   Input ~ 0
VSenseC
Text HLabel 13450 4550 2    60   Input ~ 0
ISenseA
Text HLabel 13450 4650 2    60   Input ~ 0
ISenseB
Text HLabel 13450 4750 2    60   Input ~ 0
ISenseC
Text HLabel 13450 4850 2    60   Input ~ 0
VSenseSupply
$Comp
L C C5
U 1 1 5856B354
P 3650 5100
F 0 "C5" H 3675 5200 50  0000 L CNN
F 1 "15pF" H 3675 5000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3688 4950 50  0001 C CNN
F 3 "" H 3650 5100 50  0000 C CNN
	1    3650 5100
	0    1    1    0   
$EndComp
$Comp
L C C6
U 1 1 5856B390
P 3650 5550
F 0 "C6" H 3675 5650 50  0000 L CNN
F 1 "15pF" H 3675 5450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3688 5400 50  0001 C CNN
F 3 "" H 3650 5550 50  0000 C CNN
	1    3650 5550
	0    1    1    0   
$EndComp
$Comp
L GND #PWR020
U 1 1 5856B3FF
P 3350 5650
F 0 "#PWR020" H 3350 5400 50  0001 C CNN
F 1 "GND" H 3350 5500 50  0000 C CNN
F 2 "" H 3350 5650 50  0000 C CNN
F 3 "" H 3350 5650 50  0000 C CNN
	1    3350 5650
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X05 P8
U 1 1 58585940
P 14800 5500
F 0 "P8" H 14800 5800 50  0000 C CNN
F 1 "CONN_02X05" H 14800 5200 50  0000 C CNN
F 2 "Local:SMT-HDR-10" H 14800 4300 50  0001 C CNN
F 3 "" H 14800 4300 50  0000 C CNN
F 4 "http://uk.rs-online.com/web/p/pcb-headers/0451732/?searchTerm=FTSH-105-01-F-DV-K&autocorrected=y&relevancy-data=636F3D3526696E3D4931384E4D616E506172744E756D626572266C753D656E266D6D3D6D61746368616C6C7061727469616C26706D3D5E5B5C707B4C7D5C707B4E647D2D2C2F255C2E5D2B2426706F3D3226736E3D592673723D4175746F636F727265637465642673613D6674736831303530316664762673743D4B4559574F52445F53494E474C455F414C5048415F4E554D455249432673633D592677633D4E4F4E45267573743D465453482D3130352D30312D462D44562D4B267374613D4654534831303530314644564B26" H 14800 5500 60  0001 C CNN "Order"
	1    14800 5500
	-1   0    0    -1  
$EndComp
$Comp
L +3V3 #PWR021
U 1 1 585859AF
P 15250 5150
F 0 "#PWR021" H 15250 5000 50  0001 C CNN
F 1 "+3V3" H 15250 5290 50  0000 C CNN
F 2 "" H 15250 5150 50  0000 C CNN
F 3 "" H 15250 5150 50  0000 C CNN
	1    15250 5150
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 585859E1
P 15300 5850
F 0 "#PWR022" H 15300 5600 50  0001 C CNN
F 1 "GND" H 15300 5700 50  0000 C CNN
F 2 "" H 15300 5850 50  0000 C CNN
F 3 "" H 15300 5850 50  0000 C CNN
	1    15300 5850
	-1   0    0    -1  
$EndComp
NoConn ~ 15050 5600
Text Notes 14350 5300 0    60   ~ 0
TMS
Text Notes 14350 5400 0    60   ~ 0
TCK
Text Notes 14350 5500 0    60   ~ 0
TDO
Text Notes 14400 5600 0    60   ~ 0
TDI
Text Notes 14350 5700 0    60   ~ 0
RST
Text HLabel 13100 6750 2    60   Output ~ 0
CAN_TX
Text HLabel 13100 6850 2    60   Input ~ 0
CAN_RX
Text HLabel 13250 5250 2    60   Output ~ 0
INHA
Text HLabel 13250 5150 2    60   Output ~ 0
INHB
Text HLabel 13250 5050 2    60   Output ~ 0
INHC
Text HLabel 13100 7450 2    60   Output ~ 0
INLA
Text HLabel 13100 7350 2    60   Output ~ 0
INLB
Text HLabel 13100 7250 2    60   Output ~ 0
INLC
Text HLabel 4300 7350 0    60   Output ~ 0
EN_GATE
Text HLabel 4300 7450 0    60   Input ~ 0
FAULT
Text HLabel 4350 5950 0    60   Input ~ 0
HallA
Text HLabel 4350 6050 0    60   Input ~ 0
HallB
Text HLabel 4350 6150 0    60   Input ~ 0
HallC
$Comp
L C C13
U 1 1 585D7176
P 9850 3100
F 0 "C13" H 9875 3200 50  0000 L CNN
F 1 "10nf" H 9875 3000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9888 2950 50  0001 C CNN
F 3 "" H 9850 3100 50  0000 C CNN
	1    9850 3100
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 585D71B4
P 7900 3050
F 0 "C10" H 7925 3150 50  0000 L CNN
F 1 "100nf" H 7925 2950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7938 2900 50  0001 C CNN
F 3 "" H 7900 3050 50  0000 C CNN
	1    7900 3050
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 585D7245
P 8650 3050
F 0 "C12" H 8675 3150 50  0000 L CNN
F 1 "100nf" H 8675 2950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8688 2900 50  0001 C CNN
F 3 "" H 8650 3050 50  0000 C CNN
	1    8650 3050
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 585D72FD
P 8250 3050
F 0 "C11" H 8275 3150 50  0000 L CNN
F 1 "100nf" H 8275 2950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8288 2900 50  0001 C CNN
F 3 "" H 8250 3050 50  0000 C CNN
	1    8250 3050
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 585D735D
P 7600 3050
F 0 "C9" H 7625 3150 50  0000 L CNN
F 1 "100nf 5V" H 7450 2900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 7638 2900 50  0001 C CNN
F 3 "" H 7600 3050 50  0000 C CNN
	1    7600 3050
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 585D7397
P 7200 3050
F 0 "C8" H 7225 3150 50  0000 L CNN
F 1 "4.7uf" H 7225 2950 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 7238 2900 50  0001 C CNN
F 3 "" H 7200 3050 50  0000 C CNN
	1    7200 3050
	1    0    0    -1  
$EndComp
$Comp
L C C14
U 1 1 585D746B
P 10150 3100
F 0 "C14" H 10175 3200 50  0000 L CNN
F 1 "1uf" H 10175 3000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 10188 2950 50  0001 C CNN
F 3 "" H 10150 3100 50  0000 C CNN
	1    10150 3100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR023
U 1 1 585D756A
P 8650 3300
F 0 "#PWR023" H 8650 3050 50  0001 C CNN
F 1 "GND" H 8650 3150 50  0000 C CNN
F 2 "" H 8650 3300 50  0000 C CNN
F 3 "" H 8650 3300 50  0000 C CNN
	1    8650 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR024
U 1 1 585D75A0
P 8250 3300
F 0 "#PWR024" H 8250 3050 50  0001 C CNN
F 1 "GND" H 8250 3150 50  0000 C CNN
F 2 "" H 8250 3300 50  0000 C CNN
F 3 "" H 8250 3300 50  0000 C CNN
	1    8250 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR025
U 1 1 585D75CF
P 7900 3300
F 0 "#PWR025" H 7900 3050 50  0001 C CNN
F 1 "GND" H 7900 3150 50  0000 C CNN
F 2 "" H 7900 3300 50  0000 C CNN
F 3 "" H 7900 3300 50  0000 C CNN
	1    7900 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR026
U 1 1 585D75FE
P 7600 3300
F 0 "#PWR026" H 7600 3050 50  0001 C CNN
F 1 "GND" H 7600 3150 50  0000 C CNN
F 2 "" H 7600 3300 50  0000 C CNN
F 3 "" H 7600 3300 50  0000 C CNN
	1    7600 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR027
U 1 1 585D7A19
P 7200 3300
F 0 "#PWR027" H 7200 3050 50  0001 C CNN
F 1 "GND" H 7200 3150 50  0000 C CNN
F 2 "" H 7200 3300 50  0000 C CNN
F 3 "" H 7200 3300 50  0000 C CNN
	1    7200 3300
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR028
U 1 1 585D7AA9
P 7200 2600
F 0 "#PWR028" H 7200 2450 50  0001 C CNN
F 1 "+3V3" H 7200 2740 50  0000 C CNN
F 2 "" H 7200 2600 50  0000 C CNN
F 3 "" H 7200 2600 50  0000 C CNN
	1    7200 2600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR029
U 1 1 585D7D30
P 9850 3350
F 0 "#PWR029" H 9850 3100 50  0001 C CNN
F 1 "GND" H 9850 3200 50  0000 C CNN
F 2 "" H 9850 3350 50  0000 C CNN
F 3 "" H 9850 3350 50  0000 C CNN
	1    9850 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR030
U 1 1 585D7D5F
P 10150 3350
F 0 "#PWR030" H 10150 3100 50  0001 C CNN
F 1 "GND" H 10150 3200 50  0000 C CNN
F 2 "" H 10150 3350 50  0000 C CNN
F 3 "" H 10150 3350 50  0000 C CNN
	1    10150 3350
	1    0    0    -1  
$EndComp
$Comp
L R R13
U 1 1 585D7FB9
P 9700 2500
F 0 "R13" V 9780 2500 50  0000 C CNN
F 1 "10" V 9700 2500 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 9630 2500 50  0001 C CNN
F 3 "" H 9700 2500 50  0000 C CNN
	1    9700 2500
	0    1    1    0   
$EndComp
$Comp
L INDUCTOR L1
U 1 1 585D80DD
P 9200 2500
F 0 "L1" V 9150 2500 50  0000 C CNN
F 1 "10uH" V 9300 2500 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 9200 2500 50  0001 C CNN
F 3 "" H 9200 2500 50  0000 C CNN
	1    9200 2500
	0    -1   -1   0   
$EndComp
$Comp
L C C7
U 1 1 585D83C5
P 4000 3800
F 0 "C7" H 4025 3900 50  0000 L CNN
F 1 "100nf" H 4025 3700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4038 3650 50  0001 C CNN
F 3 "" H 4000 3800 50  0000 C CNN
	1    4000 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR031
U 1 1 585D84A2
P 4000 4000
F 0 "#PWR031" H 4000 3750 50  0001 C CNN
F 1 "GND" H 4000 3850 50  0000 C CNN
F 2 "" H 4000 4000 50  0000 C CNN
F 3 "" H 4000 4000 50  0000 C CNN
	1    4000 4000
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW1
U 1 1 585D867B
P 3600 3550
F 0 "SW1" H 3750 3660 50  0000 C CNN
F 1 "RESET" H 3600 3470 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_FSMSM" H 3600 3550 50  0001 C CNN
F 3 "" H 3600 3550 50  0000 C CNN
	1    3600 3550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR032
U 1 1 585D874F
P 3300 3750
F 0 "#PWR032" H 3300 3500 50  0001 C CNN
F 1 "GND" H 3300 3600 50  0000 C CNN
F 2 "" H 3300 3750 50  0000 C CNN
F 3 "" H 3300 3750 50  0000 C CNN
	1    3300 3750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR033
U 1 1 585D8F48
P 2850 5200
F 0 "#PWR033" H 2850 4950 50  0001 C CNN
F 1 "GND" H 2850 5050 50  0000 C CNN
F 2 "" H 2850 5200 50  0000 C CNN
F 3 "" H 2850 5200 50  0000 C CNN
	1    2850 5200
	1    0    0    -1  
$EndComp
Text HLabel 13100 6550 2    60   Output ~ 0
UART_TX
Text HLabel 13100 6650 2    60   Input ~ 0
UART_RX
$Comp
L Jumper_NC_Small JP2
U 1 1 585D991C
P 14750 6350
F 0 "JP2" H 14750 6430 50  0000 C CNN
F 1 "Jumper_NC_Small" H 14760 6290 50  0001 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 14750 6350 50  0001 C CNN
F 3 "" H 14750 6350 50  0000 C CNN
	1    14750 6350
	1    0    0    -1  
$EndComp
$Comp
L Jumper_NO_Small JP1
U 1 1 585D997D
P 14400 6350
F 0 "JP1" H 14400 6430 50  0000 C CNN
F 1 "Jumper_NO_Small" H 14410 6290 50  0001 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" H 14400 6350 50  0001 C CNN
F 3 "" H 14400 6350 50  0000 C CNN
	1    14400 6350
	1    0    0    -1  
$EndComp
Text GLabel 15000 6350 2    60   Output ~ 0
NRST
Text GLabel 4350 3300 0    60   Input ~ 0
NRST
Text HLabel 4350 6550 0    60   Input ~ 0
EndStop1
Text HLabel 4350 6650 0    60   Input ~ 0
EndStop2
Text HLabel 4350 6750 0    60   Input ~ 0
Index
$Comp
L +3V3 #PWR034
U 1 1 585DAE20
P 4200 4600
F 0 "#PWR034" H 4200 4450 50  0001 C CNN
F 1 "+3V3" H 4200 4740 50  0000 C CNN
F 2 "" H 4200 4600 50  0000 C CNN
F 3 "" H 4200 4600 50  0000 C CNN
	1    4200 4600
	1    0    0    -1  
$EndComp
Text HLabel 4300 7150 0    60   Output ~ 0
MOSI
Text HLabel 4300 7050 0    60   Input ~ 0
MISO
Text HLabel 4300 6950 0    60   Output ~ 0
SCLK
$Comp
L C C4
U 1 1 585DBC77
P 2950 4900
F 0 "C4" H 2975 5000 50  0000 L CNN
F 1 "2.2uf" H 2975 4800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2988 4750 50  0001 C CNN
F 3 "" H 2950 4900 50  0000 C CNN
	1    2950 4900
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 585DBCE0
P 2750 4900
F 0 "C3" H 2775 5000 50  0000 L CNN
F 1 "2.2uf" H 2775 4800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2788 4750 50  0001 C CNN
F 3 "" H 2750 4900 50  0000 C CNN
	1    2750 4900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR035
U 1 1 585DBFC1
P 8800 8050
F 0 "#PWR035" H 8800 7800 50  0001 C CNN
F 1 "GND" H 8800 7900 50  0000 C CNN
F 2 "" H 8800 8050 50  0000 C CNN
F 3 "" H 8800 8050 50  0000 C CNN
	1    8800 8050
	1    0    0    -1  
$EndComp
$Comp
L STM32F205RBTx U2
U 1 1 585DCBF2
P 8700 5850
F 0 "U2" H 4700 7775 50  0000 L BNN
F 1 "STM32F205RBTx" H 12700 7775 50  0000 R BNN
F 2 "Housings_QFP:LQFP-64_10x10mm_Pitch0.5mm" H 12700 7725 50  0000 R TNN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/bc/21/42/43/b0/f3/4d/d3/CD00237391.pdf/files/CD00237391.pdf/jcr:content/translations/en.CD00237391.pdf" H 8700 5850 50  0001 C CNN
	1    8700 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	12800 4250 13450 4250
Wire Wire Line
	12800 4350 13450 4350
Wire Wire Line
	12800 4450 13450 4450
Wire Wire Line
	3800 5100 4350 5100
Wire Wire Line
	4350 5100 4350 5450
Wire Wire Line
	4350 5450 4600 5450
Wire Wire Line
	3800 5550 4600 5550
Wire Wire Line
	4000 5450 4000 5550
Connection ~ 4000 5550
Connection ~ 4000 5100
Wire Wire Line
	3350 5100 3350 5650
Wire Wire Line
	3350 5100 3500 5100
Wire Wire Line
	15300 5400 15300 5850
Wire Wire Line
	15300 5700 15050 5700
Wire Wire Line
	15300 5500 15050 5500
Connection ~ 15300 5700
Wire Wire Line
	15300 5400 15050 5400
Connection ~ 15300 5500
Wire Wire Line
	15250 5150 15250 5300
Wire Wire Line
	15250 5300 15050 5300
Wire Wire Line
	12800 5550 14000 5550
Wire Wire Line
	14000 5550 14000 5300
Wire Wire Line
	14000 5300 14550 5300
Wire Wire Line
	12800 5650 14100 5650
Wire Wire Line
	14100 5650 14100 5400
Wire Wire Line
	14100 5400 14550 5400
Wire Wire Line
	14200 5750 14200 5600
Wire Wire Line
	14200 5600 14550 5600
Wire Wire Line
	12800 6250 14300 6250
Wire Wire Line
	14300 6250 14300 5500
Wire Wire Line
	14300 5500 14550 5500
Wire Wire Line
	14550 5700 14550 6350
Wire Wire Line
	12800 6750 13100 6750
Wire Wire Line
	12800 6850 13100 6850
Wire Wire Line
	4350 5950 4600 5950
Wire Wire Line
	4350 6050 4600 6050
Wire Wire Line
	4350 6150 4600 6150
Wire Wire Line
	4350 6250 4600 6250
Wire Wire Line
	12800 4850 13450 4850
Wire Wire Line
	8800 2500 8800 3850
Wire Wire Line
	8800 2800 8650 2800
Wire Wire Line
	8650 2800 8650 2900
Wire Wire Line
	8650 3300 8650 3200
Wire Wire Line
	8250 3300 8250 3200
Wire Wire Line
	7900 3300 7900 3200
Wire Wire Line
	8700 3850 8700 3550
Wire Wire Line
	8700 3550 8450 3550
Wire Wire Line
	8450 3550 8450 2700
Wire Wire Line
	8450 2800 8250 2800
Wire Wire Line
	8250 2800 8250 2900
Wire Wire Line
	8600 3850 8600 3650
Wire Wire Line
	8600 3650 8050 3650
Wire Wire Line
	8050 3650 8050 2700
Wire Wire Line
	8050 2800 7900 2800
Wire Wire Line
	7900 2800 7900 2900
Wire Wire Line
	7600 3300 7600 3200
Wire Wire Line
	7600 2800 7600 2900
Wire Wire Line
	7600 2800 7750 2800
Wire Wire Line
	7750 2700 7750 3800
Wire Wire Line
	7750 3800 8500 3800
Wire Wire Line
	8500 3800 8500 3850
Wire Wire Line
	7200 3300 7200 3200
Wire Wire Line
	7200 2600 7200 2900
Wire Wire Line
	7200 2700 8800 2700
Connection ~ 7750 2800
Connection ~ 7200 2700
Connection ~ 8050 2800
Connection ~ 7750 2700
Connection ~ 8450 2800
Connection ~ 8050 2700
Connection ~ 8800 2800
Connection ~ 8450 2700
Wire Wire Line
	10150 3350 10150 3250
Wire Wire Line
	9850 3250 9850 3350
Wire Wire Line
	10150 2950 8900 2950
Wire Wire Line
	8900 2950 8900 3850
Connection ~ 9850 2950
Connection ~ 8800 2700
Wire Wire Line
	9850 2500 9850 2950
Wire Wire Line
	4500 4250 4600 4250
Wire Wire Line
	4500 3300 4500 4250
Wire Wire Line
	3900 3550 4800 3550
Wire Wire Line
	4000 3550 4000 3650
Wire Wire Line
	4000 4000 4000 3950
Connection ~ 4000 3550
Wire Wire Line
	3300 3750 3300 3550
Wire Wire Line
	3000 6350 4600 6350
Wire Wire Line
	3250 6450 4600 6450
Wire Wire Line
	12800 5750 14200 5750
Wire Wire Line
	12800 6350 14300 6350
Wire Wire Line
	14500 6350 14650 6350
Connection ~ 14550 6350
Wire Wire Line
	15000 6350 14850 6350
Wire Wire Line
	4350 3300 4500 3300
Connection ~ 4500 3550
Wire Wire Line
	9550 2500 9500 2500
Wire Wire Line
	8800 2500 8900 2500
Wire Wire Line
	4350 6750 4600 6750
Wire Wire Line
	4350 6650 4600 6650
Wire Wire Line
	4350 6550 4600 6550
Wire Wire Line
	4200 4600 4200 4650
Wire Wire Line
	4200 4650 4600 4650
Wire Wire Line
	4300 7350 4600 7350
Wire Wire Line
	4300 7450 4600 7450
Wire Wire Line
	4300 7150 4600 7150
Wire Wire Line
	4300 7050 4600 7050
Wire Wire Line
	4300 6950 4600 6950
Wire Wire Line
	8800 7850 8800 8050
Wire Wire Line
	2750 5050 2950 5050
Wire Wire Line
	2850 5050 2850 5200
Connection ~ 2850 5050
Wire Wire Line
	2750 4750 2750 4600
Wire Wire Line
	2950 4700 2950 4750
Wire Wire Line
	4400 4450 4600 4450
Wire Wire Line
	8600 7850 8600 7950
Wire Wire Line
	8600 7950 8800 7950
Connection ~ 8800 7950
Wire Wire Line
	8700 7850 8700 7950
Connection ~ 8700 7950
Wire Wire Line
	13100 6550 12800 6550
Wire Wire Line
	12800 6650 13100 6650
Wire Wire Line
	12800 4550 13450 4550
Wire Wire Line
	12800 4650 13450 4650
Wire Wire Line
	12800 4750 13450 4750
$Comp
L GND #PWR036
U 1 1 585DF79E
P 4300 4250
F 0 "#PWR036" H 4300 4000 50  0001 C CNN
F 1 "GND" H 4300 4100 50  0000 C CNN
F 2 "" H 4300 4250 50  0000 C CNN
F 3 "" H 4300 4250 50  0000 C CNN
	1    4300 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 4450 4400 4200
Wire Wire Line
	4400 4200 4300 4200
Wire Wire Line
	4300 4200 4300 4250
Wire Wire Line
	2950 4700 3700 4700
Wire Wire Line
	3700 4700 3700 4850
Wire Wire Line
	3700 4850 4600 4850
Wire Wire Line
	4600 4750 3800 4750
Wire Wire Line
	3800 4750 3800 4600
Wire Wire Line
	3800 4600 2750 4600
Text HLabel 13150 5350 2    60   BiDi ~ 0
USB_DM
Text HLabel 13150 5450 2    60   BiDi ~ 0
USB_DP
Wire Wire Line
	13150 5350 12800 5350
Wire Wire Line
	12800 5450 13150 5450
Text HLabel 13100 6950 2    60   BiDi ~ 0
I2C_SCL
Text HLabel 13100 7050 2    60   BiDi ~ 0
I2C_SDA
Wire Wire Line
	12800 6950 13100 6950
Wire Wire Line
	12800 7050 13100 7050
$Comp
L R R11
U 1 1 585E0727
P 3000 6650
F 0 "R11" V 3080 6650 50  0000 C CNN
F 1 "100R" V 3000 6650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2930 6650 50  0001 C CNN
F 3 "" H 3000 6650 50  0000 C CNN
	1    3000 6650
	1    0    0    -1  
$EndComp
$Comp
L LED-RESCUE-BMC2 D2
U 1 1 585E07CB
P 3000 7050
F 0 "D2" H 3000 7150 50  0000 C CNN
F 1 "GREEN LED" H 3000 6950 50  0000 C CNN
F 2 "LEDs:LED_0805" H 3000 7050 50  0001 C CNN
F 3 "" H 3000 7050 50  0000 C CNN
	1    3000 7050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3000 6800 3000 6850
$Comp
L R R12
U 1 1 585E0AA4
P 3250 6650
F 0 "R12" V 3330 6650 50  0000 C CNN
F 1 "100R" V 3250 6650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3180 6650 50  0001 C CNN
F 3 "" H 3250 6650 50  0000 C CNN
	1    3250 6650
	1    0    0    -1  
$EndComp
$Comp
L LED-RESCUE-BMC2 D3
U 1 1 585E0AF9
P 3250 7050
F 0 "D3" H 3250 7150 50  0000 C CNN
F 1 "RED LED" H 3250 6950 50  0000 C CNN
F 2 "LEDs:LED_0805" H 3250 7050 50  0001 C CNN
F 3 "" H 3250 7050 50  0000 C CNN
	1    3250 7050
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR037
U 1 1 585E0B52
P 3000 7300
F 0 "#PWR037" H 3000 7050 50  0001 C CNN
F 1 "GND" H 3000 7150 50  0000 C CNN
F 2 "" H 3000 7300 50  0000 C CNN
F 3 "" H 3000 7300 50  0000 C CNN
	1    3000 7300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR038
U 1 1 585E0CBA
P 3250 7300
F 0 "#PWR038" H 3250 7050 50  0001 C CNN
F 1 "GND" H 3250 7150 50  0000 C CNN
F 2 "" H 3250 7300 50  0000 C CNN
F 3 "" H 3250 7300 50  0000 C CNN
	1    3250 7300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 7300 3250 7250
Wire Wire Line
	3000 7250 3000 7300
Wire Wire Line
	3250 6850 3250 6800
Wire Wire Line
	3250 6500 3250 6450
Wire Wire Line
	3000 6350 3000 6500
Text HLabel 4350 6250 0    60   Input ~ 0
Temp
Text HLabel 4300 7250 0    60   Output ~ 0
Wake
Wire Wire Line
	4300 7250 4600 7250
Text HLabel 4300 6850 0    60   Output ~ 0
sSCS
Wire Wire Line
	4300 6850 4600 6850
Text HLabel 4300 5750 0    60   Input ~ 0
PWRGD
Wire Wire Line
	4300 5750 4600 5750
Wire Wire Line
	10150 2700 10150 2950
Text HLabel 13100 6450 2    60   Output ~ 0
CAN_STBY
Wire Wire Line
	13100 6450 12800 6450
Text HLabel 4800 3550 2    60   Input ~ 0
NRST
$Comp
L R R16
U 1 1 586D99EA
P 14750 6850
F 0 "R16" V 14830 6850 50  0000 C CNN
F 1 "10K" V 14750 6850 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 14680 6850 50  0001 C CNN
F 3 "" H 14750 6850 50  0000 C CNN
	1    14750 6850
	1    0    0    -1  
$EndComp
$Comp
L R R17
U 1 1 586D9ACD
P 14750 7250
F 0 "R17" V 14830 7250 50  0000 C CNN
F 1 "10K" V 14750 7250 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 14680 7250 50  0001 C CNN
F 3 "" H 14750 7250 50  0000 C CNN
	1    14750 7250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR039
U 1 1 586D9B9E
P 14750 7500
F 0 "#PWR039" H 14750 7250 50  0001 C CNN
F 1 "GND" H 14750 7350 50  0000 C CNN
F 2 "" H 14750 7500 50  0000 C CNN
F 3 "" H 14750 7500 50  0000 C CNN
	1    14750 7500
	-1   0    0    -1  
$EndComp
$Comp
L +5V #PWR040
U 1 1 586D9C47
P 14750 6650
F 0 "#PWR040" H 14750 6500 50  0001 C CNN
F 1 "+5V" H 14750 6790 50  0000 C CNN
F 2 "" H 14750 6650 50  0000 C CNN
F 3 "" H 14750 6650 50  0000 C CNN
	1    14750 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	14750 7400 14750 7500
Wire Wire Line
	14750 7000 14750 7100
Wire Wire Line
	14750 6700 14750 6650
Wire Wire Line
	14200 7050 15050 7050
Wire Wire Line
	14200 7050 14200 5950
Wire Wire Line
	14200 5950 12800 5950
Connection ~ 14750 7050
$Comp
L C C16
U 1 1 586DA2C8
P 15050 7250
F 0 "C16" H 15075 7350 50  0000 L CNN
F 1 "0.1uF 5V" H 15075 7150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 15088 7100 50  0001 C CNN
F 3 "" H 15050 7250 50  0000 C CNN
	1    15050 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	15050 7400 15050 7450
Wire Wire Line
	15050 7450 14750 7450
Connection ~ 14750 7450
Wire Wire Line
	15050 7050 15050 7100
$Comp
L R R15
U 1 1 586FC3AC
P 14350 6650
F 0 "R15" V 14430 6650 50  0000 C CNN
F 1 "10K" V 14350 6650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 14280 6650 50  0001 C CNN
F 3 "" H 14350 6650 50  0000 C CNN
	1    14350 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	14350 6500 14250 6500
Wire Wire Line
	14250 6500 14250 6350
Connection ~ 14250 6350
$Comp
L +3V3 #PWR041
U 1 1 586FC541
P 14500 6950
F 0 "#PWR041" H 14500 6800 50  0001 C CNN
F 1 "+3V3" H 14500 7090 50  0000 C CNN
F 2 "" H 14500 6950 50  0000 C CNN
F 3 "" H 14500 6950 50  0000 C CNN
	1    14500 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	14500 6950 14350 6950
Wire Wire Line
	14350 6950 14350 6800
$Comp
L SW_PUSH SW2
U 1 1 586FD8D0
P 14100 8050
F 0 "SW2" H 14250 8160 50  0000 C CNN
F 1 "CAL" H 14100 7970 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_FSMSM" H 14100 8050 50  0001 C CNN
F 3 "http://www.farnell.com/cad/1885096.pdf?_ga=1.237363456.1808009743.1471515981" H 14100 8050 50  0001 C CNN
F 4 "http://uk.farnell.com/alcoswitch-te-connectivity/1437566-3/pushbutton-switch-spst-0-05a-24v/dp/2468741" H 14100 8050 60  0001 C CNN "Order"
	1    14100 8050
	0    1    1    0   
$EndComp
$Comp
L R R14
U 1 1 586FE4BF
P 14100 7500
F 0 "R14" V 14180 7500 50  0000 C CNN
F 1 "10K" V 14100 7500 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 14030 7500 50  0001 C CNN
F 3 "" H 14100 7500 50  0000 C CNN
	1    14100 7500
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR042
U 1 1 586FE559
P 14100 7250
F 0 "#PWR042" H 14100 7100 50  0001 C CNN
F 1 "+3V3" H 14100 7390 50  0000 C CNN
F 2 "" H 14100 7250 50  0000 C CNN
F 3 "" H 14100 7250 50  0000 C CNN
	1    14100 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	14100 7250 14100 7350
Wire Wire Line
	14100 7650 14100 7750
$Comp
L GND #PWR043
U 1 1 586FE773
P 14100 8450
F 0 "#PWR043" H 14100 8200 50  0001 C CNN
F 1 "GND" H 14100 8300 50  0000 C CNN
F 2 "" H 14100 8450 50  0000 C CNN
F 3 "" H 14100 8450 50  0000 C CNN
	1    14100 8450
	-1   0    0    -1  
$EndComp
Wire Wire Line
	14100 8350 14100 8450
$Comp
L C C15
U 1 1 586FE929
P 14600 8100
F 0 "C15" H 14625 8200 50  0000 L CNN
F 1 "0.1uF 5V" H 14625 8000 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 14638 7950 50  0001 C CNN
F 3 "" H 14600 8100 50  0000 C CNN
	1    14600 8100
	1    0    0    -1  
$EndComp
Wire Wire Line
	14100 8400 14600 8400
Wire Wire Line
	14600 8400 14600 8250
Connection ~ 14100 8400
Wire Wire Line
	14600 7700 14600 7950
Wire Wire Line
	13800 7700 14600 7700
Connection ~ 14100 7700
Wire Wire Line
	13800 6150 13800 7700
Text HLabel 13500 6050 2    60   Input ~ 0
TempDrv
Wire Wire Line
	13500 6050 12800 6050
Wire Wire Line
	13800 6150 12800 6150
$Comp
L TEST_1P W14
U 1 1 58716B1C
P 13650 7150
F 0 "W14" H 13650 7420 50  0000 C CNN
F 1 "PB12" H 13650 7350 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 13850 7150 50  0001 C CNN
F 3 "" H 13850 7150 50  0000 C CNN
	1    13650 7150
	1    0    0    -1  
$EndComp
Wire Wire Line
	12800 7150 13650 7150
$Comp
L +3.3VADC #PWR044
U 1 1 585F9182
P 10150 2700
F 0 "#PWR044" H 10300 2650 50  0001 C CNN
F 1 "+3.3VADC" H 10150 2800 50  0000 C CNN
F 2 "" H 10150 2700 50  0000 C CNN
F 3 "" H 10150 2700 50  0000 C CNN
	1    10150 2700
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG045
U 1 1 587638B7
P 10500 2800
F 0 "#FLG045" H 10500 2895 50  0001 C CNN
F 1 "PWR_FLAG" H 10500 2980 50  0000 C CNN
F 2 "" H 10500 2800 50  0000 C CNN
F 3 "" H 10500 2800 50  0000 C CNN
	1    10500 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	10500 2800 10150 2800
Connection ~ 10150 2800
Wire Wire Line
	3500 5550 3350 5550
Connection ~ 3350 5550
NoConn ~ 12800 4950
$Comp
L PWR_FLAG #FLG046
U 1 1 587659B8
P 3300 4500
F 0 "#FLG046" H 3300 4595 50  0001 C CNN
F 1 "PWR_FLAG" H 3300 4680 50  0000 C CNN
F 2 "" H 3300 4500 50  0000 C CNN
F 3 "" H 3300 4500 50  0000 C CNN
	1    3300 4500
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG047
U 1 1 58765F43
P 3600 4550
F 0 "#FLG047" H 3600 4645 50  0001 C CNN
F 1 "PWR_FLAG" H 3600 4730 50  0000 C CNN
F 2 "" H 3600 4550 50  0000 C CNN
F 3 "" H 3600 4550 50  0000 C CNN
	1    3600 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 4550 3600 4700
Connection ~ 3600 4700
Wire Wire Line
	3300 4500 3300 4600
Connection ~ 3300 4600
$Comp
L Crystal X1
U 1 1 587A548C
P 4000 5300
F 0 "X1" H 4000 5450 50  0000 C CNN
F 1 "8MHz" H 4000 5150 50  0000 C CNN
F 2 "Crystals:ABM3" H 4000 5300 50  0001 C CNN
F 3 "http://www.farnell.com/datasheets/1754353.pdf?_ga=1.70757648.1808009743.1471515981" H 4000 5300 50  0001 C CNN
F 4 "http://uk.farnell.com/abracon/abm3-8-000mhz-d2y-t/crystal-8mhz-18pf-smd/dp/2101329" H 4000 5300 60  0001 C CNN "Order"
	1    4000 5300
	0    1    1    0   
$EndComp
Wire Wire Line
	4000 5150 4000 5100
Wire Wire Line
	12800 5050 13250 5050
Wire Wire Line
	12800 5150 13250 5150
Wire Wire Line
	12800 5250 13250 5250
Wire Wire Line
	13100 7250 12800 7250
Wire Wire Line
	12800 7350 13100 7350
Wire Wire Line
	12800 7450 13100 7450
$EndSCHEMATC
