EESchema Schematic File Version 2  date 2013-01-30 23:12:01
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
LIBS:special
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
EELAYER 43  0
EELAYER END
$Descr User 11000 8500
encoding utf-8
Sheet 2 2
Title "Étage d'alimentation pour les 3 différents Voltages"
Date "31 jan 2013"
Rev "1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Connection ~ 6750 2350
Wire Wire Line
	5650 2350 7850 2350
Connection ~ 6250 2350
Wire Wire Line
	6750 2350 6750 2700
Connection ~ 5850 2900
Wire Wire Line
	6250 2850 6250 2900
Wire Wire Line
	6250 2900 4900 2900
Wire Wire Line
	5250 2900 5250 2850
Wire Wire Line
	4450 3400 4450 2750
Connection ~ 3050 3550
Wire Wire Line
	3300 3550 2800 3550
Connection ~ 6850 5950
Wire Wire Line
	7300 5950 2950 5950
Wire Wire Line
	7300 5950 7300 4950
Connection ~ 4500 4550
Wire Wire Line
	5350 4550 3050 4550
Connection ~ 6000 4550
Wire Wire Line
	6250 4550 5950 4550
Wire Wire Line
	6850 5450 6850 5050
Connection ~ 6500 5950
Wire Wire Line
	6500 5850 6500 5950
Connection ~ 5350 5950
Connection ~ 4500 5950
Wire Wire Line
	2950 3800 2800 3800
Wire Wire Line
	4500 5950 4500 4950
Wire Wire Line
	6000 4550 6000 5050
Wire Wire Line
	6000 5250 6850 5250
Connection ~ 6850 5250
Connection ~ 6850 4550
Wire Wire Line
	7850 4550 6650 4550
Connection ~ 7300 4550
Wire Wire Line
	7850 3550 3700 3550
Wire Wire Line
	2950 5950 2950 3400
Connection ~ 2950 3800
Wire Wire Line
	3050 4550 3050 2350
Wire Wire Line
	3050 2350 4850 2350
Wire Wire Line
	4900 2900 4900 3000
Wire Wire Line
	5850 2900 5850 2850
Connection ~ 5250 2900
Wire Wire Line
	6250 2350 6250 2450
Wire Wire Line
	2950 3400 6750 3400
Wire Wire Line
	6750 3400 6750 3100
Connection ~ 4450 3400
$Comp
L CP1 C6
U 1 1 5109EA89
P 6750 2900
F 0 "C6" H 6800 3000 50  0000 L CNN
F 1 "1u" H 6800 2800 50  0000 L CNN
	1    6750 2900
	1    0    0    -1  
$EndComp
$Comp
L DIODE D3
U 1 1 5109EA7E
P 6250 2650
F 0 "D3" H 6250 2750 40  0000 C CNN
F 1 "5A min" H 6250 2550 40  0000 C CNN
	1    6250 2650
	0    -1   -1   0   
$EndComp
$Comp
L R R5
U 1 1 5109EA76
P 5850 2600
F 0 "R5" V 5930 2600 50  0000 C CNN
F 1 "23K" V 5850 2600 50  0000 C CNN
	1    5850 2600
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 5109EA70
P 5250 3150
F 0 "R4" V 5330 3150 50  0000 C CNN
F 1 "1.24K" V 5250 3150 50  0000 C CNN
	1    5250 3150
	1    0    0    -1  
$EndComp
$Comp
L CP1 C5
U 1 1 5109EA67
P 4900 3200
F 0 "C5" H 4950 3300 50  0000 L CNN
F 1 "10u" H 4950 3100 50  0000 L CNN
	1    4900 3200
	1    0    0    -1  
$EndComp
$Comp
L LM317 U2
U 1 1 5109E8FC
P 5250 2500
F 0 "U2" H 5250 2800 60  0000 C CNN
F 1 "LM317" H 5300 2250 60  0000 L CNN
	1    5250 2500
	1    0    0    -1  
$EndComp
$Comp
L CP1 C4
U 1 1 5109E81B
P 4450 2550
F 0 "C4" H 4500 2650 50  0000 L CNN
F 1 "0.1u" H 4500 2450 50  0000 L CNN
	1    4450 2550
	1    0    0    -1  
$EndComp
$Comp
L DIODE D2
U 1 1 5109E7D5
P 3500 3550
F 0 "D2" H 3500 3650 40  0000 C CNN
F 1 "5A min" H 3500 3450 40  0000 C CNN
	1    3500 3550
	1    0    0    -1  
$EndComp
$Comp
L CP1 C3
U 1 1 5109E45A
P 7300 4750
F 0 "C3" H 7350 4850 50  0000 L CNN
F 1 "1000u" H 7350 4650 50  0000 L CNN
	1    7300 4750
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5109E3D8
P 6500 5650
F 0 "C2" H 6550 5750 50  0000 L CNN
F 1 "1u" H 6550 5550 50  0000 L CNN
	1    6500 5650
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5109E3AF
P 6250 5450
F 0 "R1" V 6330 5450 50  0000 C CNN
F 1 "1K" V 6250 5450 50  0000 C CNN
	1    6250 5450
	0    -1   -1   0   
$EndComp
$Comp
L R R2
U 1 1 5109E373
P 6850 5700
F 0 "R2" V 6930 5700 50  0000 C CNN
F 1 "1.24K" V 6850 5700 50  0000 C CNN
	1    6850 5700
	1    0    0    -1  
$EndComp
$Comp
L DIODESCH D1
U 1 1 5109E362
P 6450 4550
F 0 "D1" H 6450 4650 40  0000 C CNN
F 1 "5A min" H 6450 4450 40  0000 C CNN
	1    6450 4550
	1    0    0    -1  
$EndComp
$Comp
L CP1 C1
U 1 1 5109E312
P 4500 4750
F 0 "C1" H 4550 4850 50  0000 L CNN
F 1 "100u" H 4550 4650 50  0000 L CNN
	1    4500 4750
	1    0    0    -1  
$EndComp
Text HLabel 7850 4550 2    60   Output ~ 0
Vout24V
Text HLabel 7850 3550 2    60   Output ~ 0
Vout11.1V
Text HLabel 7850 2350 2    60   Output ~ 0
Vout5V
Text HLabel 2800 3800 0    60   Input ~ 0
GND
Text HLabel 2800 3550 0    60   Input ~ 0
Vin
$Comp
L LT1170 U1
U 1 1 5109E06A
P 5350 5250
F 0 "U1" H 5350 5350 60  0000 C CNN
F 1 "LT1170" H 5350 5150 60  0000 C CNN
	1    5350 5250
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5109E064
P 6850 4800
F 0 "R3" V 6930 4800 50  0000 C CNN
F 1 "23K" V 6850 4800 50  0000 C CNN
	1    6850 4800
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR L1
U 1 1 5109E062
P 5650 4550
F 0 "L1" V 5600 4550 40  0000 C CNN
F 1 "50u" V 5750 4550 40  0000 C CNN
	1    5650 4550
	0    -1   -1   0   
$EndComp
$EndSCHEMATC
