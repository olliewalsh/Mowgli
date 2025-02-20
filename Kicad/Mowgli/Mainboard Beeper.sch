EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 14 14
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
L Device:Speaker LS1
U 1 1 62C3C8BA
P 5150 2500
F 0 "LS1" H 5320 2496 50  0000 L CNN
F 1 "Onboard Beeper" H 5320 2405 50  0000 L CNN
F 2 "" H 5150 2300 50  0001 C CNN
F 3 "~" H 5140 2450 50  0001 C CNN
	1    5150 2500
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_SGD Q1
U 1 1 62C438B2
P 4700 2950
F 0 "Q1" H 4904 2996 50  0000 L CNN
F 1 "Q_NMOS_SGD" H 4904 2905 50  0000 L CNN
F 2 "" H 4900 3050 50  0001 C CNN
F 3 "~" H 4700 2950 50  0001 C CNN
	1    4700 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 2600 4800 2600
Wire Wire Line
	4800 2600 4800 2750
$Comp
L power:+5V #PWR?
U 1 1 62C477A0
P 4800 2350
F 0 "#PWR?" H 4800 2200 50  0001 C CNN
F 1 "+5V" H 4815 2523 50  0000 C CNN
F 2 "" H 4800 2350 50  0001 C CNN
F 3 "" H 4800 2350 50  0001 C CNN
	1    4800 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 2500 4800 2500
Wire Wire Line
	4800 2500 4800 2350
$Comp
L Device:R R2
U 1 1 62C48139
P 4250 2950
F 0 "R2" V 4043 2950 50  0000 C CNN
F 1 "4.7k" V 4134 2950 50  0000 C CNN
F 2 "" V 4180 2950 50  0001 C CNN
F 3 "~" H 4250 2950 50  0001 C CNN
	1    4250 2950
	0    1    1    0   
$EndComp
Wire Wire Line
	4500 2950 4400 2950
$Comp
L Device:R R1
U 1 1 62C49EDF
P 3800 2950
F 0 "R1" V 3593 2950 50  0000 C CNN
F 1 "4.7k" V 3684 2950 50  0000 C CNN
F 2 "" V 3730 2950 50  0001 C CNN
F 3 "~" H 3800 2950 50  0001 C CNN
	1    3800 2950
	0    1    1    0   
$EndComp
Wire Wire Line
	4100 2950 4000 2950
$Comp
L power:GND #PWR?
U 1 1 62C4A7A3
P 4800 3250
F 0 "#PWR?" H 4800 3000 50  0001 C CNN
F 1 "GND" H 4805 3077 50  0000 C CNN
F 2 "" H 4800 3250 50  0001 C CNN
F 3 "" H 4800 3250 50  0001 C CNN
	1    4800 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 3150 4800 3250
$Comp
L power:GND #PWR?
U 1 1 62C4B10B
P 3600 3300
F 0 "#PWR?" H 3600 3050 50  0001 C CNN
F 1 "GND" H 3605 3127 50  0000 C CNN
F 2 "" H 3600 3300 50  0001 C CNN
F 3 "" H 3600 3300 50  0001 C CNN
	1    3600 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 2950 3600 2950
Wire Wire Line
	3600 2950 3600 3300
Text HLabel 3800 2250 0    50   Input ~ 0
BEEPER
Wire Wire Line
	4000 2950 4000 2250
Wire Wire Line
	4000 2250 3800 2250
Connection ~ 4000 2950
Wire Wire Line
	4000 2950 3950 2950
$EndSCHEMATC
