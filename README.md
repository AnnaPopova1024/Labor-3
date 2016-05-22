# Labor-3
  
#include "msp430f5529.h"

;hier ein paar Definitonen, so dass das Programm unten besser lesbar wird

T_OnOff         SET     0x02                     ; Taster an Port P1.1 
T_SET           SET     0x02                     ; Taster an Port P2.1 (der Name S2 stammt aus dem Schaltplan des LaunchPad, siehe LaunchPad-Userguide "slau533b" auf Seite 57)

LED_ON          SET     0x00                     ; rote LED ist an Port 1.0 angeschlossen (0.Bit ==> 0x00)
LED_x           SET     0x80                     ; grüne LED ist an Port 4.7 angeschlossen (7.Bit ==> 0x80)              

//______________________________________________________________________________
main            ORG     04400h                   ; Progam Start, Adresse aus Tabelle im Datenblatt Seite 22, rechte Spalte (für MSP430F5529)
//______________________________________________________________________________
RESET           mov.w   #04400h,SP               ; Stackpointer initialisieren, der Stack wächst von oben nach unten !

StopWDT         mov.w   #WDTPW+WDTHOLD,&WDTCTL   ; Watchdog Timer anhalten

; +++ Konfiguration der IO-Ports und der Port-Interrupts +++

Port_1
            mov.b   #0x00, &P1IFG           ; Alle P1-IFG's löschen, falls zufällig gesetzt         
            mov.b   #BIT0, &P1DIR           ; nur P1.0 für rote LED als Ausgang konfigurieren (1=Out, 0=In)
            mov.b   #BIT1, &P1OUT           ; LED an Port 1.0 ist aus (P1.0=0) 
            mov.b   #0x00, &P1SEL           ; kompletter Port P1 als normaler IO-Port verfügbar, nichts wird an andere Peripherie abgegeben
            mov.b   #BIT1, &P1REN           ; aktiviere PullUp an P1.1 für T_OnOff
            mov.b   #0xff, &P1IES           ; alle Port 1 Interrupts werden auf negative Flanke getriggert (das ist so, weil die Taster auf dem LaunchPad nach Masse gehen)
            mov.b   #T_OnOff, &P1IE         ; Nur Taster "T_OnOff" für Interrupt freigeben, alle anderen Interruptflags von Port 1 unterdrücken

Port_2
            bic.b   #T_SET, &P2IFG          ; Alle P2-IFG's löschen, falls zufällig gesetzt         
            bic.b   #BIT1, &P2DIR           ; T_SET = 0 -> INPUT
            bic.b   #T_SET, &P2SEL           ; kompletter Port P2 als normaler IO-Port verfügbar, nichts wird an andere Peripherie abgegeben
            bis.b   #BIT1, &P2OUT
            mov.b   #BIT1, &P2REN           ; aktiviere PullUp an P2.1 für T_SET
            mov.b   #0xff, &P2IES           ; alle Port 2 Interrupts werden auf negative Flanke getriggert (das ist so, weil die Taster auf dem LaunchPad nach Masse gehen)
            mov.b   #T_SET, &P2IE           ; Nur Taster "T_SET" für Interrupt freigeben, alle anderen Interruptflags von Port 1 unterdrücken
    
Port_4          
                ;mov.b   #0x00, &P4IFG            ; Alle P1-IFG's löschen, falls zufällig gesetzt 
            bis.b   #BIT7, &P4DIR            ; Port P4.7 als Ausgang konfigurieren für grüne LED 
            bis.b   #BIT4, &P4DIR            ; Port P4.4 als Ausgang konfigurieren für UART TxD (zum Senden serieller Daten über diesen Pin)  
            bic.b   #BIT7, &P4OUT            ; LED an Port 4.7 ausschalten zu Programmbeginn
            bis.b   #BIT4, &P4OUT            ; 1 = High = UART Idle State
            bis.b   #BIT4, &P4SEL            ; P4.4 nicht als normalen Port verwenden sondern an USCI-Modul (=UART) abgeben zum Senden (TxD)   
            bis.b   #BIT5, &P4SEL            ; P4.5 nicht als normalen Port verwenden sondern an USCI-Modul (=UART) abgeben zum Empfangen von Daten (RxD)        

Port_6
//            mov.b   #0x00, &P6IFG            ; Alle P6-IFG's löschen, falls zufällig gesetzt         
            bis.b   #BIT2, &P6DIR            ; Port P6.2 als Ausgang konfigurieren für LED1 auf Erweiterungs-Modul
//            mov.b   #BIT2, &P6DIR            ; nur P6.2 für LED1 als Ausgang konfigurieren (1=Out, 0=In)
            bic.b   #BIT2, &P6OUT            ; LED an Port 6.2 ist aus (P6.2=0) 
            bis.b   #BIT3, &P6DIR            ; Port P6.3 als Ausgang konfigurieren für LED2 auf Erweiterungs-Modul
//            mov.b   #BIT3, &P6DIR            ; nur P6.3 für LE2 als Ausgang konfigurieren (1=Out, 0=In)
            bic.b   #BIT3, &P6OUT            ; LED an Port 6.3 ist aus (P6.3=0) 
            bis.b   #BIT4, &P6DIR            ; Port P6.4 als Ausgang konfigurieren für LED3 auf Erweiterungs-Modul
//            mov.b   #BIT4, &P6DIR            ; nur P6.4 für LED3 als Ausgang konfigurieren (1=Out, 0=In)
            bic.b   #BIT4, &P6OUT            ; LED an Port 6.4 ist aus (P6.4=0) 
            mov.b   #0x00, &P6SEL            ; kompletter Port P1 als normaler IO-Port verfügbar, nichts wird an andere Peripherie abgegeben
            
//            bic.b   #BIT1+BIT2, &P6DIR       ; Port P6.0 und P6.1 als Ausgang konfigurieren für Poti_R und Poti_L auf Erweiterungs-Modul
//            bis.b   #BIT1+BIT2, &P6SEL       ; Port P6.0 und P6.1 als Peripherie auf Erweiterungs-Modul
            
            
UART_A1_config  
            bis.b   #UCSWRST,&UCA1CTL1       ; Die UART in den Reset-Mode bringen um sie nachfolgend Konfigurieren zu können, siehe UserGuide Seite 894
            
            mov.b   #0x00, &UCA1CTL0         ; Betriebsart des UART: Asynchron, 8 Datenbits, 1 Stopbit, kein Paritybit, LSB zuerst senden/empfangen 
            bis.b   #UCSSEL1,&UCA1CTL1       ; SMCLK als Taktquelle auswählen
            mov.b   #109, &UCA1BR0           ; Lowbyte, Baudrate einstellen entsprechend Tabelle aus dem UserGuide    
            mov.b   #0, &UCA1BR1             ; Highbyte, Baudrate einstellen entsprechend Tabelle aus dem UserGuide    
            bis.b   #UCBRS1,&UCA1MCTL        ; Modulator einstellen entsprechend Tabelle aus dem UserGuide
            mov.b   #0x00, &UCA1STAT         ; alle möglichen Flags löschen
            mov.b   #0x00, &UCA1ABCTL        ; keine Auto-Baudrate-Detektion
            
            bic.b   #UCSWRST,&UCA1CTL1       ; Die UART in den normalen Betrieb versetzen nachdem zuvor alles Konfiguriert wurde, siehe UserGuide Seite 894
            
            bis.b   #UCRXIE, &UCA1IE         ; Interrupt für die UART aktivieren: wenn ein Byte empfangen wurde (RxD-Interrupt), diese Zeile MUSS zwingend nach dem Reset des UART Moduls erfolgen, da sonst die aktivierten Interrupts wieder deaktiviert werden (siehe Example auf Seite 894 im UserGuide)
//______________________________________________________________________________                          
ADC_INIT
            bic.w       #ADC12ENC, &ADC12CTL0   ; Das Flag "ADC12ENC" sicher löschen --> erst danach darf der Rest des Registers konfiguriert werden! Siehe UserGuide Seite 742!

            bis.b       #BIT0+BIT1, &P6SEL      ; Bit0 und Bit1 specielle Fkt-> Port P6.0 und P6.1 als Ausgang konfigurieren für Poti_R und Poti_L auf Erweiterungs-Modul
            bic.b       #BIT0+BIT1, &P6DIR      ; Port P6.0 und P6.1 als Peripherie auf Erweiterungs-Modul
//            
            bic.w       #REFMSTR, &REFCTL0      ; für die eigene interne Spannung
            mov.b       #ADC12INCH0+ADC12SREF_1, &ADC12MCTL0    ; P6.0
            mov.b       #ADC12INCH1+ADC12SREF_0, &ADC12MCTL1    ; P6.1
            mov.b       #ADC12INCH0+ADC12SREF_1, &ADC12MCTL2    ; P6.0
            mov.b       #ADC12INCH1+ADC12SREF_0+ADC12EOS, &ADC12MCTL3    ; P6.1
            mov.w       #ADC12ON+ADC12REFON+ADC12REF2_5V+ADC12MSC+ADC12SHT0_3, &ADC12CTL0
            mov.w       #ADC12CSTARTADD_0+ADC12SHP+ADC12SHS_0+ADC12SSEL_2+ADC12CONSEQ_1, &ADC12CTL1           ; ACLK als Taktquelle wählen, Single-Conversion-Mode
            mov.w       #ADC12RES_2, &ADC12CTL2 ; auf 12 Bit Auflösung setzen (falls vergessen, dann auf 8)
            mov.w       BIT3, &ADC12IE          ; wenn EOS kommt, dann IE setzen
            bis.w       #ADC12ENC, &ADC12CTL0   ; nach der Konfiguration der Kontroll-Register das "Enable Conversion"-Bit wieder setzen. Einstellungen übernehmen

//               
//                bic.w   #REFMSTR, &REFCTL0      ; alte separate interne Referenz abschalten --> jetzt erst kann man die neue interne Referenz des ADC12 benutzen mit den Kontroll-Bits im Register ADC12CTL0
//                
//                mov.w   #ADC12SHT1_7+ADC12SHT0_7+ADC12REFON+ADC12ON, &ADC12CTL0      ; Mittlere Sample-Zeit einstellen, interene Referenz auf 1.5V einstellen und aktvieren, ADC12 einschalten
//                mov.b   #ADC12EOS+ADC12SREF_1+ADC12INCH_10, &ADC12MCTL0              ; interne Referenz wählen, internen Temperatursensor als Input signal wählen
//                mov.w   #0x0000, &ADC12IE                                            ; keine Interrupts für den ADC12 aktivieren
//                mov.w   #0x0000, &ADC12IFG                                           ; vorsichtshalber alle Interruptflags löschen (falls zufällig gesetzt) 
//               
//______________________________________________________________________________
Register_Init  
            mov.w       #0x00, R5                       ; Merker für T_OnOff
            mov.w       #0x00, R6                       ; Merker für T_SET
            mov.w       #0x00, R7                       ; Merker für Poti
            mov.w       #0x00, R12                      ; R12 wird für String benutzt
//______________________________________________________________________________
Mainloop        
            nop
            bis.w       #GIE, SR                        ; global Interruptsystem aktivieren
            nop
            jmp         Mainloop                        ; Endlosschleife
            nop                                         ; sinnloser letzter Befehl, damit der IAR Simulator nicht meckert...
//______________________________________________________________________________
T_OnOff_ISR
            mov.w       #50000, R15 
Entprellen1     
            dec.w       R15                             ; Decrement R15
            jnz         Entprellen1                     ; Springe zu Entprellen wenn R15 noch nicht 0 ist

            inc.w       R5                              ; Merker für Zustand
            cmp.w       #1, R5
            jz          Zustand_1
            cmp.w       #2, R5
            jz          Zustand_2
            
Zustand_1                                               ; Programm START
            bis.b       #BIT0, &P1OUT                   ; LED_x einschalten
            bis.b       #BIT7, &P4OUT                   ; LED_ON einschalten
            
            bit.w       #MC1, &TA0CTL                   ; Teste ob TimerA aktiv ist?
            jnz         Timer_aktiv
            jmp         Timer_n_aktiv
            
Timer_n_aktiv            
            call        #TA0                            ; Timer aktivieren

Timer_aktiv
            mov.w       #40960, &TA0CCR0                ; 5 sec = 5/8 * 65536 = 40960 in TA0CCRO schreiben             
            mov.w       #txt_Start, R12                 ; kopiere String mit Matr.Nummer in R12
            call        #SubR_Str_TX                    ; die SubRoutine aufrufen um Matr.Nummer zu senden
            
            jmp         Ende1
            
Zustand_2                                               ; Programm AUS
            bic.b       #BIT0, &P1OUT                   ; LED_x ausschalten
            bic.b       #BIT7, &P4OUT                   ; LED_ON ausschalten

            mov.w       #txt_Ende, R12                  ; kopiere String mit "System Off" Meldung in R12
            call        #SubR_Str_TX                    ; die SubRoutine aufrufen um "System Off" Meldung zu senden
            mov.w       #TACLR, &TA0CTL                 ; TimerA ausschalten
            mov.w       #0, R5                          ; Setze Merker zurück auf 0 
Ende1
            bic.b       #T_OnOff, &P1IFG                ; das gesetzte Interruptflag löschen, sonst würde ISR sofort wieder neu ausgelöst werden
            reti
//______________________________________________________________________________
T_SET_ISR
            mov.w       #50000,R15  
Entprellen2 dec.w       R15                             ; Decrement R15
            jnz         Entprellen2                     ; Springe zu Entprellen wenn R15 noch nicht 0 ist

            mov.w       #TACLR, &TA0CTL                 ; TimerA ausschalten
            call        #TA0                            ; Timer aktivieren
            bit.b       #0xff, &P2IES                   ; Ist Interrupt von negativer Flanke?
            jnz         Zustand_5                       ; Wenn ja, dann setze Taster Interrupt auf pos. Flanke
            jmp         Zustand_6
            

Zustand_5
            mov.b       #0x00, &P2IES                   ; Setze Taster Interrupt auf pos. Flanke
//            mov.w       #16384, &TA0CCR1                ; 2 sec = 2/8 * 65536 = 16384 in TA0CCRO schreiben 
//            mov.w       #OUTMOD_3, &TA0CCTL1            ; Set/Reset (S.468 User's Guide)
//            mov.w       #CCIE, &TA0CCTL1                ; Controlle
            bic.b       #T_SET, &P2IFG                  ; Lösche Taster Ereignis
            reti
Zustand_6         
            mov.w       #TACLR, &TA0CTL                 ; TimerA ausschalten
            mov.b       #0xff, &P2IES                   ; Setze Taster Interrupt auf neg. Flanke
            mov.w       #1, R7                          ; Setze Merker auf 1 -> Poti_R als Quelle
            call        #SubR_Poti_Wahl                 ; die SubRoutine aufrufen um Poti auszuwählen 
            bic.b       #T_SET, &P2IFG                  ; Lösche Taster Ereignis
            reti
//______________________________________________________________________________
UART_RxD_ISR 
            mov.b       &UCA1RXBUF, R12
            reti
//______________________________________________________________________________
TIMER_A0_ISR
            bit.b       #BIT7, &P4OUT                   ; Test ob LED_x eingeschaltet ist
            jz          TA0_TSet
            jmp         TA0_LED

TA0_LED
            bit.b       #BIT0, &P1OUT                   ; Test ob LED_ON eingeschaltet ist
            jnz         Zustand_3
            bit.b       #BIT7, &P4OUT                   ; Test ob LED_x eingeschaltet ist
            jnz         Zustand_4
            
            mov.w       #txt_Start, R12                 ; kopiere String mit Matrikelnummer in R12
TA0_TSet            
            mov.b       #0xff, &P2IES                   ; Setze Taster Interrupt auf neg. Flanke zurück
            mov.w       #TACLR, &TA0CTL                 ; TimerA ausschalten
            mov.w       #2, R7                          ; Setze Merker auf 2 -> Poti_L als Quelle
            call        #SubR_Poti_Wahl                 ; die SubRoutine aufrufen um Poti auszuwählen    
//            bis.w       #ADC12SC, &ADC12CTL0
            bic.b       #T_SET, &P2IFG                  ; Lösche Taster Ereignis
            reti

Zustand_3       
            bic.b       #BIT7, &P4OUT                   ; LED_x ausschalten
            mov.w       #TACLR, &TA0CTL                 ; TimerA ausschalten
            mov.w       #16384, &TA0CCR0                ; 2 sec = 2/8 * 65536 = 16384 in TA0CCRO schreiben 
            reti

Zustand_4
            reti
//______________________________________________________________________________
ADC12_ISR
            
            
            
            mov.w   &ADC12MEM0, R8
            mov.w   &ADC12MEM1, R8
            mov.w   &ADC12MEM2, R8
            mov.w   &ADC12MEM3, R8
            clrc
            mov.w   &ADC12MEM0, R13
            add.w   &ADC12MEM1, R13
            add.w   &ADC12MEM2, R13
            add.w   &ADC12MEM3, R13
            rra.w   R13
            rra.w   R13
            mov.w   R13, R8
            incd.w  R13
            and.w   #32-1, R13
            reti
//______________________________________________________________________________
TA0             
            mov.w       #TACLR,&TA0CTL                  ; alle zufällig gesetzten Interrupts löschen
//            mov.w       #40960, &TA0CCR0                ; 5 sec = 5/8 * 65536 = 40960 in TA0CCRO schreiben             
//            mov.w       #8192,&TA0CCR0                  ; 1 sec = 1/8 * 65536 = 8192 in TA0CCRO schreiben 
//            mov.w       #16384, &TA0CCR1                ; 2 sec = 2/8 * 65536 = 16384 in TA0CCRO schreiben 
//            mov.w       #OUTMOD_7, &TA0CCTL1            ; Output Mode 7: Reset/Set (S.468 User's Guide)
            mov.w       #CCIE, &TA0CCTL0                ; Interrupt wenn Wert in TA0CCTL0 erreicht
            mov.w       #TASSEL_1+ID_2+MC_1, &TA0CTL    ; Timer konfiguration: ACLK, Input devider 2 (0...8s), Up-Mode
            ret            
//______________________________________________________________________________
SubR_Poti_Wahl 
            cmp.w       #1, R7
            jz          Poti_R
            jmp         Poti_L
            
Poti_R
            mov.w       #txt_Poti_R, R12                ; kopiere String mit "Poti_R" Meldung in R12
            call        #SubR_Str_TX                    ; die SubRoutine aufrufen um "Poti_R" Meldung zu senden

            ret
            
Poti_L
            mov.w       #txt_Poti_L, R12                ; kopiere String mit "Poti_l" Meldung in R12
            call        #SubR_Str_TX                    ; die SubRoutine aufrufen um "Poti_L" Meldung zu senden
                        
            
            
            ret
//______________________________________________________________________________
// normiert ADC12_Wert in 0...3300mV, bei Vref = 3,3V
// EingangsWert in R12, binäres DatenFormat
// AusgangsWert in R12, binäres DatenFormat
//______________________________________________________________________________
SubR_N_3300MV  
            rla.w       R12                             ; MessWert ins 15Q1 Datenformat
            mov.w       R12, &MPY                       ; MessWert ins HW-MUL.Reg.1
            mov.w       #0x6726, &OP2                   ; N.Wert 1Q15 ins HW.MUL.Reg.2
            nop                                         ; warte 1 Takt um sicherzugehen
            mov.w       &RESHI, R12                     ; Wert: 0...3300 ins InOut.Reg
            ret
//______________________________________________________________________________
SubR_Str_TX
            cmp.b       #0xFF, 0(R12)                   ; Test ob Ptr.Inhalt == 0xFF?
            jz          Str_fertig                      ; wenn ja, dann weiter ab Label
            mov.b       @R12+, &UCA1TXBUF               ; sende das Zeichen und hole das nächste Zeichen
TX_no_ok2
            bit.b       #UCTXIFG,&UCA1IFG               ; warten bis Transfer fertig
            jz          TX_no_ok2         
            jmp         SubR_Str_TX                     ; weiter mit ....
Str_fertig
            ret

txt_Start
            db          "1425104"                       ; String für Matrikelnummer  
            db          0xFF
txt_Ende            
            db          "System Off"                    ; String für System off
            db          0xFF            
txt_Poti_R            
            db          "Poti R"                        ; String für Poti_R
            db          0xFF           
txt_Poti_L            
            db          "Poti L"                        ; String für Poti_L
            db          0xFF            
//______________________________________________________________________________            

            ORG         0FFFEh                          ; MSP430 RESET Vector
            DW          RESET                   
            
            ORG         0FFDEh                          ; Interrupt Vektor für die Flags in Register P1IFG
            DW          T_OnOff_ISR                     ; die Interrupt Vektor Adresse 0xFFDE steht im Datenblatt des MSP430F5529 auf Seite 52 in der Tabelle (Table 6-1. Interrupt Sources, Flags, and Vectors)   

            ORG         0FFD4h                          ; Interrupt Vektor für die Flags in Register P1IFG
            DW          T_SET_ISR                       ; die Interrupt Vektor Adresse 0xFFDE steht im Datenblatt des MSP430F5529 auf Seite 52 in der Tabelle (Table 6-1. Interrupt Sources, Flags, and Vectors)    

            ORG         0FFDCh                          ; Interrupt Vektor für die Flags UCA1RXIFG, UCA1TXIFG (USCI_A1 Receive or Transmit Interrupt)
            DW          UART_RxD_ISR                    ; die Interrupt Vektor Adresse 0xFFDE steht im Datenblatt des MSP430F5529 auf Seite 52 in der Tabelle (Table 6-1. Interrupt Sources, Flags, and Vectors)
            
            ORG         0FFEAh                          ; Interrupt Vektor für das Flag CCIFG0 im Register TA0CCTL0 (dieses Flag wird gesetzt, wenn der Timer_A0 bis zu TA0CCR0 hochgezählt hat)
            DW          TIMER_A0_ISR                    ; die Interrupt Vektor Adresse 0xFFEA steht im Datenblatt des MSP430F5529 auf Seite 52 in der Tabelle (Table 6-1. Interrupt Sources, Flags, and Vectors)
            
            ORG         0FFECh                          ; Interrupt Vektor für die Flags
            DW          ADC12_ISR                       ; die Interrupt Vektor Adresse 0xFFEC steht im Datenblatt des MSP430F5529 auf Seite 52 in der Tabelle (Table 6-1. Interrupt Sources, Flags, and Vectors)

            END
