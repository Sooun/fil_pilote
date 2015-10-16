/* _______________________________________________________________________________
 *
 * 			P R O G R A M M E    'Tx_Fil_Pilote'
 * _______________________________________________________________________________

 Change History :
 ---------------
 * Rev   Date         Description
 * 1.0   16/10/2015   Initial release

  
 Configuration Hard:
 -------------------
 Fosc = 32Mhz (limite de la Freq interne de ce PIC)
 
 Rx UART => pin RA1, APFCON0 register
 Tx UART => pin RA0, APFCON0 register
 
 PIN OUTPUT:
 ¨¨¨¨¨¨¨¨¨¨¨
 RF OUTPUT => RC1
 LED ROUGE => RA4
 LED VERTE => RC5

 PIN INPUT:
 ¨¨¨¨¨¨¨¨¨
 Boutton Poussoir => RA5
 Li_Moins => RC4
 Li_Plus => RC3
  
 Remarque:
 ---------

 */


#include <pic16f1825.h>
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>



/********** C O N F I G U R A T I O N   B I T S ******************************/

#pragma config CPD = OFF        // Data Memory Code Protection disable
#pragma config BOREN = OFF      // Brown-out Reset disabled in hardware and software
#pragma config IESO = OFF       // Internal/External Switchover is disable
#pragma config FOSC = INTOSC    // Internal oscillator => Voir registre OSCCON
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor disable
#pragma config MCLRE = ON       // MCLR/VPP pin function is MCLR
#pragma config WDTE = OFF       // Watchdog Timer disable
#pragma config CP = OFF         // Flash Program Memory Code Protection is Off
#pragma config CLKOUTEN = OFF   // CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
#pragma config PLLEN = ON       // 4x PLL enabled even on internal oscillator.
#pragma config WRT = OFF        // Flash Memory Self-Write Protection Off
#pragma config STVREN = ON      // ON Stack Overflow or Underflow will cause a Reset
#pragma config BORV = HI        // Brown-out Reset Voltage (Vbor), high trip point selected.
#pragma config LVP = OFF        // High-voltage on MCLR/VPP must be used for programming

/********** DEFINE ******************************/
//SIMPLIFICATION LANGUAGE
#define TRUE         1		// ON
#define FALSE        0  	// OFF
#define INPUT_PIN    1		// INPUT PIN
#define OUTPUT_PIN   0		// INPUT OUT
#define ON           1		// ON
#define OFF          0		// OFF
#define HIGH         1		// ON
#define LOW          0  	// OFF
#define IO           0
#define AD           1

//MASK
#define SPBRG_9600_BAUD     207                 //  Valeur de spbrg : BAUD = FOSC / (16 * (spbrg + 1))
#define _XTAL_FREQ          32000000            //oscillator frequency en Hz

//FLAG
#define CLIGN_ROUGE		Flag.bits.b0    // Indique que la led rouge doit clignoter
#define CLIGN_VERT 		Flag.bits.b1	// Indique que la led verte doit clignoter
#define SYNCHRO 		Flag.bits.b2	// Flag de demande synchro

//PINS ATTRIBUTION
#define BP      	!PORTAbits.RA5
#define LED_VERT	LATAbits.LATA4
#define LED_ROUGE	LATCbits.LATC5
#define Li_Moins        PORTCbits.RC4
#define Li_Plus         PORTCbits.RC3
#define RF_OUT  	LATCbits.LATC1

#define TRIS_BP      	TRISAbits.TRISA5
#define TRIS_LED_VERT	TRISAbits.TRISA4
#define TRIS_LED_ROUGE	TRISCbits.TRISC5
#define TRIS_Li_Moins   TRISCbits.TRISC4
#define TRIS_Li_Plus    TRISCbits.TRISC3
#define TRIS_RF_OUT  	TRISCbits.TRISC1

#define  ANSEL_Li_Plus  ANSELCbits.ANSC3           // Place le port RA2 en I/O
#define  ANSEL_Li_Moins NONE
#define  ANSEL_BP       NONE

#define WPU_BP          WPUAbits.WPUA5          // Place une résistance de pull-up sur le port RA5
#define INT_BP          INTCONbits.IOCIE   // interrupt-on-change pour le boutton BP
#define FLG_BP          IOCAFbits.IOCAF5    //Flag d'interruption sur le BP
#define DIR_BP          IOCANbits.IOCAN5     // Direction du front qui génère une interruption sur BP


// CONSTANTES

#define ARRET		0x2C674E81
#define CONF		0x2C674E90
#define ECO             0x2C674E91
#define HORS_GEL	0x2C674E92

#define NB_TRAME 5      // Nombre de trame envoyé


#define pulse_high              RF_OUT = HIGH; __delay_us(275); RF_OUT = LOW	//275us => 2 200 cyc
#define pulse_low_zero		__delay_us(240)                                 //240us => 1 920 cyc
#define pulse_low_one		__delay_us(1300)				//1300us => 10 400cyc
#define pulse_low_start		__delay_us(2675)				//2675us =>  21 400cyc
#define delay_10ms()		__delay_ms(10)					//10ms => 80 000 cyc
#define delay_read		__delay_ms(70);					// 70ms. Delais entre 2 lecture de Li_Plus/Li_Moins


/********** DEFINITION STRUCTURE ******************************/

typedef unsigned char		BYTE;				// 8-bit unsigned
typedef union _BYTE_VAL
{
    BYTE Val;
    struct
    {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
    } bits;
} BYTE_VAL, BYTE_BITS;


/********** DEFINITION des variables golbales ******************************/
volatile BYTE_VAL Flag;				// Flag général
volatile unsigned int n_TMR1, n_TMR1_bis, n_Des, n_Mont; 

/* ____________________________________________________________________________
 * 
 *                            Function Prototype
 * ____________________________________________________________________________
 */

void InitializeSystem(void);
void interrupt Interruption_Haute_Priorite(void);
void ISR_TMR1(void);
void ISR_BP(void);
void Display_InitRS232(void);
void send_one (void);
void send_zero (void);
void send_preambule (void);
void send(unsigned long);
void Gestion_LED(unsigned long);
unsigned long read(void);
extern void	putch(char);
char* StrLg_to_Str(unsigned long);



/* ____________________________________________________________________________
 * 
 *                            Interruptions
 * ____________________________________________________________________________
 */

/* Routine d'interruption priorité haute - High priority interrupt routine */
void interrupt Interruption_Haute_Priorite(void)
{
        NOP();
        NOP();
	if(PIR1bits.TMR1IF)
	{
            ISR_TMR1();			// Routine d'interruption de TMR1
            PIR1bits.TMR1IF=0;		//Reset Flag
	}
        else if (FLG_BP)
        {
            ISR_BP();			// Routine d'interruption du BP
            FLG_BP=0;                   //Reset Flag
        }
} // End Interruption_Haute_Priorite()



/********************************************************************
 * Function:        void InitializeSystem(void)
 *******************************************************************/
void InitializeSystem(void)
{
	// Configuration du Registre de control de FOSC
        OSCCONbits.IRCF = 0b1110;   // Internal Oscillator Frequency set to 8 if PPL disable or 32Mhz if PPL enable
									// PPL defined by PLLEN in Config word
									//				    ||====== ||
									// Ici on a Choisi PPL ON => Fosc = || 32Mhz ||
									//				    ||====== ||
        OSCCONbits.SCS = 0b00;      // System Clock determined by FOSC<2:0> in Configuration Word 1.
									// Ici on a choisi internal oscillator


	// Configuration du Timer1
        T1CONbits.TMR1ON = 1;       // Enables Timer1
	T1CONbits.TMR1CS = 0b00;    // Timer1 clock source is Instruction Clock (Fosc/4) => 32/4 = 8Mhz ici
	T1CONbits.T1OSCEN = 0 ;     // Dedicated Timer1 oscillator circuit disabled
	T1CONbits.T1CKPS = 0b11 ;   // 1:8 Prescale value => 8Mhz/8 = 1Mhz => 1us par incrémentation du timer
	T1CONbits.nT1SYNC = 1;      // Do not synchronize external clock inputs
	T1GCONbits.TMR1GE = 0;      // Timer1 counts regardless of Timer1 gate function
        PIE1bits.TMR1IE = 1;        // interrupt TMR1 enable
        INTCONbits.PEIE = 1;        // Peripherical interruption enable


	// Configuration de l'UART, (BAUD) 9600 N 8 1 -> Fosc = 32 MHz
	TXIE = 0;                   //USART_TX_INT_OFF ,PIE1 register
	TXEN = 1;                   //USART_ASYNCH_MODE, TXSTA register
	SYNC = 0;
	SPEN = 1;
	TX9 = 0;                    //USART_EIGHT_BIT,  TXSTA register
	BRGH = 1;                   // USART_BRGH_HIGH  TXSTA register
	SPBRGL = SPBRG_9600_BAUD;   // (BAUD) 9600 N
	RXDTSEL = 1;                // Rx sur la pin RA1, APFCON0 register
	TXCKSEL = 1;                // Tx sur la pin RA0, APFCON0 register


	// Validation des l'interruptions
	INTCONbits.GIE = 1; 	// Global Interrupt Enable bit

        INT_BP = 1;   // interrupt-on-change pour le boutton BP
        DIR_BP = 1;   // Front descendant qui génère une interruption sur BP

	
	// Configuration des PINS
        TRIS_BP = INPUT_PIN;
        TRIS_LED_ROUGE = OUTPUT_PIN;
        TRIS_LED_VERT = OUTPUT_PIN;
        TRIS_Li_Moins = INPUT_PIN;
        TRIS_Li_Plus = INPUT_PIN;
        TRIS_RF_OUT = OUTPUT_PIN;

        ANSEL_Li_Plus = IO;         // Place le port Li_Plus en I/O

        OPTION_REGbits.nWPUEN=0;    // Autorise les résistances de rappel individuel
        WPU_BP = ON;                // Place une résistance de pull up sur BP


        // Initialisation programme
        LED_ROUGE = ON;
        LED_VERT = OFF;
        RF_OUT = OFF;
        CLIGN_ROUGE = OFF;
        CLIGN_VERT = OFF;
        SYNCHRO = OFF;


//        _delay(100000);
//        _delay(100000);
//        _delay(100000);
//        _delay(100000);
	Display_InitRS232();

} // End InitializeSystem()


/********************************************************************
 * Function:        void ISR_TMR1(void)
 *******************************************************************/
void ISR_TMR1(void)
{
    n_TMR1++; 		//Compte le nombre de passage dans TMR1
    if (n_TMR1 > 15)
    {
        if (CLIGN_VERT)
        {
            LED_VERT= ~LED_VERT;
            n_TMR1= 0;
        }
        else if (CLIGN_ROUGE)
        {
            LED_ROUGE= ~LED_ROUGE;
            n_TMR1= 0;
        }
    }
}

/********************************************************************
 * Function:        void ISR_BP(void)
 *******************************************************************/
void ISR_BP(void)
{
    SYNCHRO = 1;
    INT_BP = 0;         // coupe l'interruption sur BP
}

/********************************************************************
 * Function:        void InitializeSystem(void)
 *******************************************************************/

/* Affichage message initialisation OK sur l'interface PC */
void Display_InitRS232(void)
{
	/* Envoie vers PC : */
	printf("\r\n______________________________________________________________________\r\n\n");
	printf("  L'initialisation de la communication RS232 s'est bien deroulee \r\n");
	printf("______________________________________________________________________\r\n\n");

} // End Display_InitRS232()


/********************************************************************
 * Function:        void putch(void)
 *******************************************************************/
void putch(char data) 
{
	while( ! TXIF)
	continue;
	TXREG = data;
}

/********************************************************************
 * Function:        void send(unsigned long data)
 *******************************************************************/
void send(unsigned long data)
{
	char j=0;
	char i=0;
	for (i=0; i < NB_TRAME; i++)
	{
		send_preambule();
		while (j<32)
		{
			if ((data & (1ul<<(31-j)))!= 0)		// Test le jieme bit pour savoir si 1 ou 0
			{
				send_one();
			}
			else
			{
				send_zero();
			}
			j++;
		}
		pulse_high;
		j=0;
		delay_10ms();
	}
}

/********************************************************************
 * Function:        void send_one (void)
 *******************************************************************/
void send_one (void)
{
	pulse_high;
	pulse_low_one;
	pulse_high;
	pulse_low_zero;

}

/********************************************************************
 * Function:        void send_zero (void)
 *******************************************************************/
void send_zero (void)
{
	pulse_high;
	pulse_low_zero;
	pulse_high;
	pulse_low_one;

}

/********************************************************************
 * Function:        void send_preambule (void)
 *******************************************************************/
void send_preambule (void)
{
	pulse_high;
	pulse_low_start;
}

/********************************************************************
 * Function:        unsigned long read(void)
 *******************************************************************/
unsigned long read(void)
{
    unsigned long order=0;
    char i=0;
    char input;
    char old_input;
    input = (Li_Plus <<1) + (Li_Moins<<0) ;
    delay_read;
    old_input=input;
    for (i=0; i < 3; i++)
    {
        input = (Li_Plus <<1) + (Li_Moins<<0) ;
	if (input==old_input)
	{
            delay_read;
        }
        else
        {
            input=0xFF;
            break;
        }
    }
    switch(input)
    {
        case 3:
            order = CONF;
            break ;
        case 0:
            order = ECO ;
            break ;
        case 2:
            order = HORS_GEL;
            break ;
        case 1:
            order = ARRET;
            break ;
        case 0xFF:
            order = 0 ;
            break ;
        default :
            order = 0;
            break;
    }
    return order;
}

/********************************************************************
 * Function:        Gestion_LED(unsigned long data)
 *******************************************************************/
void Gestion_LED(unsigned long data)
{
    if (data == CONF)
    {
        CLIGN_ROUGE =OFF;
        CLIGN_VERT =OFF;
        LED_ROUGE=OFF;
        LED_VERT=ON;
    }
    else if (data == ECO)
    {
        CLIGN_ROUGE =OFF;
        CLIGN_VERT =ON;
        n_TMR1=0;
        LED_ROUGE=OFF;
        LED_VERT= OFF;
    }
    else if (data == HORS_GEL)
    {
        CLIGN_ROUGE =OFF;
        CLIGN_VERT =OFF;
        LED_ROUGE=ON;
        LED_VERT= OFF;
    }
    else if (data ==ARRET)
    {
        CLIGN_ROUGE =ON;
        n_TMR1=0;
        CLIGN_VERT =OFF;
        LED_ROUGE=OFF;
        LED_VERT= OFF;
    }
}

/********************************************************************
 * Function:        StrLg_to_Str(unsigned long data)
 *******************************************************************/
char* StrLg_to_Str(unsigned long data)
{
    static char string[5]={0};
    string[3]=(char)data ;
    string[2]=(char)(data>>8) ;
    string[1]=(char)(data>>16) ;
    string[0]=(char)(data>>24) ;
    return string; 
}



/* ____________________________________________________________________________
 *
 *                            Function Principale
 * ____________________________________________________________________________
 */

void main()
{
    unsigned long cmd = ARRET; 		// Contient la trame qui doit être envoyé. Contient 32bits
    unsigned long Old_cmd = ARRET;

    InitializeSystem();		// Initialisation de l'ensemble des périfériques du microcontrôleur

    // Pour permettre la synchronisation en début de trame, le programme reste bloqué dans cette boucle tant que la condition n'est pas rempli.
    while (1)
    {
        cmd = read();
        if(!cmd)
        {
            printf(" !! Erreur lecture Opto-coupleur\r\n");
        }
        else if (cmd != Old_cmd)
        {
            Old_cmd=cmd;
            send(cmd);
            printf("Commande en cours : %lX\r\n",cmd);
            Gestion_LED(cmd);
        }
        if (SYNCHRO)
        {
            send(cmd);
            printf("Synchronisation de la commande : %lX\r\n",cmd);
            SYNCHRO = OFF;
            FLG_BP = 0;         // Efface le flag de l'interruption sur BP
            INT_BP = 1;         // Puis ré-autorise l'interruption sur BP
        }
    }
}

