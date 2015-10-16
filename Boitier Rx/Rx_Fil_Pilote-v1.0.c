/* _______________________________________________________________________________
 *
 * 			P R O G R A M M E    'Rx_Fil_Pilote'
 * _______________________________________________________________________________
 
 Change History :
 ---------------
 * Rev   Date         Description
 * 1.0   16/10/2015   Initial release
 
 
 Configuration Hard:
 -------------------
 Fosc = 32Mhz (limite de la Freq interne de ce PIC)
 CCP3 => Pin RA2
 CCP4 => Pin RC1

 UART:
 ¨¨¨¨¨
 Rx UART => pin RA1, APFCON0 register
 Tx UART => pin RA0, APFCON0 register
 
 PIN OUTPUT:
 ¨¨¨¨¨¨¨¨¨¨¨
 LED ROUGE => RA5
 LED VERTE => RC5
 OPTO_POS => RC4
 OPTO_NEG => RC3

 PIN INPUT:
 ¨¨¨¨¨¨¨¨¨¨
 RF INPUT/CCP3 => Pin RA2
 RF INPUT/CCP4 => Pin RC1
  
 Remarque:
 ---------
 On pourrait également faire tourner le Pic à 4Mhz sans que cela ne change rien
 au programme: On a juste à passer le postscaller du TIMER1 à 1.
 De même, à 8Mhz, on a juste à passer le postscaller du TIMER1 à 2.

 La communication UART a été testé le 30/08 et fonctionne parfaitement.
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



#define TRUE         1		// ON
#define FALSE        0  	// OFF
#define INPUT_PIN    1
#define OUTPUT_PIN   0
#define ON           1		// ON
#define OFF          0

//FLAG
#define PulseOK		Flag.bits.b0    // Indique si pulse OK
#define CLIGN_ROUGE	Flag.bits.b1    // Indique que la led rouge doit clignoter
#define CLIGN_VERT 	Flag.bits.b2	// Indique que la led verte doit clignoter


#define SPBRG_9600_BAUD 	207		//  Valeur de spbrg : BAUD = FOSC / (16 * (spbrg + 1))

// Attribution des pins par functions

#define LED_ROUGE	LATAbits.LATA5
#define LED_ORANGE	LATAbits.LATA4
#define LED_VERT	LATCbits.LATC5
#define OPTO_NEG	LATCbits.LATC4
#define OPTO_POS	LATCbits.LATC3

#define TRIS_LED_ROUGE	TRISAbits.TRISA5
#define TRIS_LED_ORANGE	TRISAbits.TRISA4
#define TRIS_LED_VERT	TRISCbits.TRISC5
#define TRIS_OPTO_NEG	TRISCbits.TRISC4
#define TRIS_OPTO_POS	TRISCbits.TRISC3

#define CP3_TRIS    TRISAbits.TRISA2            // Pin RA2 pour le moculde CCP3
#define CP4_TRIS    TRISCbits.TRISC1            // Pin RC1 pour le moculde CCP4

#define CAPTURE_INT_ON          0b11111111  	/* Enable Capture interrupt */
#define CAPTURE_INT_OFF         0b01111111  	/* Disable Capture interrupt */
#define CAP_EVERY_FALL_EDGE     0b10000100  	/* Capture on every falling edge*/
#define CAP_EVERY_RISE_EDGE     0b10000101  	/* Capture on every rising edge*/
#define CAP_EVERY_4_RISE_EDGE   0b10000110  	/* Capture on every 4th rising edge*/
#define CAP_EVERY_16_RISE_EDGE  0b10000111  	/* Capture on every 16th rising edge*/



// CONSTANTES
#define ARRET		0x2C674E81
#define CONF		0x2C674E90
#define ECO             0x2C674E91
#define HORS_GEL	0x2C674E92


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



union capstatus
{
  struct
  {
        unsigned Cap1OVF:1; /* CAPTURE1 overflow status. Following a read, the bit will be set to indicate overflow */
        unsigned Cap2OVF:1; /* CAPTURE2 overflow status. Following a read, the bit will be set to indicate overflow*/
        unsigned Cap3OVF:1; /* CAPTURE3 overflow status. Following a read, the bit will be set to indicate overflow*/
        unsigned Cap4OVF:1; /* CAPTURE4 overflow status. Following a read, the bit will be set to indicate overflow */
   };
  unsigned :8;
};


/* used to hold the 16-bit captured value */
union CapResult
{
 unsigned int lc;	// holds the 16-bit captured value
 char bc[2];		// holds the 16-bit captured value
};





/********** DEFINITION des variables golbales ******************************/
volatile BYTE_VAL Flag;				// Flag général
volatile unsigned long rise,fall,pulse_width,lastrise,lowtime;
volatile unsigned int n_TMR1, n_TMR1_bis, n_Des, n_Mont; 
volatile int Array[64];				// Contient la longueur des impulstions basses
//volatile int High_Array[64];			// Contient la longueur des impulstions hautes
volatile union capstatus CapStatus;             // Déclaration de la variable "CapStatus"



/* ____________________________________________________________________________
 * 
 *                            Function Prototype
 * ____________________________________________________________________________
 */

void InitializeSystem(void);
void interrupt Interruption_Haute_Priorite(void);
void ISR_CCP3(void);
void ISR_CCP4(void);
void ISR_TMR1(void);
void Display_InitRS232(void);
void OpenCapture3(unsigned char);
void OpenCapture4(unsigned char);
unsigned int ReadCapture3(void);
unsigned int ReadCapture4(void);
extern void	putch(char);
void Gestion_OPTO_LED(unsigned long);
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
	else if(PIR3bits.CCP3IF)
	{
		ISR_CCP3();			// Routine d'interruption de CCP3
		PIR3bits.CCP3IF =0;		//Reset Flag
	}
	else if (PIR3bits.CCP4IF)
	{
		ISR_CCP4();			// Routine d'interruption de ECCP4
		PIR3bits.CCP4IF =0;		//Reset Flag
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
                                                                    //									||====== ||
                                                                    // Ici on a Choisi PPL ON => Fosc = || 32Mhz ||
                                                                    //									||====== ||
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

    // Configuration des modules CCP1 & CCP2
    OpenCapture3(                   CAPTURE_INT_ON		&
                                    CAP_EVERY_RISE_EDGE	);		// interuption sur front montant

    ANSELAbits.ANSA2 = 0;           // Place le port RA2 en I/O

    OpenCapture4(                   CAPTURE_INT_ON		&
                                    CAP_EVERY_FALL_EDGE	);		// front descendant: pas d'interuption

    ANSELCbits.ANSC1 = 0;           // Place le port RC1 en I/O

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
    INTCONbits.GIE = 1; 				// Global Interrupt Enable bit


    // Initialisation programme
    TRIS_LED_ROUGE = OUTPUT_PIN;
    TRIS_LED_VERT = OUTPUT_PIN;
    TRIS_LED_ORANGE = OUTPUT_PIN;
    TRIS_OPTO_NEG = OUTPUT_PIN;
    TRIS_OPTO_POS = OUTPUT_PIN;

    LED_ROUGE = ON;
    LED_VERT = OFF;
    LED_ORANGE = OFF;
    OPTO_POS = OFF;
    OPTO_POS = OFF;


//        _delay(100000);
//        _delay(100000);
//        _delay(100000);
//        _delay(100000);
    Display_InitRS232();

} // End InitializeSystem()

/********************************************************************
 * Function:        void ISR_CCP1(void)
 *******************************************************************/
void ISR_CCP3(void)					// Interruption sur front montant sur RA2
{
    n_Mont = n_TMR1;
    if (!PulseOK)
    {
        n_TMR1 = 0;					// On remet le compteur à zéro
        rise = n_Mont*65536 + ReadCapture3();	// Lit la valeur de CCP1
        fall = n_Des*65536 + ReadCapture4();	// Lit la valeur de ECCP1
        lowtime = rise - fall;			// Largeur du front bas
        pulse_width = fall - lastrise;		// Largeur du front haut => Presque inutile
        lastrise = ReadCapture3();			// Sauvegarde
        PulseOK = TRUE;				// Indique qu'on a reçu un nouveu RF bit
//            printf("lowtime:  %u us \r\n", lowtime);
   }
}

/********************************************************************
 * Function:        void ISR_ECCP1(void)
 *******************************************************************/
void ISR_CCP4(void)			// Interruption sur front descendant
{
    n_Des = n_TMR1;				// On ne fait qu'enregistrer le compteur de TMR1
}

/********************************************************************
 * Function:        void ISR_TMR1(void)
 *******************************************************************/
void ISR_TMR1()
{
    n_TMR1++; 		//Compte le nombre de passage dans TMR1
    n_TMR1_bis++;
    if (n_TMR1_bis > 15)
    {
        if (CLIGN_VERT)
        {
            LED_VERT= ~LED_VERT;
            n_TMR1_bis= 0;
        }
        else if (CLIGN_ROUGE)
        {
            LED_ROUGE= ~LED_ROUGE;
            n_TMR1_bis= 0;
        }
    }
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
 * Function:        void OpenCapture3(void)
 *******************************************************************/
void OpenCapture3(unsigned char config)
{
  CCP3CON = config&0x0F; //  Configure capture

  if(config&0x80)
  {
    PIR3bits.CCP3IF = 0;  // Clear the interrupt flag
    PIE3bits.CCP3IE = 1;  // Enable the interrupt
  }

  CapStatus.Cap3OVF = 0;  // Clear the capture overflow status flag

  CP3_TRIS = 1;		//TRIS direction as input
}

/********************************************************************
 * Function:        void OpenCapture4(void)
 *******************************************************************/
void OpenCapture4(unsigned char config)
{
  CCP4CON = config&0x0F; //  Configure capture

  if(config&0x80)
  {
    PIR3bits.CCP4IF = 0;  // Clear the interrupt flag
    PIE3bits.CCP4IE = 1;  // Enable the interrupt
  }

  CapStatus.Cap4OVF = 0;  // Clear the capture overflow status flag

  CP4_TRIS = 1;		//TRIS direction as input
}

/********************************************************************
 * Function:        void ReadCapture3(void)
 *******************************************************************/
unsigned int ReadCapture3(void)
{
  union CapResult Cap;

  CapStatus.Cap3OVF = 0;   // Clear the overflow flag in the
                           // status variable

  if(PIR3bits.CCP3IF)      // If the overflow flag is set
    CapStatus.Cap3OVF = 1; // Set the overflow flag

  Cap.bc[0] = CCPR3L;      // Read CA3L into the lower byte
  Cap.bc[1] = CCPR3H;      // Read CA3H into the high byte

  return (Cap.lc);         // Return the int
}

/********************************************************************
 * Function:        void ReadCapture4(void)
 *******************************************************************/
unsigned int ReadCapture4(void)
{
  union CapResult Cap;

  CapStatus.Cap4OVF = 0;   // Clear the overflow flag in the
                           // status variable

  if(PIR3bits.CCP4IF)      // If the overflow flag is set
    CapStatus.Cap4OVF = 1; // Set the overflow flag

  Cap.bc[0] = CCPR4L;      // Read CA3L into the lower byte
  Cap.bc[1] = CCPR4H;      // Read CA3H into the high byte

  return (Cap.lc);         // Return the int
}

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
 * Function:        Gestion_LED(unsigned long data)
 *******************************************************************/
void Gestion_OPTO_LED(unsigned long i)
{
    if (i == CONF)
    {   OPTO_NEG=OFF;
        OPTO_POS=OFF;
        CLIGN_ROUGE =OFF;
        CLIGN_VERT =OFF;
        LED_ROUGE=OFF;
        LED_VERT=ON;
    }
    else if (i == ECO)
    {
        OPTO_NEG= ON;
        OPTO_POS= ON;
        CLIGN_ROUGE =OFF;
        CLIGN_VERT =ON;
        n_TMR1_bis=0;
        LED_ROUGE=OFF;
        LED_VERT= OFF;
    }
    else if (i == HORS_GEL)
    {
        OPTO_NEG= ON;
        OPTO_POS= OFF;
        CLIGN_ROUGE =OFF;
        CLIGN_VERT =OFF;
        LED_ROUGE=ON;
        LED_VERT= OFF;
    }
    else if (i ==ARRET)
    {
        OPTO_NEG= OFF;
        OPTO_POS= ON;
        CLIGN_ROUGE =ON;
        n_TMR1_bis=0;
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
    int j;				// Compteur d'impulsion. Si la trame est composé de 32bits, alors j = 64 au maximum.
    int badData = FALSE;		// Variable qui déclare si la trame encours de décodage est toujours cohérente
    unsigned long data; 		// Contient la trame reçu et décodé. Contient 32bits

    InitializeSystem();		// Initialisation de l'ensemble des périfériques du microcontrôleur

	// Pour permettre la synchronisation en début de trame, le programme reste bloqué dans cette boucle tant que la condition n'est pas rempli.
    while (1)
    {
        while( (lowtime) < 2500)                        // Front bas pendant 2,5ms
        {
            PulseOK = FALSE;			// La synchronisation ne peut pas démarrer tant que le front bas est trop court
        }

        j = 0;						// Pour avoir la correspondance avec le numéro de bit faire j+1
        data = 0;
        badData = FALSE;

        while(j<63)
        {
            if (PulseOK)
            {
                Array[j] = lowtime;  			// Formate la valeur du front bas pour contenir des us pour faciliter la lecture et l?ajustement des timings.
                if (Array[j] < 150 || Array[j] > 1600)  // Si on est en dehors des clous pour le front bas.
                {
                    badData = TRUE; 			// Not valid pulse, exit loop and wait for next start
                    break;
                }
                if (j%2 ==0 ) 				//  longeur dr on rentre le byte de donné dans data (numéro de bit impair)
                {
                    if (Array[j] > 500)			// le RF bit est un 1
                    {
                        data = (data << 1)+1;		// On rentre un 1 dans data
                    }
                    else				// le RF bit est un 0
                    {
                        data = (data << 1);		// On rentre un 0 dans data
                    }
                }
                else 					// si j est pair on vérifie que le codage Manchester est bien respecté.
                {
                    if (Array[j] > 500)			// le RF bit est un 1
                    {
                        if (data & 1 ) 			// On teste que le dernier bit de data est bien un 0
                        {
                            badData = TRUE;		// Si le résultat est un 1 alors il y a un soucis.
                            break;
                        }
                    }
                    else				// le RF bit est un 0
                    {
                        if ( data & 1 == 0 )            // On teste que le dernier bit de data est bien un 1
                        {
                            badData = TRUE;             // Si le résultat est un 0 alors il y a un soucis.
                            break;
                        }
                    }
                }
                PulseOK = FALSE; 			// Get another pulse
                ++j;
            }
        }

        if (!badData)                                   // Si la trame reçue est cohérente, on traite.
        {
            printf("Ordre : %lX = %s\r\n",data,StrLg_to_Str(data));
            Gestion_OPTO_LED(data);
        }
        PulseOK = FALSE;
    }
}

