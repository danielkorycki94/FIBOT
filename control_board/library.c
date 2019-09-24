#include "library.h"


    Timer_A_initCompareModeParam paramA1CCR1 = { 0 };
    Timer_A_initCompareModeParam paramA1CCR2 = { 0 };
    Timer_B_initCompareModeParam paramB0CCR1 = { 0 };
    Timer_B_initCompareModeParam paramB0CCR2 = { 0 };
    Timer_B_initCompareModeParam paramB1CCR1 = { 0 };
    Timer_B_initCompareModeParam paramB1CCR2 = { 0 };
    uint16_t pl_temp = 0;
    uint16_t pp_temp = 0;
// zmienne globalne do obs³ugi czujników odleg³owsci
    uint16_t czujniki[4] = { 0 }; //tablica w której bêd¹ przechowywane wartoœci pomiaru czasu impulsu odpowiedzi z czujników
    uint16_t czujniki_cm[4] = { 0 }; //tablica do przechowywania danych z czujników w cm
    uint16_t czas = 0;
    uint8_t nrczujnika = 0;
    uint8_t n = 0;

    uint16_t napbate = 0;
    uint16_t znacznik_serw=0, timerwlaczony =0 ;

//UART
    uint8_t RXData = 0, TXData = 0;
    uint8_t l = 0;
    uint8_t ramka_TX[10] = { 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j' }; //ramka transmitowana
    uint8_t ramka_RX[10] = { 0 }; // ramka odbierana
    uint8_t m = 0, zapis = 0;

//zmienne globalne do obs³ugi enkoderów
    int16_t impulsy[4] = { 0 }; //tablica w której bêd¹ przechowywane impulsy
    int32_t obrotytemp[4] = { 0 };
    int32_t obroty[4] = { 0 };  // tablicza obrotów ko³a
    int32_t obrp[4] = { 0 };
    int32_t obrsub[4] = { 0 };
    int predkosc[4] = { 0 }; // tablica predkosci

//zmienne globalne do sterowania predkosci silnika
    uint8_t klp = 0;
    uint8_t kpp = 0;
    uint16_t plp = 0;
    uint16_t ppp = 0;

    uint8_t kla = 0;
    uint8_t kpa = 0;
    uint16_t pla = 0;
    uint16_t ppa = 0;

    uint8_t kl = 0;
    uint8_t kp = 0;
    uint16_t pl = 0;
    uint16_t pp = 0;

    uint8_t kieruneksilnikow = 0;

    uint8_t servo1p = 0;
    uint8_t servo2p = 0;

//zmienne globalne do pomiaru napiecia na baterii i pradu na silnikach
    uint16_t adcpomiar[14] = { 0 };
    uint16_t adcwyniki[5] = { 0 };  // wyniki pomiaru adc na pieciu kana³ach

    uint16_t current_ref[5] = { 0 }; // zmierzona wartosc pradu w chwili zatrzymanego robota
    int16_t current_temp[4] = { 0 };
    long current[4] = { 0 };

//zmienne globalne do obslugi PID

    float kpPID = 0.2;
    float kiPID = 10;
    float kdPID = 0.005;

    int uchyb_S1 = 0;
    int uchybp_S1 = 0;
    int sygnal_S1 = 0;
    int Ca_S1 = 0;

    int uchyb_S2 = 0;
    int uchybp_S2 = 0;
    int sygnal_S2 = 0;
    int Ca_S2 = 0;

    int uchyb_S3 = 0;
    int uchybp_S3 = 0;
    int sygnal_S3 = 0;
    int Ca_S3 = 0;

    int uchyb_S4 = 0;
    int uchybp_S4 = 0;
    int sygnal_S4 = 0;
    int Ca_S4 = 0;



void start_UART_TX()
{
    //tutaj znajduje sie kod ktory zamienia dane odczytane z czujnikow na format niezbedny do wyslania







    //tutaj zaczynamy transmisje uart. reszta realizowana jest w przerwaniu
    l=0;
    EUSCI_A_UART_enableInterrupt(EUSCI_A1_BASE,
                                  EUSCI_A_UART_TRANSMIT_INTERRUPT);                     // Enable interrupt
    EUSCI_A_UART_transmitData(EUSCI_A1_BASE,
                              ramka_TX[l]);
}

//funkcje które pozwalaja odczytac dane z czujników odleglosci, predkosc i prad.
uint16_t * daneczujniki()
{
    return czujniki_cm;

}
int * getspeed()
{
    return predkosc;

}
// wylicza prad w ten sposób ze bierze aktualny oddejmuje pomiary z zatrzymanych silnikow i zwraca wartosc
long * getcurrent()
{
    uint8_t i =0;
    for(i = 0; i<4 ; i++)
    {
        current_temp[i] = (int)(adcwyniki[i+1]-current_ref[i+1]);
        current[i] = (current_temp[i] * MNOZNIKNAPC);
    }
    return current;

}
//funkcja która mierzy odczyt z czujnika pradu najpierw zatrzymujac silniki
void calibrateDCcurrent()
{
    DCmotor(1,1,0,0); // zatrzymujemy silniki
    __delay_cycles(160000);

    uint8_t i =0;
    for(i=0; i<5 ; i++)
    {
        current_ref[i] = adcwyniki [i];
    }
}


void setbuzzer(uint8_t stan) // 1 = w³¹czony, 0 = wy³aczony
{
    if (stan == 1)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_PJ, GPIO_PIN3);
    }
    else if (stan == 0)
    {
        GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN3);
    }
}



void setled(uint8_t stan) // 1 = w³¹czony, 0 = wy³aczony
{
    if (stan == 1)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_PJ, GPIO_PIN2);
    }
    else if (stan == 0)
    {
        GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN2);
    }
}
// funkcja odpowiadajaca za sterowanie serwomechaznizmami
void setservos(uint8_t servo1, uint8_t servo2)
{
    if(servo1 != servo1p || servo2 != servo2p )
    {
        if(timerwlaczony == 0)
        {
            //ustawienie tajmera B1, który bêdzie sluzyl do sterowania predkoscia serw
                     //Ustawienia TIMERA B1
                     Timer_B_initUpModeParam paramB1 = { 0 };
                     paramB1.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
                     paramB1.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_1;
                     paramB1.timerPeriod = SERVO_TIMER_PERIOD;
                     paramB1.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_DISABLE;
                     paramB1.captureCompareInterruptEnable_CCR0_CCIE =
                     TIMER_B_CCIE_CCR0_INTERRUPT_ENABLE;
                     paramB1.timerClear = TIMER_B_DO_CLEAR;
                     paramB1.startTimer = true;
                     Timer_B_initUpMode(TIMER_B1_BASE, &paramB1);

                     // konfiguracja CCR1 servo1

                     paramB1CCR1.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_1;
                     paramB1CCR1.compareInterruptEnable =
                     TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE;
                     paramB1CCR1.compareOutputMode = TIMER_B_OUTPUTMODE_RESET_SET;
                     paramB1CCR1.compareValue = 1500;
                     Timer_B_initCompareMode(TIMER_B1_BASE, &paramB1CCR1);

                     //konfiguracja CCR2 servo2

                     paramB1CCR2.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_2;
                     paramB1CCR2.compareInterruptEnable =
                     TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE;
                     paramB1CCR2.compareOutputMode = TIMER_B_OUTPUTMODE_RESET_SET;
                     paramB1CCR2.compareValue = 1500;
                     Timer_B_initCompareMode(TIMER_B1_BASE, &paramB1CCR2);

                     timerwlaczony = 1;

        }




                uint16_t dana1 = (servo1 * 11) + 500;

                paramB1CCR1.compareValue = dana1;
                Timer_B_initCompareMode(TIMER_B1_BASE, &paramB1CCR1);




                uint16_t dana2 = (servo2 * 11) + 500;

                paramB1CCR2.compareValue = dana2;
                Timer_B_initCompareMode(TIMER_B1_BASE, &paramB1CCR2);


         servo1p = servo1;
         servo2p = servo2;

    }
}

void configureIO()
{
    //konfiguracja wyjœcia do obs³ugi LED i BUZZERA
    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN2);

    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN3);
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN3);

    //Serwa
    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN1);

    //wyjscia timerow do sterowania silnikami
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,
                                                GPIO_PIN2 + GPIO_PIN3,
                                                GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,
        GPIO_PIN4 + GPIO_PIN5,
                                                    GPIO_PRIMARY_MODULE_FUNCTION);

    //timer A0 odpowiada za reazliowanie pomiarów odleg³oœci
    // czas trwania pomiaru czterech odleg³oœci bêdzie zajmowa³ 200 ms
    // na porcie P1.0 bêdzie realizowany triger za pomoc¹ CCR1 dodatkowo obs³ugiwane bêdzie przerwanie od CCR0
        GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0,
        GPIO_PRIMARY_MODULE_FUNCTION);


    //I2C
    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P1,
    GPIO_PIN6 + GPIO_PIN7,
    GPIO_SECONDARY_MODULE_FUNCTION);

    //konfiguracja portów p2.0 - 2 i p2.7 jako wejœcia
    GPIO_setAsInputPin(GPIO_PORT_P2,
    GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
    GPIO_selectInterruptEdge(GPIO_PORT_P2,
    GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7,
                             GPIO_LOW_TO_HIGH_TRANSITION);

    //konfiguracja portów do obs³ugi enkoderów
    GPIO_setAsInputPin(GPIO_PORT_P3,
                       GPIO_PIN2 + GPIO_PIN3 + GPIO_PIN4 + GPIO_PIN5 + GPIO_PIN6 + GPIO_PIN7); // ustawienie wejœcia enkodera silnika 1
    GPIO_selectInterruptEdge(GPIO_PORT_P3, GPIO_PIN3 + GPIO_PIN5 + GPIO_PIN7,
                             GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_P3, GPIO_PIN3 + GPIO_PIN5 + GPIO_PIN7);
    GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN3 + GPIO_PIN5 + GPIO_PIN7);

    GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN0 + GPIO_PIN1); // ustawienie wejœcia enkodera silnika 1
    GPIO_selectInterruptEdge(GPIO_PORT_P4, GPIO_PIN1,
    GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_P4, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN1);

    //konfiguracja portów do obs³ugi ADC
    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P1,GPIO_PIN1,GPIO_TERNARY_MODULE_FUNCTION);

    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P2,GPIO_PIN3,GPIO_TERNARY_MODULE_FUNCTION);

    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P2,GPIO_PIN4,GPIO_TERNARY_MODULE_FUNCTION);

    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P3,GPIO_PIN0,GPIO_TERNARY_MODULE_FUNCTION);

    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P3,GPIO_PIN1,GPIO_TERNARY_MODULE_FUNCTION);

}

void configureADC()
{
    ADC10CTL0 = ADC10SHT_6 + ADC10MSC + ADC10ON; // 16ADCclks, MSC, ADC ON
    ADC10CTL1 = ADC10SHP + ADC10CONSEQ_1; // sampling timer, s/w trig.,single sequence
    ADC10CTL2 |= ADC10RES;                   // 8-bit resolution
    ADC10MCTL0 = ADC10INCH_13;                 //

    // Configure DMA0 (ADC10IFG trigger)
    DMACTL0 = DMA0TSEL__ADC10IFG;             // ADC10IFG trigger
    __data16_write_addr((unsigned short) &DMA0SA, (unsigned long) &ADC10MEM0);
    // Source single address
    __data16_write_addr((unsigned short) &DMA0DA, (unsigned long) &adcpomiar[0]);
    // Destination array address
    DMA0SZ = 0x0e;                            // 14 conversions
    DMA0CTL = DMADT_4 + DMADSTINCR_3+ DMAEN + DMAIE;
    // Rpt, inc dest, byte access,
    // enable int after seq of convs

    while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
    ADC10CTL0 |= ADC10ENC + ADC10SC;        // Sampling and conversion start

}

uint16_t napbat()
{
    uint16_t napbat = 0;
    napbat = adcwyniki[0] / MNOZNIKNAPB;
    return napbat;
}

void configureCS()
{   //ustawienia systemu zegarowego
    GPIO_setAsPeripheralModuleFunctionInputPin(
        GPIO_PORT_PJ,
        GPIO_PIN4 + GPIO_PIN5,
        GPIO_PRIMARY_MODULE_FUNCTION
    );

    CS_setDCOFreq(CS_DCORSEL_1, CS_DCOFSEL_3);   //ustawienie DCO na 24MHZ
    CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); //MCLK 24MHZ


    CS_setExternalClockSource(16000000,0);
    CS_turnOnXT1(CS_XT1_DRIVE_3);

    CS_initClockSignal(CS_SMCLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_16); //SMCLK 1MHZ
    CS_initClockSignal(CS_ACLK,CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_16);  //ACLK 1MHZ


}

void configureI2C()
{

    EUSCI_B_I2C_initMasterParam param = { 0 };
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS;
    param.byteCounterThreshold = 1;
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

    EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param); // inicjalizacji I2C w master

    EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, SLAVE_ADDRESS); // ustawienie adresu sleava

    EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

    EUSCI_B_I2C_enable(EUSCI_B0_BASE);

}

void conigureUART() // uart na wartosciach domyslnych z predkoscia 115200
{
    GPIO_setAsPeripheralModuleFunctionInputPin(
                                               GPIO_PORT_P2,
                                               GPIO_PIN5 + GPIO_PIN6,
                                               GPIO_SECONDARY_MODULE_FUNCTION);

    // Configure UART
    EUSCI_A_UART_initParam paramUART = { 0 };
    paramUART.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_ACLK;
    paramUART.clockPrescalar = 8;
    paramUART.firstModReg = 0;
    paramUART.secondModReg = 0xD6;
    paramUART.parity = EUSCI_A_UART_NO_PARITY;
    paramUART.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    paramUART.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    paramUART.uartMode = EUSCI_A_UART_MODE;
    paramUART.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;

    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A1_BASE, &paramUART))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A1_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A1_BASE,
                                EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable USCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A1_BASE,
                                 EUSCI_A_UART_RECEIVE_INTERRUPT);                     // Enable interrupt

}

void cofigureTimers()
{
    //Do generacji PWM dla silnika 1 oraz 2, wykorzystano TIMER A1 oraz jego CCR1 i CCR2
    //Timer odlicza do 1000 cykli co daje nam 20 ms okresu i czestotliowsc 50 Hz
    // Na poczatku ustawiamy P1.2 i P1.3 jako wyjœcia tajmera w trybie reset_set.

    //konfiguracja Timera A1
    Timer_A_initUpModeParam paramA1 = { 0 };
    paramA1.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    paramA1.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    paramA1.timerPeriod = TIMER_PERIOD;
    paramA1.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    paramA1.captureCompareInterruptEnable_CCR0_CCIE =
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
    paramA1.timerClear = TIMER_A_DO_CLEAR;
    paramA1.startTimer = true;
    Timer_A_initUpMode(TIMER_A1_BASE, &paramA1);

    // konfiguracja ccR1 A1
    paramA1CCR1.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    paramA1CCR1.compareInterruptEnable =
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
    paramA1CCR1.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    paramA1CCR1.compareValue = 0;
    Timer_A_initCompareMode(TIMER_A1_BASE, &paramA1CCR1);

    // konfiguracja ccR2 A1
    paramA1CCR2.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;
    paramA1CCR2.compareInterruptEnable =
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
    paramA1CCR2.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    paramA1CCR2.compareValue = 0;
    Timer_A_initCompareMode(TIMER_A1_BASE, &paramA1CCR2);

    // Do generacji PWM dla silnika 3 i 4 u¿yto timera B0 oraz rejestrów CCR1 i CCR2
    //na poczatku ustawiono wyjœcia na p1.4 oraz p1.5

    //Ustawienia TIMERA B0
    Timer_B_initUpModeParam paramB0 = { 0 };
    paramB0.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
    paramB0.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_1;
    paramB0.timerPeriod = TIMER_PERIOD;
    paramB0.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_DISABLE;
    paramB0.captureCompareInterruptEnable_CCR0_CCIE =
    TIMER_B_CCIE_CCR0_INTERRUPT_DISABLE;
    paramB0.timerClear = TIMER_B_DO_CLEAR;
    paramB0.startTimer = true;
    Timer_B_initUpMode(TIMER_B0_BASE, &paramB0);

    // konfiguracja CCR1

    paramB0CCR1.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_1;
    paramB0CCR1.compareInterruptEnable =
    TIMER_B_CAPTURECOMPARE_INTERRUPT_DISABLE;
    paramB0CCR1.compareOutputMode = TIMER_B_OUTPUTMODE_RESET_SET;
    paramB0CCR1.compareValue = 0;
    Timer_B_initCompareMode(TIMER_B0_BASE, &paramB0CCR1);

    //konfiguracja CCR2

    paramB0CCR2.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_2;
    paramB0CCR2.compareInterruptEnable =
    TIMER_B_CAPTURECOMPARE_INTERRUPT_DISABLE;
    paramB0CCR2.compareOutputMode = TIMER_B_OUTPUTMODE_RESET_SET;
    paramB0CCR2.compareValue = 0;
    Timer_B_initCompareMode(TIMER_B0_BASE, &paramB0CCR2);

    //timer A0 odpowiada za reazliowanie pomiarów odleg³oœci
    // czas trwania pomiaru czterech odleg³oœci bêdzie zajmowa³ 200 ms
    // na porcie P1.0 bêdzie realizowany triger za pomoc¹ CCR1 dodatkowo obs³ugiwane bêdzie przerwanie od CCR0

    //konfiguracja Timera A0
    Timer_A_initUpModeParam paramA0 = { 0 };
    paramA0.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    paramA0.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    paramA0.timerPeriod = CZUJ_TIMER_PERIOD;
    paramA0.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
    paramA0.captureCompareInterruptEnable_CCR0_CCIE =
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    paramA0.timerClear = TIMER_A_DO_CLEAR;
    paramA0.startTimer = true;
    Timer_A_initUpMode(TIMER_A0_BASE, &paramA0);

    // konfiguracja ccR1 A1
    Timer_A_initCompareModeParam paramA0CCR1 = { 0 };
    paramA0CCR1.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    paramA0CCR1.compareInterruptEnable =
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE;
    paramA0CCR1.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    paramA0CCR1.compareValue = 20; // dlugoœæ impulsu Trigger do ultrasonic sensor
    Timer_A_initCompareMode(TIMER_A0_BASE, &paramA0CCR1);


    /*//ustawienie tajmera B1, który bêdzie sluzyl do sterowania predkoscia serw
    //Ustawienia TIMERA B1
    Timer_B_initUpModeParam paramB1 = { 0 };
    paramB1.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
    paramB1.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_1;
    paramB1.timerPeriod = SERVO_TIMER_PERIOD;
    paramB1.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_DISABLE;
    paramB1.captureCompareInterruptEnable_CCR0_CCIE =
    TIMER_B_CCIE_CCR0_INTERRUPT_ENABLE;
    paramB1.timerClear = TIMER_B_DO_CLEAR;
    paramB1.startTimer = true;
    Timer_B_initUpMode(TIMER_B1_BASE, &paramB1);

    // konfiguracja CCR1 servo1

    paramB1CCR1.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_1;
    paramB1CCR1.compareInterruptEnable =
    TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE;
    paramB1CCR1.compareOutputMode = TIMER_B_OUTPUTMODE_RESET_SET;
    paramB1CCR1.compareValue = 1500;
    Timer_B_initCompareMode(TIMER_B1_BASE, &paramB1CCR1);

    //konfiguracja CCR2 servo2

    paramB1CCR2.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_2;
    paramB1CCR2.compareInterruptEnable =
    TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE;
    paramB1CCR2.compareOutputMode = TIMER_B_OUTPUTMODE_RESET_SET;
    paramB1CCR2.compareValue = 1500;
    Timer_B_initCompareMode(TIMER_B1_BASE, &paramB1CCR2);*/

    //ustawienie tajmera B2 do wyzwalania sterowania silnikami
    Timer_B_initUpModeParam paramB2 = { 0 };
    paramB2.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
    paramB2.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_2;
    paramB2.timerPeriod = DC_TIMER_PERIOD;
    paramB2.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_DISABLE;
    paramB2.captureCompareInterruptEnable_CCR0_CCIE =
    TIMER_B_CCIE_CCR0_INTERRUPT_ENABLE;
    paramB2.timerClear = TIMER_B_DO_CLEAR;
    paramB2.startTimer = true;
    Timer_B_initUpMode(TIMER_B2_BASE, &paramB2);


}

// funkcja któr¹ definujemy kierunek oraz prêdkosci silników
// dwa silniki po lewej stronie traktowane bêd¹ jako jeden, jak równie¿ dwa po prawej jako jeden.
// kl - kierunek lewy oznacza w która stronê ma obracac sie silnik lewy 1-do przodu 2 do ty³u 0-swobodny , to samo tyczy prawego silnika
void DCmotor(uint8_t kla, uint8_t kpa, uint16_t pla, uint16_t ppa)
{
    //jesli napiecie zasilania jest zbyt male silniki nie pojada wogóle

    napbate = napbat();
    if (napbate < MINBAT)
    {
        kl = 0;
        kp = 0;
        pl = 0;
        pp = 0;

    }
    else
    {
        kl = kla;
        kp = kpa;
        pl = pla;
        pp = ppa;
        if(kl==0)
        {
            pl=0;

        }
        if(kp==0)
        {
            pp=0;
        }


    }

}

void DCmotor_time(uint8_t klf, uint8_t kpf, uint16_t plf, uint16_t ppf)
{
    if (klf != klp || kpf != kpp)
    {

        uint8_t rejestrl = 0;
        uint8_t rejestrp = 0;
        if (klf == 2)
        {
            rejestrl = 0b00001010;
        }
        else if (klf == 1)
        {
            rejestrl = 0b00000101;
        }
        else if (klf == 0)
        {
            rejestrl = 0b00001111;
        }

        if (kpf == 1)
        {
            rejestrp = 0b10100000;
        }
        else if (kpf == 2)
        {
            rejestrp = 0b01010000;
        }
        else if (kpf == 0)
        {
            rejestrp = 0b11110000;
        }

        kieruneksilnikow = rejestrp | rejestrl;

        EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, kieruneksilnikow); //wysy³amy dane do IO ekspandera, ¿eby ustawic wyjscie na mostku
        while (EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
        klp = klf;
        kpp = kpf;
    }



  // jesli odleglosc odczytana z czujnikow jest mniejsza niz 10cm robot nie pojedzie do tylu lub do przodu
    if( klf == 1 && kpf == 1 && czujniki_cm[1]<10)
    {
        sygnal_S1=0;
        sygnal_S2=0;
        sygnal_S3=0;
        sygnal_S4=0;
        Ca_S1 = 0;
        Ca_S2 = 0;
        Ca_S3 = 0;
        Ca_S4 = 0;

    }
    else if( klf == 2 && kpf == 2 && czujniki_cm[3]<10)
    {
        sygnal_S1=0;
        sygnal_S2=0;
        sygnal_S3=0;
        sygnal_S4=0;
        Ca_S1 = 0;
        Ca_S2 = 0;
        Ca_S3 = 0;
        Ca_S4 = 0;
    }
    else // jesli jedzie to tylko z predkoscia wyliczona za pomoca PID
    {
        // kod realizujacy PID dla obslugi 4 silnikow
            uchyb_S1 =  (int)plf   -  (int) predkosc[0];

            Ca_S1 = Ca_S1 + (((uchyb_S1 + uchybp_S1) / 2) * 0.1);

            sygnal_S1 = kpPID * uchyb_S1 + kiPID * Ca_S1 - kdPID * (uchyb_S1 - uchybp_S1);

            if (sygnal_S1 > 1000)
            {
                sygnal_S1 = 1000;
            }
            if (sygnal_S1 < 0)
            {
                sygnal_S1 = 0;
            }
            if (Ca_S1 > 100)
                       {
                           Ca_S1 = 100;
                       }
                       if (Ca_S1 < -100)
                       {
                           Ca_S1 = -100;
                       }

            uchyb_S2 = (int) plf -  (int) predkosc[1];

            Ca_S2 = Ca_S2 + (((uchyb_S2 + uchybp_S2) / 2) * 0.1);

            sygnal_S2 = kpPID * uchyb_S2 + kiPID * Ca_S2 - kdPID * (uchyb_S2 - uchybp_S2);

            if (sygnal_S2 > 1000)
            {
                sygnal_S2 = 1000;
            }
            if (sygnal_S2 < 0)
            {
                sygnal_S2 = 0;
            }
            if (Ca_S2 > 100)
                                 {
                                       Ca_S2 = 100;
                                   }
                                   if (Ca_S2 < -100)
                                   {
                                       Ca_S2 = -100;
                                   }

            uchyb_S3 = (int) ppf -  (int) predkosc[2];

            Ca_S3 = Ca_S3 + (((uchyb_S3 + uchybp_S3) / 2) * 0.1);

            sygnal_S3 = kpPID * uchyb_S3 + kiPID * Ca_S3 - kdPID * (uchyb_S3 - uchybp_S3);

            if (sygnal_S3 > 1000)
            {
                sygnal_S3 = 1000;
            }
            if (sygnal_S3 < 0)
            {
                sygnal_S3 = 0;
            }
            if (Ca_S3 > 100)
                                   {
                                       Ca_S3 = 100;
                                   }
                                   if (Ca_S3 < -100)
                                   {
                                       Ca_S3 = -100;
                                   }

            uchyb_S4 = (int) ppf - (int) predkosc[3];

            Ca_S4 = Ca_S4 + (((uchyb_S4 + uchybp_S4) / 2) * 0.1);

            sygnal_S4 = kpPID * uchyb_S4 + kiPID * Ca_S4 - kdPID * (uchyb_S4 - uchybp_S4);

            if (sygnal_S4 > 1000)
            {
                sygnal_S4 = 1000;
            }
            if (sygnal_S4 < 0)
            {
                sygnal_S4 = 0;
            }
            if (Ca_S4 > 100)
                                   {
                                       Ca_S4 = 100;
                                   }
                                   if (Ca_S4 < -100)
                                   {
                                       Ca_S4 = -100;
                                   }

    }

//Lewa strona
    //paramA1CCR1.compareValue = sygnal_S1;
    Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, sygnal_S1);
    //paramA1CCR2.compareValue = sygnal_S2;
    Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, sygnal_S2);;
// prawa strona
    //paramB0CCR1.compareValue = sygnal_S3;
    Timer_B_setCompareValue(TIMER_B0_BASE, TIMER_B_CAPTURECOMPARE_REGISTER_1 , sygnal_S3);
    //paramB0CCR2.compareValue = sygnal_S4;
    Timer_B_setCompareValue(TIMER_B0_BASE, TIMER_B_CAPTURECOMPARE_REGISTER_2 , sygnal_S4);

}


// przerwanie nastepuje co 50ms
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER0_A0_VECTOR)))
#endif
void TIMER0_A0_ISR(void)
{

    switch (nrczujnika)
    {
    case 0:
        GPIO_disableInterrupt(GPIO_PORT_P2,
        GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
        GPIO_clearInterrupt(GPIO_PORT_P2,
        GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
        GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN0);
        nrczujnika = 1;
        break;
    case 1:
        GPIO_disableInterrupt(GPIO_PORT_P2,
        GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
        GPIO_clearInterrupt(GPIO_PORT_P2,
        GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
        GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN1);
        nrczujnika = 2;
        break;
    case 2:
        GPIO_disableInterrupt(GPIO_PORT_P2,
        GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
        GPIO_clearInterrupt(GPIO_PORT_P2,
        GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
        GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN2);
        nrczujnika = 3;
        break;
    case 3:
        GPIO_disableInterrupt(GPIO_PORT_P2,
        GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
        GPIO_clearInterrupt(GPIO_PORT_P2,
        GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
        GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN7);
        nrczujnika = 0;
        break;
    default:
        break;

    }

}

// przerwanie nastepuje co 20ms
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_B0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER1_B0_VECTOR)))
#endif
void TIMER1_B0_ISR(void)
{

    GPIO_setOutputHighOnPin(GPIO_PORT_PJ, GPIO_PIN0 + GPIO_PIN1);

}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_B1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER1_B1_VECTOR)))
#endif
void TIMER1_B1_ISR(void)
{

    switch(__even_in_range(TB1IV, TB1IV_TBIFG))
      {
        case TB1IV_NONE:            break;
        case TB1IV_TBCCR1:
            GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN0);

            break;               // CCR1 interrupt
        case TB1IV_TBCCR2:
            GPIO_setOutputLowOnPin(GPIO_PORT_PJ,GPIO_PIN1);

            break;               // CCR2 interrupt
        case TB1IV_TBIFG:
            break;               // TBIFG interrupt
        default: break;
      }
    znacznik_serw ++;
    if (znacznik_serw>40)
       {
           znacznik_serw=0;
           //zatrzymujemy timer
           Timer_B_stop(TIMER_B1_BASE);
           Timer_B_clear(TIMER_B1_BASE);
           timerwlaczony=0;
       }

}

//przerwanie nastepuje co 50ms
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER2_B0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER1_B0_VECTOR)))
#endif
void TIMER2_B0_ISR(void)
{
    DCmotor_time(kl, kp, pl, pp); //wyzwalacz strerowania silnikow


    // tutaj znajduje siê kod który wylicza prêdkoœci silnikow
    float temp;
    int j = 0;

    for (j = 0; j < 4; j++)
    {
        temp = (long)impulsy[j]*1000 / IMPNAOB;
        obroty[j] = obrotytemp[j]*1000 + temp;
        obrsub[j] = obroty[j] - obrp[j];
        obrp[j] = obroty[j];
        predkosc[j] = obrsub[j] * 6.144; //  /1000  * 6185 predkosc jest wartoscia
        if (predkosc[j] < 0)
        {
            predkosc[j] = -predkosc[j];
        }
    }

    //DCmotor_time(kl, kp, pl, pp); //wyzwalacz strerowania silnikow
    while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active

       ADC10CTL0 |= ADC10ENC + ADC10SC;        // Sampling and conversion start

}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT2_VECTOR)))
#endif
void Port_2(void)
{

    switch (__even_in_range(P2IV, 16))
    {
    case 0:
        break;
    case 2:
        if (n == 0)
        {
            czas = Timer_A_getCounterValue(TIMER_A0_BASE);
            GPIO_clearInterrupt(GPIO_PORT_P2,
            GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
            GPIO_selectInterruptEdge(
                    GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7,
                    GPIO_HIGH_TO_LOW_TRANSITION);
            n = 1;
        }
        else if (n == 1)
        {
            czujniki[0] = Timer_A_getCounterValue(TIMER_A0_BASE) - czas;
            czujniki_cm[0]= czujniki[0]/58;
            GPIO_clearInterrupt(GPIO_PORT_P2,
            GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
            GPIO_selectInterruptEdge(
                    GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7,
                    GPIO_LOW_TO_HIGH_TRANSITION);
            n = 0;

        }
        break;
    case 4:
        if (n == 0)
        {
            czas = Timer_A_getCounterValue(TIMER_A0_BASE);
            GPIO_clearInterrupt(GPIO_PORT_P2,
            GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
            GPIO_selectInterruptEdge(
                    GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7,
                    GPIO_HIGH_TO_LOW_TRANSITION);
            n = 1;
        }
        else if (n == 1)
        {
            czujniki[1] = Timer_A_getCounterValue(TIMER_A0_BASE) - czas;
            czujniki_cm[1]=   czujniki[1]/58;
            GPIO_clearInterrupt(GPIO_PORT_P2,
            GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
            GPIO_selectInterruptEdge(
                    GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7,
                    GPIO_LOW_TO_HIGH_TRANSITION);
            n = 0;
        }
        break;
    case 6:
        if (n == 0)
        {
            czas = Timer_A_getCounterValue(TIMER_A0_BASE);
            GPIO_clearInterrupt(GPIO_PORT_P2,
            GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
            GPIO_selectInterruptEdge(
                    GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7,
                    GPIO_HIGH_TO_LOW_TRANSITION);
            n = 1;
        }
        else if (n == 1)
        {
            czujniki[2] = Timer_A_getCounterValue(TIMER_A0_BASE) - czas;
            czujniki_cm[2]= czujniki[2]/58;
            GPIO_clearInterrupt(GPIO_PORT_P2,
            GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
            GPIO_selectInterruptEdge(
                    GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7,
                    GPIO_LOW_TO_HIGH_TRANSITION);
            n = 0;
        }
        break;
    case 8:
        break;
    case 10:
        break;
    case 12:
        break;
    case 14:
        break;
    case 16:
        if (n == 0)
        {
            czas = Timer_A_getCounterValue(TIMER_A0_BASE);
            GPIO_clearInterrupt(GPIO_PORT_P2,
            GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
            GPIO_selectInterruptEdge(
                    GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7,
                    GPIO_HIGH_TO_LOW_TRANSITION);
            n = 1;
        }
        else if (n == 1)
        {
            czujniki[3] = Timer_A_getCounterValue(TIMER_A0_BASE) - czas;
            czujniki_cm[3]= czujniki[3]/58;
            GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7);
            GPIO_selectInterruptEdge(
                    GPIO_PORT_P2, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN7,
                    GPIO_LOW_TO_HIGH_TRANSITION);
            n = 0;
        }
        break;
    default:
        break;

    }

}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT3_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT3_VECTOR)))
#endif
void Port_3(void)
{

    switch (__even_in_range(P3IV, 16))
    {
    case 0:
        break;
    case 2:
        break;
    case 4:
        break;
    case 6:
        break;
    case 8:
        if (GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN2) == GPIO_INPUT_PIN_LOW)
        {
            impulsy[0] = impulsy[0] + 1;
            if (impulsy[0] >= IMPNAOB)
            {
                obrotytemp[0] = obrotytemp[0] + 1;
                impulsy[0] = 0;
            }

        }
        else
        {
            impulsy[0] = impulsy[0] - 1;
            if (impulsy[0] <= -IMPNAOB)
            {
                obrotytemp[0] = obrotytemp[0] - 1;
                impulsy[0] = 0;
            }
        }

        GPIO_clearInterrupt(GPIO_PORT_P3, GPIO_PIN3);
        break;
    case 10:
        break;
    case 12:
        if (GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN4) == GPIO_INPUT_PIN_LOW)
        {
            impulsy[1] = impulsy[1] + 1;
            if (impulsy[1] >= IMPNAOB)
            {
                obrotytemp[1] = obrotytemp[1] + 1;
                impulsy[1] = 0;
            }

        }
        else
        {
            impulsy[1] = impulsy[1] - 1;
            if (impulsy[1] <= -IMPNAOB)
            {
                obrotytemp[1] = obrotytemp[1] - 1;
                impulsy[1] = 0;
            }
        }

        GPIO_clearInterrupt(GPIO_PORT_P3, GPIO_PIN5);

        break;
    case 14:
        break;
    case 16:
        if (GPIO_getInputPinValue(GPIO_PORT_P3,
        GPIO_PIN6) == GPIO_INPUT_PIN_HIGH)
        {
            impulsy[2] = impulsy[2] + 1;
            if (impulsy[2] >= IMPNAOB)
            {
                obrotytemp[2] = obrotytemp[2] + 1;
                impulsy[2] = 0;
            }

        }
        else
        {
            impulsy[2] = impulsy[2] - 1;
            if (impulsy[2] <= -IMPNAOB)
            {
                obrotytemp[2] = obrotytemp[2] - 1;
                impulsy[2] = 0;
            }
        }

        GPIO_clearInterrupt(GPIO_PORT_P3, GPIO_PIN7);
        break;
    default:
        break;

    }

}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT4_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(PORT4_VECTOR)))
#endif
void Port_4(void)
{

    switch (__even_in_range(P4IV, 16))
    {
    case 0:
        break;
    case 2:
        break;
    case 4:
        if (GPIO_getInputPinValue(GPIO_PORT_P4,
        GPIO_PIN0) == GPIO_INPUT_PIN_HIGH)
        {
            impulsy[3] = impulsy[3] + 1;
            if (impulsy[3] >= IMPNAOB)
            {
                obrotytemp[3] = obrotytemp[3] + 1;
                impulsy[3] = 0;
            }

        }
        else
        {
            impulsy[3] = impulsy[3] - 1;
            if (impulsy[3] <= -IMPNAOB)
            {
                obrotytemp[3] = obrotytemp[3] - 1;
                impulsy[3] = 0;
            }
        }

        GPIO_clearInterrupt(GPIO_PORT_P4, GPIO_PIN1);
        break;
    case 6:
        break;
    case 8:
        break;
    case 10:
        break;
    case 12:
        break;
    case 14:
        break;
    case 16:
        break;
    default:
        break;

    }

}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=DMA_VECTOR
__interrupt void DMA0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(DMA_VECTOR))) DMA0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(DMAIV,16))
  {
    case  0: break;                          // No interrupt
    case 2:
        adcwyniki[0] = adcpomiar[12];
        adcwyniki[1] = adcpomiar[7];
        adcwyniki[2] = adcpomiar[6];
        adcwyniki[3] = adcpomiar[1];
        adcwyniki[4] = adcpomiar[0];


        break;                                 // DMA0IFG
    case  4: break;                          // DMA1IFG
    case  6: break;                          // DMA2IFG
    case  8: break;                          // Reserved
    case 10: break;                          // Reserved
    case 12: break;                          // Reserved
    case 14: break;                          // Reserved
    case 16: break;                          // Reserved
    default: break;
  }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A1_VECTOR)))
#endif
void EUSCI_A1_ISR(void)
{
    switch (__even_in_range(UCA1IV, USCI_UART_UCTXCPTIFG))
    {
    case USCI_NONE:
        break;

        // obs³uga przerwania odpowiadajaca za odbieranie danych z UART
    case USCI_UART_UCRXIFG:
        RXData = EUSCI_A_UART_receiveData(EUSCI_A1_BASE);

        if (zapis == 0)
        {
            if ((RXData == 'a' && m == 0))                   // Check value
            {
                ramka_RX[m++] = RXData;
                zapis = 1;
            }

        }
        else
        {
            ramka_RX[m++] = RXData;
            if (m == ((sizeof ramka_RX)))
            {
                zapis = 0;
                m = 0;
                //tutaj ustawiam zmienne steruj¹ce silnikami i serva

                pl_temp = (ramka_RX[3] << 8) | (ramka_RX[4]);
                pp_temp = (ramka_RX[5] << 8) | (ramka_RX[6]);
                DCmotor(ramka_RX[1], ramka_RX[2], pl_temp, pp_temp);

                setservos(ramka_RX[7], ramka_RX[8]);
                switch (ramka_RX[9])
                {
                case 0:
                    setled(0);
                    setbuzzer(0);
                    break;
                case 1:
                    setled(1);
                    setbuzzer(0);
                    break;
                case 2:
                    setled(0);
                    setbuzzer(1);
                    break;
                case 3:
                    setled(1);
                    setbuzzer(1);
                    break;
                default:
                    setled(0);
                    setbuzzer(0);
                    break;
                }

            }

        }


      break;



    //obs³uga przerywania odpowiadajaca za wysylanie danych po UART
    case USCI_UART_UCTXIFG:
        EUSCI_A_UART_transmitData(EUSCI_A1_BASE,
                                           ramka_TX[l++]);

        if(l==((sizeof ramka_TX)))
        {
            EUSCI_A_UART_disableInterrupt(EUSCI_A1_BASE,
                                          EUSCI_A_UART_TRANSMIT_INTERRUPT);                     // DISable interrupt

        }

        break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
  }
}
