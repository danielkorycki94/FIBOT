/*
 * library.h
 *
 *  Created on: 18.12.2018
 *      Author: danie
 */

#ifndef LIBRARY_H_
#define LIBRARY_H_

#include "driverlib.h"

#define SLAVE_ADDRESS 0x20 //adres modu³u rozszerzen po I2C odpowiadaj¹cego za sterowanie mostkami h
#define TIMER_PERIOD 1000 // reguluje czêstotliwoœc pwm na silniki
#define CZUJ_TIMER_PERIOD 62000 //co tyle us uruchamiany jest pomiar odleglosci za pomoca czujnikow
#define SERVO_TIMER_PERIOD 20000 // sterowanie serwomechanizmami
#define DC_TIMER_PERIOD 49000 // co tyle us uaktualniana jest predkosc i kierunek silnikow
#define IMPNAOB 898     // ilosc impulsow enkodera na 1 obrót ko³a
#define MNOZNIKNAPB    1.1342 // s³u¿y do wyliczenia napiêcia na bateri x100
// #define MNOZNIKNAPS     1.47  //
// #define MNOZNIKSYG      12
#define MNOZNIKNAPC   2.58064 // przelicza napiecie z adc na prad
#define MINBAT 650   // minimalne napiecie baterii 7.5 x100


void setbuzzer(uint8_t stan); // 1 = w³¹czony, 0 = wy³aczony
void setled(uint8_t stan); // 1 = w³¹czony, 0 = wy³aczony
void configureIO();
void configureADC();
void configureCS();
void configureI2C();
void conigureUART();
void cofigureTimers();
void DCmotor_time(uint8_t klf, uint8_t kpf, uint16_t plf, uint16_t ppf);
void DCmotor(uint8_t kla, uint8_t kpa, uint16_t pla, uint16_t ppa );
void setservos(uint8_t serwo1, uint8_t servo2);
void start_UART_TX();

uint16_t napbat();

uint16_t * daneczujniki();

int * getspeed();

void calibrateDCcurrent();

long * getcurrent();


#endif /* LIBRARY_H_ */
