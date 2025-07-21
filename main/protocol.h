

//Este fichero contiene las definiciones necesarias para soportar el prootocolo entre el microcontolador y el PC.

#ifndef __protocol_H__
#define __protocol_H__

#include<stdint.h>


typedef enum {COMANDO_PING,COMANDO_LED,COMANDO_SONDEA_BOTONES,COMANDO_BOTONES_ASYNC,COMANDO_PWM_LEDS,COMANDO_ADC_START,COMANDO_ADC_STOP,COMANDO_ADC_DATA,COMANDO_DAC_VALUE,COMANDO_CLOCK_SET,COMANDO_MODO,COMANDO_RECHAZADO} TipoComando;


#pragma pack(1)

typedef struct {
    uint32_t bit0:1;
    uint32_t bit1:1;
    uint32_t bit2:1;
    uint32_t bit3:1;
    uint32_t bit4:1;
    uint32_t bit5:1;
    uint32_t bit6:1;
    uint32_t bit7:1;
    uint32_t bit8:1;
    uint32_t bit9:1;
    uint32_t bit10:1;
    uint32_t bit11:1;
    uint32_t bit12:1;
    uint32_t bit13:1;
    uint32_t bit14:1;
    uint32_t bit15:1;
    uint32_t bit16:1;
    uint32_t bit17:1;
    uint32_t bit18:1;
    uint32_t bit19:1;
    uint32_t bit20:1;
    uint32_t bit21:1;
    uint32_t bit22:1;
    uint32_t bit23:1;
    uint32_t bit24:1;
    uint32_t bit25:1;
    uint32_t bit26:1;
    uint32_t bit27:1;
    uint32_t bit28:1;
    uint32_t bit29:1;
    uint32_t bit30:1;
    uint32_t bit31:1;
} bitfield;

typedef struct {
    int16_t channel0;
} adcData;

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} pwmData;

typedef struct {
    uint16_t u16value0;
    uint16_t u16value1;
} twoWords;


typedef struct {
    uint8_t comando;
    union {	/* Ojo, que esto es una UNION y no una estructura */
        twoWords words;
        uint16_t value;
        uint32_t u32value;
        float fvalue;
        bitfield bits;
        adcData adc;
        pwmData pwm;
     } parametro;
} TipoPaquete;


#pragma pack()


#endif  /* ndef __protocol_H__ */
