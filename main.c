// (入力PC0,PC1)AD変換した値(10bit)をそれぞれi2cスレーブで返す(2+2byte)。
// 2022-04-06 by Penkich
//
#define    F_CPU    12000000UL // 12MHz

#include <util/twi.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "I2CSlave.h"
// SDA:PC4, SCL:PC5 

#define sbi(BYTE,BIT) BYTE|=_BV(BIT) // BYTEの指定BITに1をセット
#define cbi(BYTE,BIT) BYTE&=~_BV(BIT) // BYTEの指定BITをクリア

#define sw0 0
#define sw1 1
#define sw2 2
#define sw3 3

static void (*I2C_recv)(uint8_t);
static void (*I2C_req)();

uint8_t Count =0;

void port_init(void){
	PORTB = 0b000001;
	DDRB = 0x00;
	PORTC = 0x00;
	DDRC = 0x00;
	PORTD = 0x00;
	DDRD = 0xFF;
	// PD0=PCINT0 ピン変化割込設定
	//PCMSK0 |= 1 << PCINT0;         // ピン変化マスクレジスタにPCINT0を許可
	//PCICR |= 1 << PCIE0;            // ピン変化レジスタにPCINT0～PCINT7を許可
}

SIGNAL(PCINT0_vect)    // ピン変化割込処理
{
	_delay_ms(10);  // チャタリング10ms待ち
	
	if ( PINB & (1 << PINB0)) // スイッチSW0がOFFならば
	{
		PORTD &= ~( 1 << PORTD4);       // LED4を消灯
	}
	else  // スイッチSW0がON
	{
		PORTD |= 1 << PORTD4;   // LED4を点灯
		Count++;
	}
	_delay_ms(100);  // 100ms待ち
}

void I2C_setCallbacks(void (*recv)(uint8_t), void (*req)())
{
	I2C_recv = recv;
	I2C_req = req;
}

void I2C_init(uint8_t address)
{
	cli();
	// load address into TWI address register
	TWAR = address << 1;
	// set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
	TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
	sei();
}

void I2C_stop(void)
{
	// clear acknowledge and enable bits
	cli();
	TWCR = 0;
	TWAR = 0;
	sei();
}

ISR(TWI_vect)
{
	switch(TW_STATUS)
	{
		case TW_SR_DATA_ACK:
		    // received data from master, call the receive callback
		    I2C_recv(TWDR);
		    TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		    break;
		case TW_ST_SLA_ACK:
		    // master is requesting data, call the request callback
		    I2C_req();
		    TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		    break;
		case TW_ST_DATA_ACK:
		    // master is requesting data, call the request callback
		    I2C_req();
		    TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		    break;
		case TW_BUS_ERROR:
		    // some sort of erroneous state, prepare TWI to be readdressed
		    TWCR = 0;
		    TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		    break;
		default:
		    TWCR = (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
		    break;
	}
}


void adc_init(void){
	//ADMUX = (0<<REFS0); // 外部基準電圧
	ADMUX = 0x40; // AVcc, ADC0, 10bit 選択
	ADCSRA = (1<<ADEN)|(1<<ADSC)|(0<<ADPS2)|(0<<ADPS1);
}

#define I2C_ADDR 0x0a
volatile uint8_t data;

void I2C_received(uint8_t received_data)
{
	data = received_data;
}

volatile int cnt=0;
//volatile uint8_t data1H =0;
volatile uint8_t data1L =0;
//volatile uint8_t data2H =0;
volatile uint8_t data2L =0;
void I2C_requested()
{
	switch (cnt){
	    case 0:
			ADMUX = 0x40; // ADC0
			cbi(ADCSRA,ADIF);
			sbi(ADCSRA,ADSC); // 変換開始
			loop_until_bit_is_set(ADCSRA,ADIF); // 変換完了まで待つ
			//data1H = ADCH;
			data1L = ADCL;
        	I2C_transmitByte(ADCH);
		    cnt++;
		    break;
	    case 1:
	        I2C_transmitByte(data1L);	
	        cnt ++;
		    break;
	    case 2:
	        ADMUX = 0x41;  // ADC1
	        cbi(ADCSRA,ADIF);
	        sbi(ADCSRA,ADSC); // 変換開始
	        loop_until_bit_is_set(ADCSRA,ADIF); // 変換完了まで待つ
	        //data2H = ADCH;
	        data2L = ADCL;
	        I2C_transmitByte(ADCH);
	        cnt++;
	        break;
	    case 3:
	        I2C_transmitByte(data2L);
	        cnt =0;
	        break;
	}
}

void setup()
{
	// set received/requested callbacks
	I2C_setCallbacks(I2C_received, I2C_requested);

	// init I2C
	I2C_init(I2C_ADDR);
}

int main(){
	port_init();
	setup();
	adc_init(); // ADConverter設定
	// Main program loop
	while(1){
		;
	}
}
