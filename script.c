#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/EEPROM.h>
#include <stdbool.h>

#define F_CPU 8000000UL 

//LCD
#define LCD_E		PB0 //Pino que controla o enable do LCD
#define LCD_RS		PB1 //Pino que controla o R/S (Seleciona Registrador) do LCD

//Teclado
#define TEC 		PC0 //Pino que recebe as tensões dos botões que rpresentam o teclado

//LEDS
#define LED_ATIV	PC1 //Pino que controla Led que indica modo ativado 
#define LED_PROG	PC2 //Pino que controla Led que indica modo programação
#define LED_ZN1		PC3 //Pino que controla Led que indica modo programação
#define LED_ZN2		PC4 //Pino que controla Led que indica modo programação
#define LED_ZN3		PC5 //Pino que controla Led que indica modo programação

//ALARME
#define ALARME		PB6 //Pino que controla Led que indica modo programação

//RTC
#define MISO 		PB4 //
#define MOSI 		PB3 //
#define SCK 		PB5 //
#define SS 		PB2 //

//SENSORES

#define CLK_S     PD0
#define LOAD      PB7
#define SERIAL    PD3

//USART
#define CLK 2000000 // Clock
#define BAUD 9600                           // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)

#define set_bit(reg, bit) (reg |= (1<<bit)) //o bit do registrador indicado vai pra 1
#define clr_bit(reg, bit) (reg &= ~(1<<bit)) //o bit do registrador indicado vai pra 0

volatile int tempoR=0, timeoutt=0, tempoS, tempoA=0;
//----------------- LCD -------------------------

//Mantém o enable do LCD ligado pelo tempo necessário
   void habilitar_LCD()
   {    
       _delay_us(1); 
       set_bit(PORTB, LCD_E); 
       _delay_us(1); 
       clr_bit(PORTB, LCD_E); 
       _delay_us(45);
   }

   //recebe o comando de instrução ou escrita no LCD
   void cmd_LCD(unsigned char LCD_dados, char LCD_comando)
   {
    if(LCD_comando == 0) //0 se for isntrucao para o LCD
    clr_bit(PORTB, LCD_RS);
    else //1 se for dados para o LCD
    set_bit(PORTB, LCD_RS);
    
   //Envia os primeiros 4 bits de dados para o LCD pelos MSB do PORT
    PORTD = (PORTD & 0x0F)|(0xF0 & LCD_dados);
    
    habilitar_LCD();
    //Envia os 4 bits finais de dados para o LCD pelos MSB do PORT
    PORTD = (PORTD & 0x0F) | (0xF0 & (LCD_dados<<4));
    
    habilitar_LCD();
    if((LCD_comando==0) && (LCD_dados<4)) //se for instrução de retorno ou limpeza espera LCD estar pronto
    _delay_ms(2);
   }

//inicializa o display pra um funcionamento correto
   void inicializa_LCD()
   { 
    //o LCD será só escrito. Então, R/W é sempre zero.
    clr_bit(PORTB, LCD_RS);//RS em zero indicando que o dado para o LCD será uma instrução
    clr_bit(PORTB, LCD_E);//pino de habilitação em zero
    _delay_ms(20); /*tempo para estabilizar a tensão do LCD, após VCC
   ultrapassar 4.5 V (na prática pode ser maior).*/
    
    PORTD = (PORTD & 0x0F) | 0x20;
    habilitar_LCD();
    
    cmd_LCD(0x28,0); //interface de 4 bits 2 linhas (aqui se habilita as 2 linhas)
    //são enviados os 2 nibbles (0x2 e 0x8)
    cmd_LCD(0x08,0); //desliga o display
    cmd_LCD(0x01,0); //limpa todo o display
    cmd_LCD(0x0C,0); //mensagem aparente cursor inativo não piscando
    cmd_LCD(0x80,0); //inicializa cursor na primeira posição a esquerda - 1a linha
   }

   //funcao que auxilia no comando de escrita do LCD
   void escreve_LCD(const char *dados)
   {
    for (; *dados!=0; dados++) cmd_LCD(*dados, 1);
   }
   
//----------------------Teclado-------------------
int Teclado(){
	int adc_start();
	int conver = adc_start();
   
	if (conver<10)//1014
	return(17);
	else if (conver>1000 && conver<1024)//1014
	return(1);
	else if (conver>930 && conver<990)//962
	return(4);
	else if (conver>890 && conver<920)//901
	return(7);
	else if (conver>830 && conver<860)//840
	return(16);
	else if (conver>760 && conver<790)//778
	return(2);
	else if (conver>700 && conver<730)//717
	return(5);
	else if (conver>660 && conver<690)//671
	return(8);
	else if (conver>560 && conver<610)//594
	return(0);
	else if (conver>500 && conver<550)//533
	return(3);
	else if (conver>460 && conver<490)//471
	return(6);
	else if (conver>390 && conver<430)//410
	return(9);
	else if (conver>330 && conver<360)//348
	return(15);
	else if (conver>260 && conver<300)//287
	return(12);
	else if (conver>210 && conver<240)//225
	return(13);
	else if(conver>140 && conver<210)//164
	return(11);
	else if (conver>10 && conver<140)//64
	return(14);

}

//===================================TIMER==============================

void timer1_init()
{
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;

	//1 Hz (8000000/((31249+1)*256)) freq = FCPU/(prescaler)*(OCR1A+1)
	OCR1A = 3800;// 31249 Era pra ser isso....

	TCCR1B |= (1 << WGM12);//Habilita modo de comparação
	TCCR1B |= (1 << CS12); //Prescaler 256

	TIMSK1 |= (1 << OCIE1A); //Habilita Comparação com o OCR1A
	
	sei();
}

ISR (TIMER1_COMPA_vect)
{
	timeoutt++;
	tempoA++;
	tempoR++;
	tempoS++;
}
//===================================================================
//------------------Inicio do captura senha--------------------------

int capturaSenha(){
	int resultado;
	int tecla;
	
	       while (1) {
                tecla = Teclado();
		   if (tecla < 10){
		     _delay_ms(500);
		     resultado = tecla * 1000;
		     
		     cmd_LCD(0xC6,0);
		     escreve_LCD("*");
		     break;
		   }
	       }
	       
	       while (1) {
                tecla = Teclado();
		   if (tecla < 10){
		     _delay_ms(500);
		     resultado = resultado + (tecla * 100);
		     
		     escreve_LCD("*");
		     break;
		   }
	       }
	       
	       while (1) {
                tecla = Teclado();
		   if (tecla < 10){
		     _delay_ms(500);
		     resultado = resultado + (tecla * 10);
		     
		     escreve_LCD("*"); 
		     break;
		   }
	       }
	       
	       while (1) {
                tecla = Teclado();
		   if (tecla < 10){
		     _delay_ms(500);
		     resultado = resultado + tecla;
		     
		     escreve_LCD("*");
		     break;
		   }
	       }
	       
	       while (1) {
                tecla = Teclado();
		   if (tecla == 14){
		     break;
		   }
	       }
   
	return(resultado);
}

//------------------Final do captura senha---------------------------


//------------------Início do captura tempo---------------------------
int capturaTempo(int temp){
	int resultado;
	int tecla;
	char y[2];
   
	    while (1) {
	     tecla = Teclado();
	       
	     if (timeoutt >= temp) {
	     //  estadoAtual = estado_DESATIVADO;
	       break;
	       break;
	     }
	       
		if (tecla < 10){
		  _delay_ms(500);
		  resultado = tecla*100;
		  itoa(tecla, y, 10); 
		  escreve_LCD(y);
		  break;
		}
	    }
	    
	    while (1) {
	     tecla = Teclado();
	       
	     if (timeoutt >= temp) {
	    //   estadoAtual = estado_DESATIVADO;
	       break;
	       break;
	     }
		if (tecla < 10){
		  _delay_ms(500);
		  resultado = resultado + (tecla*10);
		  itoa(tecla, y, 10);
		  escreve_LCD(y);
		  break;
		}
	    }
	    
	    while (1) {
	     tecla = Teclado();
	       
	     if (timeoutt >= temp) {
	 //      estadoAtual = estado_DESATIVADO;
	       break;
	       break;
	     }
	       
		if (tecla < 10){
		  _delay_ms(500);
		  resultado = resultado + tecla;
		  itoa(tecla, y, 10); 
		  escreve_LCD(y);
		  break;
		}
	    }
       
      return(resultado);
}
//-----------------Final do captura tempo-----------------------------


//------------------ ADC ---------------------------------

void ADC_init() {
	ADMUX &=~ (1<<MUX3)|(1<<MUX2)|(1<<MUX1)|(1<<MUX0);
	ADMUX &=~ (1<<REFS1)|(1<<REFS0);
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADEN);
	ADCSRA &=~ (1<<ADLAR)|(1<<ADPS0)|(1<<ADATE);
}

int adc_start() {
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));
	return ADCW;
}
//--------------------------------------------------------

//----------------SPI------------------------------------------
void SPI_Init()
{
	DDRB |= (1<<MOSI)|(1<<SCK)|(1<<SS);	//Saida
	DDRB &= ~(1<<MISO);			//Entrada
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);	//Ativa SPI, Mestre, Clock/16
	SPSR &=~ (1<<SPI2X);			//Desabilita SPI2X
}

void SPI_dado(uint8_t data)//Envia dados
{
	PORTB &=~ (1<<SS);//Habilita transferencia
	SPDR = data;  //dado
	while(!(SPSR & (1 << SPIF)));
}

int SPI_ler_horas()			//Ler horas
{
	SPI_dado(0x85);
	SPDR = 0xFF;
	while(!(SPSR & (1<<SPIF)));	
	PORTB |= (1<<SS);
	return (SPDR);			
}

int SPI_ler_minutos()	
{
	SPI_dado(0x83);
	SPDR = 0xFF;
	while(!(SPSR & (1<<SPIF)));	
	PORTB |= (1<<SS);
	return (SPDR);	
}

int SPI_ler_dia()
{
	SPI_dado(0x8B);
	SPDR = 0xFF;
	while(!(SPSR & (1<<SPIF)));	
	PORTB |= (1<<SS);
	return (SPDR);	
}
//-----------------------------------------------------------

/*
void USART_inicializacao(unsigned int ubrr){

   UBRR0H = (unsigned char)(ubrr>>8);           // Configura o Baud Rate High
   UBRR0L = (unsigned char)ubrr;                // Configura o Baud Rate Low
   UCSR0B = (1<<RXEN0)|(1<<TXEN0);              // Dá enable no transmissor de receptor
   UCSR0C = (2<<UPM00)|(0<<USBS0)|(3<<UCSZ00);  // Configura 8 bits de palavra e  1 bit de parada
}

void USART_transmissao(unsigned char mensagem_usart){

   while (!(UCSR0A & (1<<UDRE0)));              // Espera o esvaziamento do buffer de transmissão
   UDR0 = mensagem_usart;                       // Coloca a mensagem no buffer
}

unsigned char USART_recepcao(void){

   while (!(UCSR0A & (1<<RXC0)));              // Espera uma mensagem ser recebida
   return UDR0;                                // Retorna a mensagem do buffer
}*/

void EEPROM_escrita(unsigned int endereco_escrita_eeprom, unsigned char mensagem_escrita_eeprom){

   while(EECR & (1<<EEPE));                    // Espera a finalização da ultima escrita
   EEAR = endereco_escrita_eeprom;             // Configura o endereço de escrita
   EEDR = mensagem_escrita_eeprom;             // Passa a mensagem
   EECR |= (1<<EEMPE);                         // Da enable na escrita
   EECR |= (1<<EEPE);                          // Começa a escrita
}

unsigned char EEPROM_leitura(unsigned int endereco_leitura_eeprom){

   while(EECR & (1<<EEPE));                    // Espera a finalização da ultima escrita
   EEAR = endereco_leitura_eeprom;             // Configura o endereço de leitura
   EECR |= (1<<EERE);                          // Da enable na leitura
   return EEDR;                                // Retorna a mensagem da leitura
}

unsigned char sensores(){
   unsigned char sens = 0;
   unsigned char contador_serial = 0;
   
   PORTD &= ~(1 << CLK_S);
   PORTB &= ~(1 << LOAD);
   _delay_us(10);
   PORTB |= (1 << LOAD);
	   while(contador_serial<8){
		   if(PIND & (1 << SERIAL)){
			   sens |= 1 << contador_serial;
			   PORTD |= (1 << CLK_S);
		   }
		   else if(!(PIND & (1 << SERIAL))){
			   sens &= ~(1 << contador_serial);
			   PORTD |= (1 << CLK_S);
		   }
	   _delay_us(10);
	   PORTD &= ~(1 << CLK_S);
	   contador_serial = contador_serial + 1;
	   }
   return sens;
}

void uart_init (void) {
    UBRR0H=(BAUDRATE>>8);
    UBRR0L=BAUDRATE;                         //set baud rate
    UCSR0B|=(1<<TXEN0)|(1<<RXEN0);             //enable receiver and transmitter
    UCSR0C|=(1<<UCSZ00)|(1<<UCSZ01);// 8bit data format
}
  
// function to send data - NOT REQUIRED FOR THIS PROGRAM IMPLEMENTATION
void uart_transmit (unsigned char data) {
    while (!( UCSR0A & (1<<UDRE0)));            // wait while register is free
    UDR0 = data;                             // load data in the register
}
  
// function to receive data
unsigned char uart_recieve (void) {
    while(!(UCSR0A) & (1<<RXC0));           // wait while data is being received
    return UDR0;                             // return 8-bit data
}

//--------ESTADOS---------------------------------------

typedef enum estado_e{
    estado_INICIO,
    estado_RECUPERACAO,
    estado_DESATIVADO,
    estado_CHECAR_SENHA_IN,
    estado_CHECAR_SENHA_OUT,
    estado_PANICO,
    estado_ATIVADO,
    estado_PROGRAMACAO,
    estado_CONFIGURACAO,
    estado_HABILITAR,
    estado_DESABILITAR,
    estado_DESABILITAR_SENSOR,
    estado_DESASSOCIAR_SENSOR,
    estado_DESASSOCIAR_ZONA,
    estado_DESABILITAR_ZONA,
    estado_USUARIO,
    estado_NOVA_SENHA,
    estado_HABILITAR_SENSOR,
    estado_ASSOCIAR_SENSOR,
    estado_ASSOCIAR_ZONA,
    estado_HABILITAR_ZONA,
    estado_TEMP_ATIVACAO,
    estado_TEMP_TIME_OUT,
    estado_TEMP_SIRENE,
}estado_t;  

static estado_t estadoAtual = estado_INICIO;

//========================== CODIGO PRINCIPAL ===========================
int main(void){

   DDRD = 0b11110011;
   DDRB = 0b11101111;
   //DDRC = 0b11111110;

   set_bit(DDRC,LED_ATIV);
   set_bit(DDRC,LED_PROG);
   set_bit(DDRC,LED_ZN1);
   set_bit(DDRC,LED_ZN2);
   set_bit(DDRC,LED_ZN3);
   
   set_bit(DDRB, ALARME);
   
   int temporizador_ativacao;
   int temporizador_timeout;
   int temporizador_sirene;
   
   int tecla;
   int temp;
   int sensor_associado;
   int zona;
   int aux;
   
   int usuario[4] = {1000, 1001, 1002, 1003};
   
   int senha[4];
   int senha_in;
   int i=0;
   
   int horas, minutos, dia;
   
   unsigned char oito_sensores = 0b00000000;
   unsigned char mask_zona1 = 0b00000000;
   unsigned char mask_zona2 = 0b00000000;
   unsigned char mask_zona3 = 0b00000000;
   unsigned char sens_habilitado = 0b11111111;
   unsigned char modo;
   
   bool hab_zn1 = true;
   bool hab_zn2 = true;
   bool hab_zn3 = true;
   
   int endereco=0;
   
   char y[2];
   
   timer1_init();
   
   ADC_init();
   
   SPI_Init();
   //uart_init();
   
    while(1){
        switch(estadoAtual){
 //-------------Inicio da inicialização------------------------------------------------------------------------------------           
	    case estado_INICIO:
                inicializa_LCD();
                estadoAtual = estado_RECUPERACAO;
            break;
//-------------Fim da inicialização------------------------------------------------------------------------------------ 
	   
//-------------Inicio do estado Recuperação---------------------------------------------------------------------------------
            case estado_RECUPERACAO:
	       
	        cmd_LCD(0x01,0);
		escreve_LCD("      MODO");
		cmd_LCD(0xC0,0);
                escreve_LCD("  RECUPERACAO");
	    
		temporizador_ativacao = 0;
		temporizador_timeout = 99;
		temporizador_sirene = 0;      
		senha[0] = 1234;
		senha[1] = NULL;
		senha[2] = NULL;
		senha[3] = NULL;
	    
	       mask_zona1 = 0b00000000;
	       mask_zona2 = 0b00000000;
	       mask_zona3 = 0b00000000;
	       sens_habilitado = 0b11111111;

	       hab_zn1 = true;
	       hab_zn2 = true;
	       hab_zn3 = true;
	    
	       estadoAtual = estado_DESATIVADO;
	       break;
//-------------Fim do estado Recuperação---------------------------------------------------------------------------------

//-------------Inicio do estado Desativado---------------------------------------------------------------------------------
            case estado_DESATIVADO:
		modo = "D";
		clr_bit(PORTB, ALARME);
		clr_bit(PORTC, LED_ATIV);
	        clr_bit(PORTC, LED_PROG);
		clr_bit(PORTC, LED_ZN1);
	        clr_bit(PORTC, LED_ZN2);
	        clr_bit(PORTC, LED_ZN3);
	    
		cmd_LCD(0x01,0);
		escreve_LCD("      MODO");
		cmd_LCD(0xC0,0);
                escreve_LCD("   DESATIVADO");
	        
		while (1) {
		   tecla = Teclado();
		   if (tecla == 13){
		     estadoAtual = estado_CHECAR_SENHA_IN;
		     break;
		   } else if (tecla == 12){
		     estadoAtual = estado_PROGRAMACAO;
		     break;
		   } else if (tecla == 15){
		     estadoAtual = estado_PANICO;
		     break;
		   } else if (tecla == 16){
		      tempoR = 0;
		      
		      while(tecla == 16){
			 tecla = Teclado();
			 if(tempoR >=10){
			    estadoAtual = estado_RECUPERACAO;
			    break;
			 }
		      }
		      
		      break;
		   }
	       }
	       break;
//-------------Fim do estado Desativado---------------------------------------------------------------------------------

//-------------Inicio do estado que verifica a senha para ativar o alarme---------------------------------------------------------------------------------
            case estado_CHECAR_SENHA_IN:
	       cmd_LCD(0x01,0);
	       escreve_LCD(" DIGITE A SENHA");
	       cmd_LCD(0xC0,0);
	       escreve_LCD("");
	    
	       senha_in = NULL;
	       senha_in = capturaSenha();
	       
	       for(i = 0; i <= 3; i++) {
		  if (senha_in == senha[i]) { 
					 
			 horas = SPI_ler_horas();
			 minutos = SPI_ler_minutos();
			 dia = SPI_ler_dia();

			eeprom_write_byte(endereco,horas);
			 endereco=endereco+1;
			 eeprom_write_byte(endereco,minutos);
			 endereco=endereco+1;
			 eeprom_write_byte(endereco,dia);
			 endereco=endereco+1;
			 eeprom_write_byte(endereco,i);
			 endereco=endereco+1;
			 eeprom_write_word(endereco,modo);
			 endereco=endereco+1;
		     
			 if (temporizador_ativacao > 0) {
			    tempoA = 0;
			    
			    while (tempoA < temporizador_ativacao) {
			       cmd_LCD(0xC0,0);
			       escreve_LCD("AGUARDE ATIVACAO");
			    }
			    
			 }
			 
			 estadoAtual = estado_ATIVADO;
		     break;
		  } else {
		     estadoAtual = estado_DESATIVADO;
		  }
	       }
	       
	       break;
//-------------Fim do estado que verifica a senha para ativar o alarme---------------------------------------------------------------------------------
	       
//-------------Inicio do estado que verifica a senha para desativar o alarme----------------------------------------------
	    case estado_CHECAR_SENHA_OUT:
	       cmd_LCD(0x01,0);
	       escreve_LCD(" DIGITE A SENHA");
	       cmd_LCD(0xC0,0);
	       escreve_LCD("");
	    
	       senha_in = NULL;
	       senha_in = capturaSenha();
	    
	       for(i = 0; i <= 3; i++) {
		  if (senha_in == senha[i]){
			 
			 horas = SPI_ler_horas();
			 minutos = SPI_ler_minutos();
			 dia = SPI_ler_dia();

			eeprom_write_byte(endereco,horas);
			 endereco=endereco+1;
			 eeprom_write_byte(endereco,minutos);
			 endereco=endereco+1;
			 eeprom_write_byte(endereco,dia);
			 endereco=endereco+1;
			 eeprom_write_byte(endereco,i);
			 endereco=endereco+1;
			 eeprom_write_byte(endereco,modo);
			 endereco=endereco+1;
		     
		     estadoAtual = estado_DESATIVADO;
		     break;
		  } else {
		     estadoAtual = estado_ATIVADO;
		  }
	       }
	       
	       break;
//-------------Fim do estado que verifica a senha para desativar o alarme----------------------------------------------
	       
//-------------Inicio do estado Ativado---------------------------------------------------------------------------------
            case estado_ATIVADO:
		modo = "A";
	        set_bit(PORTC, LED_ATIV);
	        cmd_LCD(0x01,0);
		escreve_LCD("      MODO");
		cmd_LCD(0xC0,0);;
                escreve_LCD("    ATIVADO");
	       bool teste = false;
                while (1) {
		oito_sensores = sensores();
		
		if( ((hab_zn1 && (((oito_sensores & sens_habilitado)& mask_zona1) != 0)) ||
		  (hab_zn2 && (((oito_sensores & sens_habilitado)& mask_zona2) != 0)) || 
		  (hab_zn3 && (((oito_sensores & sens_habilitado)& mask_zona3) != 0)))  && (teste == false)){
		     
		     tempoS = 0;
		     teste = true;
		}
		
		if(hab_zn1 && (((oito_sensores & sens_habilitado)& mask_zona1) != 0)){
		   set_bit(PORTC,LED_ZN1);
		   set_bit(PORTB, ALARME);
		}
		if(hab_zn2 && (((oito_sensores & sens_habilitado)& mask_zona2) != 0)){
		   set_bit(PORTC,LED_ZN2);
		   set_bit(PORTB, ALARME);
		}
		if(hab_zn3 && (((oito_sensores & sens_habilitado)& mask_zona3) != 0)){
		   set_bit(PORTC,LED_ZN3);
		   set_bit(PORTB, ALARME);
		}
		
                tecla = Teclado();
		   if (tecla == 11){
		     estadoAtual = estado_CHECAR_SENHA_OUT;
		     break;
		   } else if (tecla == 15){
		     estadoAtual = estado_PANICO;
		     break;
		   } else if (tecla == 16){
		      tempoR = 0;
		      
		      while(tecla == 16){
			 tecla = Teclado();
			 if(tempoR >=10){
			    estadoAtual = estado_RECUPERACAO;
			    break;
			 }
		      }
		      
		      break;
		   }
		   
		   if(teste == true){
		      if(temporizador_sirene > 0 && tempoS >= temporizador_sirene){
			 teste = false;
			 estadoAtual = estado_DESATIVADO;
			 break;
		      }
		   }
	       }
	       break;
//-------------Fim do estado Ativado---------------------------------------------------------------------------------

//-------------Inicio do estado Pânico---------------------------------------------------------------------------------	       
            case estado_PANICO:
		set_bit(PORTB, ALARME);
		clr_bit(PORTC, LED_ATIV);
	        clr_bit(PORTC, LED_PROG);
	    
	        cmd_LCD(0x01,0);
		escreve_LCD("      MODO");
		cmd_LCD(0xC0,0);
                escreve_LCD("     PANICO");
	    
		while (1) {
		   tecla = Teclado();
		   if (tecla == 15){
		     estadoAtual = estado_DESATIVADO;
		     break;
		   }
		}
		break;
//-------------Fim do estado Pânico---------------------------------------------------------------------------------
		
//-------------Inicio do estado Programação---------------------------------------------------------------------------------
	    case estado_PROGRAMACAO:
	        set_bit(PORTC, LED_PROG);
	        cmd_LCD(0x01,0);
		escreve_LCD("      MODO");
		cmd_LCD(0xC0,0);
                escreve_LCD("  PROGRAMACAO");
	       _delay_ms(1000);
	    
	       cmd_LCD(0x01,0);
	       escreve_LCD(" DIGITE A SENHA");
	       cmd_LCD(0xC0,0);
	       escreve_LCD("");
	    
	       timeoutt = 0;
	    
	       senha_in = NULL;
	    
	       while (1) {
                tecla = Teclado();
		  
		if (timeoutt >= temporizador_timeout) {
		  estadoAtual = estado_DESATIVADO;
		  break;
	        }
		  
		   if (tecla < 10){
		     _delay_ms(500);
		     senha_in = tecla * 1000;
		     
		     cmd_LCD(0xC6,0);
		     escreve_LCD("*");
		     break;
		   }
	       }
	       
	       while (1) {
                tecla = Teclado();
		  
		if (timeoutt >= temporizador_timeout) {
		  estadoAtual = estado_DESATIVADO;
		  break;
	        }
		  
		   if (tecla < 10){
		     _delay_ms(500);
		     senha_in = senha_in + (tecla * 100);
		     
		     escreve_LCD("*");
		     break;
		   }
	       }
	       
	       while (1) {
                tecla = Teclado();
		 
		if (timeoutt >= temporizador_timeout) {
		  estadoAtual = estado_DESATIVADO;
		  break;
	        }
		  
		   if (tecla < 10){
		     _delay_ms(500);
		     senha_in = senha_in + (tecla * 10);
		     
		     escreve_LCD("*"); 
		     break;
		   }
	       }
	       
	       while (1) {
                tecla = Teclado();
		 
		if (timeoutt >= temporizador_timeout) {
		  estadoAtual = estado_DESATIVADO;
		  break;
	        }
		  
		   if (tecla < 10){
		     _delay_ms(500);
		     senha_in = senha_in + tecla;
		     
		     escreve_LCD("*");
		     break;
		   }
	       }
	       
	       while (1) {
                tecla = Teclado();
		
		if (timeoutt >= temporizador_timeout) {
		  estadoAtual = estado_DESATIVADO;
		  break;
		}
		
		   if (tecla == 14){
		     break;
		   }
	       }
	    
	       if (timeoutt >= temporizador_timeout) {
		  estadoAtual = estado_DESATIVADO;
		  break;
	       }
	    
	       //testando a senha mestre
		  if (senha_in == senha[0]) {
		     estadoAtual = estado_CONFIGURACAO;
		     break;
		  }else{
		    estadoAtual = estado_DESATIVADO;
		     break;
		  } 
//-------------Fim do estado Programacao---------------------------------------------------------------------------------

//-------------Inicio do estado Configuração---------------------------------------------------------------------------------		  
	    case estado_CONFIGURACAO:
	        cmd_LCD(0x01,0);
		escreve_LCD("    SELECIONE");
		cmd_LCD(0xC0,0);
                escreve_LCD("     A ou D");

	       while (1) {
		  tecla = Teclado();
		  
		  if (timeoutt >= temporizador_timeout) {
		     cmd_LCD(0x01,0);
		     escreve_LCD("     TEMPO");
		     cmd_LCD(0xC0,0);
                     escreve_LCD("    EXCEDIDO");
		     
		     estadoAtual = estado_DESATIVADO;
		     break;
		  }
		  
		  if (tecla == 11){ //tecla D
		     estadoAtual = estado_DESABILITAR;
		     break;
		   } else if (tecla == 15){ //tecla S
		     estadoAtual = estado_PANICO;
		     break;
		   //} else if (tecla > 14){ //tecla E
		     //estadoAtual = estado_DESATIVADO;
		     //break;
		   }  else if (tecla == 13){ //tecla A
		     estadoAtual = estado_HABILITAR;
		     break;
		   } else if (tecla == 16){ //tecla R
		      tempoR = 0;
		      
		      while(tecla == 16){
			 tecla = Teclado();
			 estadoAtual = estado_DESATIVADO;
			 if(tempoR >=10){
			    estadoAtual = estado_RECUPERACAO;
			    break;
			 }
		      }
		      
		      break;
		   }
		}
		break;
                
//-------------Fim do estado Configuracao---------------------------------------------------------------------------------
		
		
//-------------Inicio do estado Habilitar---------------------------------------------------------------------------------
		case estado_HABILITAR:
		   cmd_LCD(0x01,0);
		   escreve_LCD("   HABILITAR");
		   cmd_LCD(0xC0,0);
		   escreve_LCD("     2 -- 8  ");
		   
		while (1) {
		   tecla = Teclado();
		   if (tecla == 2){
		     estadoAtual = estado_USUARIO;
		     break;
		   } else if (tecla == 3){
		     estadoAtual = estado_HABILITAR_SENSOR;
		     break;
		   } else if (tecla == 4){
		     estadoAtual = estado_ASSOCIAR_SENSOR;
		     break;
		   } else if (tecla == 5){
		     estadoAtual = estado_HABILITAR_ZONA;
		     break;
		   } else if (tecla == 6){
		     estadoAtual = estado_TEMP_ATIVACAO;
		     break;
		   } else if (tecla == 7){
		     estadoAtual = estado_TEMP_TIME_OUT;
		     break;
		   } else if (tecla == 8){
		     estadoAtual = estado_TEMP_SIRENE;
		     break;
		   } else if (tecla == 16){ //tecla R
		     estadoAtual = estado_CONFIGURACAO;
		     break;
		   } 
		   
		   if (timeoutt >= temporizador_timeout) {
			estadoAtual = estado_DESATIVADO;
			break;
		    }
		}
		break;		
//-------------Fim do estado Habilitar---------------------------------------------------------------------------------
		
		
//-------------Inicio do estado Desabilitar---------------------------------------------------------------------------------
		case estado_DESABILITAR:
		   cmd_LCD(0x01,0);
		   escreve_LCD("  DESABILITAR");
		   cmd_LCD(0xC0,0);
		   escreve_LCD("   3, 4 ou 5  ");
		   
		while (1) {
		   tecla = Teclado();
		   if (tecla == 3){
		     estadoAtual = estado_DESABILITAR_SENSOR;
		     break;
		   } else if (tecla == 4){
		     estadoAtual = estado_DESASSOCIAR_SENSOR;
		     break;
		   } else if (tecla == 5){
		     estadoAtual = estado_DESABILITAR_ZONA;
		     break;
		   } else if (tecla == 16){ //tecla R
		     estadoAtual = estado_CONFIGURACAO;
		     break;
		   } 
		   
		   if (timeoutt >= temporizador_timeout) {
			estadoAtual = estado_DESATIVADO;
			break;
		    }
		}
		break;		
//-------------Fim do estado Desabilitar---------------------------------------------------------------------------------
		
//-------------Inicio do estado Desabilitar Sensor---------------------------------------------------------------------------------
		case estado_DESABILITAR_SENSOR:
		   cmd_LCD(0x01,0);
		   escreve_LCD("SELECIONE SENSOR");
		   cmd_LCD(0xC0,0);
		   escreve_LCD("( 0 - 7 ): ");
		
		while (1) {
		   tecla = Teclado();
		   
		   if (timeoutt >= temporizador_timeout) {
			estadoAtual = estado_DESATIVADO;
			break;
		    }
		   
		      if (tecla < 8){
			aux = tecla;
			_delay_ms(500);
			itoa(tecla, y, 10); 
			escreve_LCD(y); 
			while(1){
			   tecla = Teclado();
			   
			   if (timeoutt >= temporizador_timeout) {
				estadoAtual = estado_DESATIVADO;
				break;
			   }
			   
			   if(tecla == 14){
			      sens_habilitado &= ~(1 << aux);
			      estadoAtual = estado_CONFIGURACAO;
			      break;
			   } else if (tecla == 16){ //tecla R
			      estadoAtual = estado_CONFIGURACAO;
			      break;
			   }  else if (tecla == 3){ //tecla 3
			      sens_habilitado &= ~(1 << aux);
			      break;
			   } 
			}
			break;
		      } else if (tecla == 16){ //tecla R
			estadoAtual = estado_CONFIGURACAO;
			break;
		      } 
		}
	
		break;
//-------------Fim do estado Desabilitar Sensor---------------------------------------------------------------------------------
		
//-------------Inicio do estado Desassociar Sensor---------------------------------------------------------------------------------
		case estado_DESASSOCIAR_SENSOR:
		   cmd_LCD(0x01,0);
		   escreve_LCD("SELECIONE SENSOR");
		   cmd_LCD(0xC0,0);
		   escreve_LCD("( 0 - 7 ): ");
		
		while (1) {
		   tecla = Teclado();
		   
		   if (timeoutt >= temporizador_timeout) {
			estadoAtual = estado_DESATIVADO;
			break;
		    }
		   
		      if (tecla < 8){
			 aux = tecla;
			_delay_ms(500);
			itoa(tecla, y, 10); 
			escreve_LCD(y); 
			while(1){
			   tecla = Teclado();
			   
			   if (timeoutt >= temporizador_timeout) {
			      estadoAtual = estado_DESATIVADO;
			      break;
			   }
			   
			   if(tecla == 14){
			      sensor_associado = aux;
			      estadoAtual = estado_DESASSOCIAR_ZONA;
			      break;
			   } else if (tecla == 16){ //tecla R
			      estadoAtual = estado_CONFIGURACAO;
			      break;
			   } 
			}
			break;
		      } else if (tecla == 16){ //tecla R
			estadoAtual = estado_CONFIGURACAO;
			break;
		      } 
		}
		break;
//-------------Fim do estado Desassociar Sensor---------------------------------------------------------------------------------
		
//-------------Inicio do estado Desassociar zona---------------------------------------------------------------------------------
		case estado_DESASSOCIAR_ZONA:
		   cmd_LCD(0x01,0);
		   escreve_LCD("SELECIONE ZONA");
		   cmd_LCD(0xC0,0);
		   escreve_LCD("( 1 - 3 ): ");
		
		while (1) {
		   tecla = Teclado();
		   
		   if (timeoutt >= temporizador_timeout) {
			estadoAtual = estado_DESATIVADO;
			break;
		   }
		   
		      if (tecla>0 && tecla < 4){
			_delay_ms(500);
			itoa(tecla, y, 10); 
			escreve_LCD(y); 
			while(1){
			   tecla = Teclado();
			   
			   if (timeoutt >= temporizador_timeout) {
			      estadoAtual = estado_DESATIVADO;
			      break;
			   }
			   
			   if(tecla == 14){
			      if(zona == 1){
				 mask_zona1 &= ~(1 << sensor_associado);
			      } else if(zona == 2){
				 mask_zona2 &= ~(1 << sensor_associado);
			      } else if(zona == 3){
				 mask_zona3 &= ~(1 << sensor_associado);
			      }
			      estadoAtual = estado_CONFIGURACAO;
			      break;
			   } else if (tecla == 16){ //tecla R
			      estadoAtual = estado_CONFIGURACAO;
			      break;
			   } else if (tecla == 4){
			      if(zona == 1){
				 mask_zona1 &= ~(1 << sensor_associado);
			      } else if(zona == 2){
				 mask_zona2 &= ~(1 << sensor_associado);
			      } else if(zona == 3){
				 mask_zona3 &= ~(1 << sensor_associado);
			      }
			      estadoAtual = estado_DESASSOCIAR_SENSOR;
			      break;
			   }
			}
			break;
		      } else if (tecla == 16){ //tecla R
			estadoAtual = estado_CONFIGURACAO;
			break;
		      } 
		}
		break;
//-------------Fim do estado Desassociar zona---------------------------------------------------------------------------------


//-------------Inicio do estado Desabilitar zona---------------------------------------------------------------------------------
		case estado_DESABILITAR_ZONA:
		   cmd_LCD(0x01,0);
		   escreve_LCD("SELECIONE ZONA");
		   cmd_LCD(0xC0,0);
		   escreve_LCD("( 1 - 3 ): ");
		
		      while (1) {
			 tecla = Teclado();
			 
			 if (timeoutt >= temporizador_timeout) {
			      estadoAtual = estado_DESATIVADO;
			      break;
			  }
			 
			 if (tecla>0 && tecla < 4){
			   aux = tecla;
			   _delay_ms(500);
			   itoa(tecla, y, 10); 
			   escreve_LCD(y); 
			   while(1){
			      tecla = Teclado();
			      
			      if (timeoutt >= temporizador_timeout) {
				 estadoAtual = estado_DESATIVADO;
				 break;
			      }
			      
			      if(tecla == 14){
				 if(aux == 1){
				    hab_zn1 = false;
				 }else if(aux == 2){
				    hab_zn2 = false;
				 }else if(aux == 3){
				    hab_zn3 = false;
				 }
				 estadoAtual = estado_CONFIGURACAO;
				 break;
			      } else if (tecla == 16){ //tecla R
				 estadoAtual = estado_CONFIGURACAO;
				 break;
			      }  else if (tecla == 5){ //tecla 5
				 if(aux == 1){
				    hab_zn1 = false;
				 }else if(aux == 2){
				    hab_zn2 = false;
				 }else if(aux == 3){
				    hab_zn3 = false;
				 }
				 break;
			      } 
			   }
			break;
		      } else if (tecla == 16){ //tecla R
			estadoAtual = estado_CONFIGURACAO;
			break;
		      }  
		}
		break;
//-------------Fim do estado Desabilitar zona ---------------------------------------------------------------------------------		
	       
//-------------Inicio do estado Usuario---------------------------------------------------------------------------------
	       case estado_USUARIO:
		   cmd_LCD(0x01,0);
		   escreve_LCD("ESCOLHA USUARIO");
	           cmd_LCD(0xC0,0);
		   escreve_LCD("( 0 - 3 ): ");
		
		      while (1) {
			 tecla = Teclado();
			 
			 if (timeoutt >= temporizador_timeout) {
			     estadoAtual = estado_DESATIVADO;
			     break;
			 }
			 
			 if (tecla < 4){
			   _delay_ms(500);
			   i = tecla;
			   itoa(tecla, y, 10); 
			   escreve_LCD(y);
			   estadoAtual = estado_NOVA_SENHA;
			   break;
			} else if (tecla == 16){ //tecla R
			   estadoAtual = estado_CONFIGURACAO;
			   break;
			} 
		      }
		break;
//-------------Fim do estado Usuario---------------------------------------------------------------------------------		
	      
//-------------Inicio do estado Nova Senha---------------------------------------------------------------------------------
	       case estado_NOVA_SENHA:
		  cmd_LCD(0x01,0);
	       escreve_LCD("NOVA SENHA:");
	    
	       senha_in = NULL;
	       
	       while (1) {
                tecla = Teclado();
		
		if (timeoutt >= temporizador_timeout) {
		     estadoAtual = estado_DESATIVADO;
		     break;
		 }
		    
		   if (tecla < 10){
		     _delay_ms(500);
		     senha_in = tecla * 1000;
		     itoa(tecla, y, 10); 
		     cmd_LCD(0xC5,0);
		     escreve_LCD(y);
		     break;
		   }
	       }
	       
	       while (1) {
                tecla = Teclado();
		  
		if (timeoutt >= temporizador_timeout) {
		     estadoAtual = estado_DESATIVADO;
		     break;
		 }
		 
		   if (tecla < 10){
		     _delay_ms(1000);
		     senha_in = senha_in + (tecla * 100);
		     itoa(tecla, y, 10);
		     escreve_LCD(y);
		     break;
		   }
	       }
	       
	       while (1) {
                tecla = Teclado();
		  
	        if (timeoutt >= temporizador_timeout) {
		     estadoAtual = estado_DESATIVADO;
		     break;
		}
		  
		   if (tecla < 10){
		     _delay_ms(500);
		     senha_in = senha_in + (tecla * 10);
		     itoa(tecla, y, 10); 
		     escreve_LCD(y);
		     break;
		   }
	       }
	       
	       while (1) {
                tecla = Teclado();
		  
		if (timeoutt >= temporizador_timeout) {
		     estadoAtual = estado_DESATIVADO;
		     break;
		}  
		  
		   if (tecla < 10){
		     _delay_ms(500);
		     senha_in = senha_in + tecla;
		     itoa(tecla, y, 10);
		     escreve_LCD(y);
		     break;
		   }
	       }
	       
	       while(1){
		  tecla = Teclado();
		  
		  if (timeoutt >= temporizador_timeout) {
			estadoAtual = estado_DESATIVADO;
			break;
		  }
		  
		  if(tecla == 2){ //tecla 2
		     senha[i] = senha_in;
		     estadoAtual = estado_USUARIO;
		     break;
		  } else if(tecla == 14){ //tecla E
		     senha[i] = senha_in;
		     estadoAtual = estado_CONFIGURACAO;
		     break;
		  } else if (tecla == 16){ //tecla R
		     estadoAtual = estado_CONFIGURACAO;
		     break;
		   } 
	       }
	              
	       break;
//-------------Fim do estado Nova Senha---------------------------------------------------------------------------------

		      
//-------------Inicio do estado Habilitar Sensor---------------------------------------------------------------------------------
	       case estado_HABILITAR_SENSOR:
		  cmd_LCD(0x01,0);
		   escreve_LCD("SELECIONE SENSOR");
		   cmd_LCD(0xC0,0);
		   escreve_LCD("( 0 - 7 ): ");
		
		while (1) {
		   tecla = Teclado();
		   
		   if (timeoutt >= temporizador_timeout) {
			estadoAtual = estado_DESATIVADO;
			break;
		    }
		   
		      if (tecla < 8){
			sens_habilitado |= 1 << tecla;
			_delay_ms(500);
			itoa(tecla, y, 10); 
			escreve_LCD(y); 
			while(1){
			   tecla = Teclado();
			   
			   if (timeoutt >= temporizador_timeout) {
			      estadoAtual = estado_DESATIVADO;
			      break;
			   }
			   
			   if(tecla == 14){
			      estadoAtual = estado_CONFIGURACAO;
			      break;
			   } else if (tecla == 16){ //tecla R
			      estadoAtual = estado_CONFIGURACAO;
			      break;
			   }  else if (tecla == 3){ //tecla 3
			      break;
			   } 
			}
			break;
		      } else if (tecla == 16){ //tecla R
			estadoAtual = estado_CONFIGURACAO;
			break;
		      } 
		}
	       break;
//-------------Fim do estado Habilitar Sensor---------------------------------------------------------------------------------


//-------------Inicio do estado Associar Sensor---------------------------------------------------------------------------------
		case estado_ASSOCIAR_SENSOR:
		   cmd_LCD(0x01,0);
		   escreve_LCD("SELECIONE SENSOR");
		   cmd_LCD(0xC0,0);
		   escreve_LCD("( 0 - 7 ): ");
		
		while (1) {
		   tecla = Teclado();
		   
		   if (timeoutt >= temporizador_timeout) {
			estadoAtual = estado_DESATIVADO;
			break;
		    }
		   
		      if (tecla < 8){
			sensor_associado = tecla;
			_delay_ms(500);
			itoa(tecla, y, 10); 
			escreve_LCD(y); 
			while(1){
			   tecla = Teclado();
			   
			   if (timeoutt >= temporizador_timeout) {
			      estadoAtual = estado_DESATIVADO;
			      break;
			   }
			   
			   if(tecla == 14){
			      estadoAtual = estado_ASSOCIAR_ZONA;
			      break;
			   }  else if (tecla == 16){ //tecla R
			      estadoAtual = estado_CONFIGURACAO;
			      break;
			    } 
			}
			break;
		      } else if (tecla == 16){ //tecla R
			estadoAtual = estado_CONFIGURACAO;
			break;
		      } 
		}
		break;
//-------------Fim do estado Associar Sensor---------------------------------------------------------------------------------


//-------------Inicio do estado Associar zona--------------------------------------------------------------------------------
		case estado_ASSOCIAR_ZONA:
		   cmd_LCD(0x01,0);
		   escreve_LCD("SELECIONE ZONA");
		   cmd_LCD(0xC0,0);
		   escreve_LCD("( 1 - 3 ): ");
		
		while (1) {
		   tecla = Teclado();
		   
		      if (timeoutt >= temporizador_timeout) {
			   estadoAtual = estado_DESATIVADO;
			   break;
		      }
		   
		      if (tecla>0 && tecla < 4){
			zona = tecla;
			_delay_ms(500);
			itoa(tecla, y, 10); 
			escreve_LCD(y); 
			while(1){
			   tecla = Teclado();
			   
			   if (timeoutt >= temporizador_timeout) {
			      estadoAtual = estado_DESATIVADO;
			      break;
			   }
			   
			   if(tecla == 14){
			      if(zona == 1){
				 mask_zona1 |= 1 << sensor_associado;
			      } else if(zona == 2){
				 mask_zona2 |= 1 << sensor_associado;
			      } else if(zona == 3){
				 mask_zona3 |= 1 << sensor_associado;
			      }
			      estadoAtual = estado_CONFIGURACAO;
			      break;
			   } else if (tecla == 16){ //tecla R
			     estadoAtual = estado_CONFIGURACAO;
			     break;
			   }  
			}
			break;
		      } else if (tecla == 4){
			      estadoAtual = estado_ASSOCIAR_SENSOR;
			      break;
		      }  else if (tecla == 16){ //tecla R
			estadoAtual = estado_CONFIGURACAO;
			break;
		      } 
		}
		break;
//-------------Fim do estado Associar zona---------------------------------------------------------------------------------
		
//-------------Inicio do estado Habilitar zona---------------------------------------------------------------------------------
		case estado_HABILITAR_ZONA:
		   cmd_LCD(0x01,0);
		   escreve_LCD("SELECIONE ZONA");
		   cmd_LCD(0xC0,0);
		   escreve_LCD("( 1 - 3 ): ");
		
		      while (1) {
			 tecla = Teclado();
			 
			 if (timeoutt >= temporizador_timeout) {
			      estadoAtual = estado_DESATIVADO;
			      break;
			 }
			 
			 if (tecla>0 && tecla < 4){
			   aux= tecla;
			   _delay_ms(500);
			   itoa(tecla, y, 10); 
			   escreve_LCD(y); 
			   while(1){
			      tecla = Teclado();
			      
			      if (timeoutt >= temporizador_timeout) {
				    estadoAtual = estado_DESATIVADO;
				    break;
			      }
			      
			      if(tecla == 14){
				 if(aux == 1){
				    hab_zn1 = true;
				 }else if(aux == 2){
				    hab_zn2 = true;
				 }else if(aux == 3){
				    hab_zn3 = true;
				 }
				 estadoAtual = estado_CONFIGURACAO;
				 break;
			      } else if (tecla == 16){ //tecla R
				estadoAtual = estado_CONFIGURACAO;
				break;
			      }  else if (tecla == 5){ //tecla 5
				 break;
			      }   
			   }
			break;
		      } else if (tecla == 16){ //tecla R
			estadoAtual = estado_CONFIGURACAO;
			break;
		      }  
		}
		break;
//-------------Fim do estado Habilitar zona ---------------------------------------------------------------------------------			


//-------------Inicio do estado Temp Ativacao---------------------------------------------------------------------------------
		case estado_TEMP_ATIVACAO:
		   cmd_LCD(0x01,0);
		   escreve_LCD("TIMER ATIVACAO");
		   cmd_LCD(0xC0,0);
		   escreve_LCD("(000-999): ");
		   temp = NULL;
		   temp = capturaTempo(temporizador_timeout);
		
		   while(1){
		     tecla = Teclado();
		      
		     if (timeoutt >= temporizador_timeout) {
			estadoAtual = estado_DESATIVADO;
			break;
		     }
		      
		     if(tecla == 14){
			temporizador_ativacao = temp;
			break;
		     } else if (tecla == 16){ //tecla R
			break;
		      }  
		   }
		   
		   estadoAtual = estado_CONFIGURACAO;
		   break;
//-------------Fim do estado Temp Ativacao---------------------------------------------------------------------------------
		     
//-------------Inicio do estado Temp Time Out---------------------------------------------------------------------------------
		case estado_TEMP_TIME_OUT:
		   cmd_LCD(0x01,0);
		   escreve_LCD("TIMER TIMEOUT");
		   cmd_LCD(0xC0,0);
		   escreve_LCD("(00-99 ): ");
		   temp = NULL;
		   
		   while (1) {
		      tecla = Teclado();
		      
		      if (timeoutt >= temporizador_timeout) {
			estadoAtual = estado_DESATIVADO;
			break;
		      }
		    
			 if (tecla < 10){
			   _delay_ms(500);
			   temp = temp + (tecla*10);
			   itoa(tecla, y, 10);
			   escreve_LCD(y);
			   break;
			 }
		   }
		     
		   while (1) {
		      tecla = Teclado();
		      
		      if (timeoutt >= temporizador_timeout) {
			   estadoAtual = estado_DESATIVADO;
			   break;
		      }
		      
			 if (tecla < 10){
			   _delay_ms(500);
			   temp = temp + tecla;
			   itoa(tecla, y, 10); 
			   escreve_LCD(y);
			   break;
			 } 
		   }
		
		
		   while(1){
		     tecla = Teclado();
		      
		     if (timeoutt >= temporizador_timeout) {
			estadoAtual = estado_DESATIVADO;
			break;
		     }
		      
		     if(tecla == 14){
			timeoutt = 0;
			temporizador_timeout = temp;
			break;
		     } else if (tecla == 16){ //tecla R
			break;
		      }  
		   }
		   estadoAtual = estado_CONFIGURACAO;
		   break;
//-------------Fim do estado Temp Time Out---------------------------------------------------------------------------------

//-------------Inicio do estado Temp Sirene---------------------------------------------------------------------------------
		case estado_TEMP_SIRENE:
		   cmd_LCD(0x01,0);
		   escreve_LCD("TIMER SIRENE");
		   cmd_LCD(0xC0,0);
		   escreve_LCD("(000-999): ");
		   temp = NULL;
		   temp = capturaTempo(temporizador_timeout);

		   while(1){
		     tecla = Teclado();
		     
		     if (timeoutt >= temporizador_timeout) {
			estadoAtual = estado_DESATIVADO;
			break;
		    } 
		     
		     if(tecla == 14){
			temporizador_sirene = temp;
			break;
		     } else if (tecla == 16){ //tecla R
			break;
		      }  
		   }
		   estadoAtual = estado_CONFIGURACAO;
		   break;
//-------------Fim do estado Temp Sirene--------------------------------------------------------------------------------		   
		     	
		}
	
        _delay_ms(1000);
    }

}	

		