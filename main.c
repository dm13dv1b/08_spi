#include <stm32f4xx.h>
#include <stm32f4xx_spi.h>

uint8_t received_val = 0, i;

#define SD_CS_LOW()     GPIO_ResetBits(GPIOE, GPIO_Pin_7)
#define SD_CS_HIGH()    GPIO_SetBits(SGPIOE, GPIO_Pin_7)

// this function initializes the SPI2 peripheral

void init_SPI2(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* configure pins used by SPI2
	 * PB13 = SCK
	 * PB14 = MISO
	 * PB15 = MOSI
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// connect SPI2 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	/* Configure the chip select pin
	   in this case we will use PD8 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	GPIOD->BSRRH |= GPIO_Pin_8; // set PE7 (CS) low	
	
	// enable peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	/* configure SPI2 in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128; // SPI frequency is APB2 frequency / 256 = 328,125 hz = 328 kHz
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI2, &SPI_InitStruct); 
	
	SPI_Cmd(SPI2, ENABLE); // enable SPI2
}

/* This funtion is used to transmit and receive data 
 * with SPI2
 * 			data --> data to be transmitted
 * 			returns received value
 */
uint8_t SPI2_send(uint8_t data){

	SPI2->DR = data; // write data to be transmitted to the SPI data register
	while( !(SPI2->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	while( !(SPI2->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	while( SPI2->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	return SPI2->DR; // return received data from SPI data register
}

uint8_t sdSpiByte(uint8_t data)
{
	SPI2->DR = data;
	while( !(SPI2->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	while( !(SPI2->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	while( SPI2->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	return SPI2->DR;
}

uint8_t sdCrc7(uint8_t* chr, uint8_t cnt, uint8_t crc)
{
	uint8_t i, a;
	uint8_t Data;
	for (a=0; a<cnt; a++)
	{
		Data = chr[a];
		for (i=0; i<8; i++)
		{
			crc <<=1;
			if ((Data & 0x80) ^ (crc & 0x80))
			{crc ^=0x09;}
			Data <<=1;
		}
	}
	return crc & 0x7F;
}

void sdSendCommand(uint8_t cmd, uint32_t param)
{
	uint8_t send[6];
	
	send[0] = cmd | 0x40;
	send[1]	= param >> 24;
	send[2]	= param >> 16;
	send[3]	= param >> 8;
	send[4]	= param ;
	send[5] = (sdCrc7(send, 5, 0) << 1) | 1;
	for (cmd = 0; cmd < sizeof(send); cmd++)
	{
		SPI2_send(send[cmd]);
	}
}

uint8_t sdReadResp(void)
{
  uint8_t Data = 0;
  
  /*!< Wait until the transmit buffer is empty */
  while( !(SPI2->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete

  /*!< Send the byte */
  SPI2_send(0xFF);

  /*!< Wait until a data is received */
	while( !(SPI2->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete

		/*!< Get the received data */
  Data = SPI2->DR;

  /*!< Return the shifted data */
  return Data;
}

uint8_t sdCommandAndResponse(uint8_t cmd, uint32_t param)
{
	uint8_t ret, retry;
	retry = 0;
	SPI2_send(0xFF);
	sdSendCommand(cmd, param);
	while ((ret = SPI_I2S_ReceiveData(SPI2)) == 0xFF)
	{
		SPI2_send(0xFF);
		if (retry++ > 100) { break; }
	}
	return ret;
}

void fatal(uint8_t val)
{
	while(1);
}

int main(void){
	
	uint8_t v;
	
	init_SPI2();
	
	GPIOD->BSRRL |= GPIO_Pin_8; // set PE7 (CS) high	
	
	for (v=0; v<30; v++)
		{
			SPI2_send(0xFF);  // transmit data
		}
		
	GPIOD->BSRRH |= GPIO_Pin_8; // set PE7 (CS) low
		
	v = sdCommandAndResponse(0, 0);
		while ( (received_val = SPI_I2S_ReceiveData(SPI2)) != 1 )
		{
			SPI2_send(0xFF);
		}
	if (v!=1) {fatal(2);}
	v = sdCommandAndResponse(1, 0);
		if (v!=1) {fatal(2);}	
	

	while(1)
	{
		SPI2_send(0xA1);  // transmit data
	}		
}
