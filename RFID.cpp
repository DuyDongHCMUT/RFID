#include "RFID.h"


uint8_t SPI_MasterTransmit(uint8_t cData)
{
	SPDR = cData;
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}

MFRC522::MFRC522(int sad, int reset) {
	_sad = sad;
	sbi(DDRB, _sad);       // Set digital as OUTPUT to connect it to the RFID /ENABLE pin
	sbi(PORTB,_sad);

	_reset = reset;
	sbi(DDRB, _reset);       // Set digital as OUTPUT to connect it to the RFID /ENABLE pin
	sbi(PORTB,_reset);
}

uint8_t MFRC522::getFirmwareVersion() {
	uint8_t response;
	response = readFromRegister(0x37);
	return response;
}

void MFRC522::writeToRegister(uint8_t addr, uint8_t val) {
	cbi(PORTB,_sad);

	//Address format: 0XXXXXX0
	SPI_MasterTransmit((addr<<1)&0x7E);
//	send((addr<<1)&0x7E);
	SPI_MasterTransmit(val);
	//send(val);
	sbi(PORTB,_sad);
}

uint8_t MFRC522::readFromRegister(uint8_t addr) {
	uint8_t val;
	cbi(PORTB, _sad);
	SPI_MasterTransmit(((addr<<1)&0x7E) | 0x80);
	//send(((addr<<1)&0x7E) | 0x80);
	val =SPI_MasterTransmit(0x00);
	//send(val);
	sbi(PORTB,_sad);
	return val;
}

void MFRC522::reset() {
	writeToRegister(CommandReg, MFRC522_SOFTRESET);
}

void MFRC522::setBitMask(uint8_t addr, uint8_t mask) {
	uint8_t current;
	current = readFromRegister(addr);
	writeToRegister(addr, current | mask);
}

void MFRC522::clearBitMask(uint8_t addr, uint8_t mask) {
	uint8_t current;
	current = readFromRegister(addr);
	writeToRegister(addr, current & (~mask));
}

void MFRC522::begin() {
	sbi(PORTB,_sad);

	reset();

	//Timer: TPrescaler*TreloadVal/6.78MHz = 24ms == 3390 * 48 / 6.78
	writeToRegister(TModeReg, 0x8D);       // Tauto=1; f(Timer) = 6.78MHz/TPreScaler
	writeToRegister(TPrescalerReg, 0x3E);  // TModeReg[3..0] + TPrescalerReg
	writeToRegister(TReloadRegL, 30);		//48
	writeToRegister(TReloadRegH, 0);

	writeToRegister(TxAutoReg, 0x40);      // 100%ASK
	writeToRegister(ModeReg, 0x3D);        // CRC initial value 0x6363

	setBitMask(TxControlReg, 0x03);        // Turn antenna on.
}

int MFRC522::commandTag(uint8_t cmd, uint8_t *data, int dlen, uint8_t *result, int *rlen) {
	int status = MI_ERR;
	uint8_t irqEn = 0x00;
	uint8_t waitIRq = 0x00;
	uint8_t lastBits, n;
	int i;

	switch (cmd) {
		case MFRC522_AUTHENT:
		irqEn = 0x12;
		waitIRq = 0x10;
		break;
		case MFRC522_TRANSCEIVE:
		irqEn = 0x77;
		waitIRq = 0x30;
		break;
		default:
		break;
	}

	writeToRegister(CommIEnReg, irqEn|0x80);    // interrupt request
	clearBitMask(CommIrqReg, 0x80);             // Clear all interrupt requests bits.
	setBitMask(FIFOLevelReg, 0x80);             // FlushBuffer=1, FIFO initialization.

	writeToRegister(CommandReg, MFRC522_IDLE);  // No action, cancel the current command.

	// Write to FIFO
	for (i=0; i < dlen; i++) {
		writeToRegister(FIFODataReg, data[i]);
	}

	// Execute the command.
	writeToRegister(CommandReg, cmd);
	if (cmd == MFRC522_TRANSCEIVE) {
		setBitMask(BitFramingReg, 0x80);  // StartSend=1, transmission of data starts
	}

	// Waiting for the command to complete so we can receive data.
	i = 25; // Max wait time is 25ms.
	do {
		_delay_ms(5);
		// CommIRqReg[7..0]
		// Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
		n = readFromRegister(CommIrqReg);
		i--;
	} while ((i!=0) && !(n&0x01) && !(n&waitIRq));

	clearBitMask(BitFramingReg, 0x80);  // StartSend=0

	if (i != 0) { // Request did not time out.
		if(!(readFromRegister(ErrorReg) & 0x1D)) {  // BufferOvfl Collerr CRCErr ProtocolErr
			status = MI_OK;
			if (n & irqEn & 0x01) {
				status = MI_NOTAGERR;
			}

			if (cmd == MFRC522_TRANSCEIVE) {
				n = readFromRegister(FIFOLevelReg);
				lastBits = readFromRegister(ControlReg) & 0x07;
				if (lastBits) {
					*rlen = (n-1)*8 + lastBits;
					} else {
					*rlen = n*8;
				}

				if (n == 0) {
					n = 1;
				}

				if (n > MAX_LEN) {
					n = MAX_LEN;
				}

				// Reading the recieved data from FIFO.
				for (i=0; i<n; i++) {
					result[i] = readFromRegister(FIFODataReg);
				}
			}
			} else {
			status = MI_ERR;
		}
	}
	return status;
}

int MFRC522::requestTag(uint8_t mode, uint8_t *data) {
	int status, len;
	writeToRegister(BitFramingReg, 0x07);  // TxLastBists = BitFramingReg[2..0]

	data[0] = mode;
	status = commandTag(MFRC522_TRANSCEIVE, data, 1, data, &len);

	if ((status != MI_OK) || (len != 0x10)) {
		status = MI_ERR;
	}

	return status;
}

int MFRC522::antiCollision(uint8_t *serial) {
	int status, i, len;
	uint8_t check = 0x00;

	writeToRegister(BitFramingReg, 0x00);  // TxLastBits = BitFramingReg[2..0]

	serial[0] = MF1_ANTICOLL;
	serial[1] = 0x20;
	status = commandTag(MFRC522_TRANSCEIVE, serial, 2, serial, &len);
	len = len / 8; // len is in bits, and we want each byte.
	if (status == MI_OK) {
		// The checksum of the tag is the ^ of all the values.
		for (i = 0; i < len-1; i++) {
			check ^= serial[i];
		}
		// The checksum should be the same as the one provided from the
		// tag (serial[4]).
		if (check != serial[i]) {
			status = MI_ERR;
		}
	}

	return status;
}

void MFRC522::calculateCRC(uint8_t *data, int len, uint8_t *result) {
	int i;
	uint8_t n;

	clearBitMask(DivIrqReg, 0x04);   // CRCIrq = 0
	setBitMask(FIFOLevelReg, 0x80);  // Clear the FIFO pointer

	//Writing data to the FIFO.
	for (i = 0; i < len; i++) {
		writeToRegister(FIFODataReg, data[i]);
	}
	writeToRegister(CommandReg, MFRC522_CALCCRC);

	// Wait for the CRC calculation to complete.
	i = 0xFF;
	do {
		n = readFromRegister(DivIrqReg);
		i--;
	} while ((i != 0) && !(n & 0x04));  //CRCIrq = 1

	// Read the result from the CRC calculation.
	result[0] = readFromRegister(CRCResultRegL);
	result[1] = readFromRegister(CRCResultRegM);
}

uint8_t MFRC522::selectTag(uint8_t *serial) {
	int i, status, len;
	uint8_t sak;
	uint8_t buffer[9];

	buffer[0] = MF1_SELECTTAG;
	buffer[1] = 0x70;
	for (i = 0; i < 5; i++) {
		buffer[i+2] = serial[i];
	}
	calculateCRC(buffer, 7, &buffer[7]);

	status = commandTag(MFRC522_TRANSCEIVE, buffer, 9, buffer, &len);

	if ((status == MI_OK) && (len == 0x18)) {
		sak = buffer[0];
	}
	else {
		sak = 0;
	}

	return sak;
}

int MFRC522::haltTag() {
	int status, len;
	uint8_t buffer[4];

	buffer[0] = MF1_HALT;
	buffer[1] = 0;
	calculateCRC(buffer, 2, &buffer[2]);
	status = commandTag(MFRC522_TRANSCEIVE, buffer, 4, buffer, &len);
	clearBitMask(Status2Reg, 0x08);  // turn off encryption
	return status;
}