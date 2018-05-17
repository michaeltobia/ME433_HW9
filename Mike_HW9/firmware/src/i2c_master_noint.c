#include<xc.h>
#include "i2c_master_noint.h"

// I2C Master utilities, 100 kHz, using polling rather than interrupts
// The functions must be callled in the correct order as per the I2C protocol
// Change I2C1 to the I2C channel you are using
// I2C pins need pull-up resistors, 2k-10k

void i2c_master_setup(void) {
  I2C2BRG = 53;                     // I2CBRG = [1/(2*Fsck) - PGD]*Pbclk - 2
                                    // FPGD = 104 ns, PBCLK = 48MHz, FSCK = 400kHz
  ANSELBbits.ANSB2 = 0;             // turn off analog pin at B2
  ANSELBbits.ANSB3 = 0;             // turn off analog pin at B3
  I2C2CONbits.ON = 1;               // turn on the I2C2 module
  
}

// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C2CONbits.SEN = 1;            // send the start bit
    while(I2C2CONbits.SEN) { ; }    // wait for the start bit to be sent
}

void i2c_master_restart(void) {
    I2C2CONbits.RSEN = 1;           // send a restart
    while(I2C2CONbits.RSEN) { ; }   // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
  I2C2TRN = byte;                   // if an address, bit 0 = 0 for write, 1 for read
  while(I2C2STATbits.TRSTAT) { ; }  // wait for the transmission to finish
  if(I2C2STATbits.ACKSTAT) {        // if this is high, slave has not acknowledged
      LATAbits.LATA4 = 0;           // turn off LED if send fails
  }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C2CONbits.RCEN = 1;             // start receiving data
    while(!I2C2STATbits.RBF) { ; }    // wait to receive the data
    return I2C2RCV;                   // read and return the data
}

void i2c_master_ack(int val) {        // sends ACK = 0 (slave should send another byte)
                                      // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val;          // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;            // send ACKDT
    while(I2C2CONbits.ACKEN) { ; }    // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // send a STOP:
  I2C2CONbits.PEN = 1;                // comm is complete and master relinquishes bus
  while(I2C2CONbits.PEN) { ; }        // wait for STOP to complete
}

void I2C_read_multiple(unsigned char address, unsigned char reg_add, unsigned char * data, int length){
    int i = 0;
    i2c_master_start();
    i2c_master_send(address << 1 | 0);
    i2c_master_send(reg_add);
    i2c_master_restart();
    i2c_master_send(address << 1 | 1);
    for(i = 0; i < length; i++){
        data[i] = i2c_master_recv();
        if(i == length-1){
            break;
        }
        i2c_master_ack(0);
    }
    i2c_master_ack(1);
    i2c_master_stop();
    return;
}

void initIMU(void) {
    
    
    // Set CTRL1_XL register (accelerometer)
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1 | 0);
    i2c_master_send(0x10);
    // 0b10000010 ([7:4] 1.66kHz samp rate, [3:2] +-2g sens, [1:0] 100 Hz filter)
    i2c_master_send(0x82);
    i2c_master_stop();
    
    // Set CTRL2_G register (gyroscope)
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1 | 0);
    i2c_master_send(0x11);
    // 0b10001000 ([7:4] 1.66kHz samp rate, [3:2] 1000 dps scale, [1:0] 00)
    i2c_master_send(0b10001000);
    i2c_master_stop();
    
    // Set CTRL3_C register (IF_INC = 1, Register Increment)
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1 | 0);
    i2c_master_send(0x12);
    // 0b00000100
    i2c_master_send(0b00000100);
    i2c_master_stop();
}

unsigned char getReg(char reg_add) {
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1 | 0);
    i2c_master_send(reg_add);
    i2c_master_restart();
    i2c_master_send(SLAVE_ADDR << 1 | 1);
    unsigned char received = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    
    return received;
}