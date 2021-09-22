/**************************************************************
 Multi Interface PIC board
  for PIC12F1822
 **************************************************************
 
---------------------------------------------------------------
Pin assignment(PIC12F1822,8P)
 7:RA0:O: TXD
 6:RA1:I: RXD
 5:RA2:A: Sensor INPUT (ADC)
 4:RA3:O:
 3:RA4:O:
 2:RA5:O: TX-RX serialization
 8:VSS:P:
 1:VDD:P:
---------------------------------------------------------------
 */

#include <xc.h>
#include <htc.h>
typedef unsigned char byte; // 8-bit

void InitializeUSART(void);

// PROTYPE DECLARATION
void COMM_RS485(void);
void put_UART(byte data);
void Send_RS485(byte CMND, byte ID, byte LEN);
byte get_UART(void);
byte Rcv_RS485(void);

// Definition
#define uart_ptr_max 10
#define Reset_RS485 0
#define Ping 1
#define Read 2
#define Write 3
#define ACK 0x41
#define NAK 0x4E
#define myID 1  //  ＊＊＊スレーブを変更するときのこのIDも変更してください＊＊＊＊

//unsigned char input_buffer[16];  // USB command buffer
//unsigned char output_buffer[16];  // USB reply buffer
unsigned char UART_rd_buffer[uart_ptr_max]; // UART receive buffer (MASTER --> SLAVE)
unsigned char UART_wr_buffer[uart_ptr_max]; // UART send buffer (SLAVE --> MASTER)
unsigned char uart_wr_ptr, uart_rd_ptr, uart_buf_bytes, n_uart_wrbuf, uart_rcv_complete;
unsigned char a;
unsigned char uart_rd_ptr, LEN;
int TMR0L, TMR0H, TMR0L1, TMR0H1, cnt01, cnt0;
unsigned char CMND, SID, ID, msg[8], FF_mode;

// PIC12F1822 Configuration Bit Settings
// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)
// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

void __interrupt() isr(void) {

    if (RCIF) {     // USART Interrupt?
        a = RCREG;  // Receive Register
        UART_rd_buffer[uart_rd_ptr] = a;
        uart_rd_ptr++;
        uart_buf_bytes++;

        if (FF_mode == 0) {
            if (a == 0xFF) {
                FF_mode = 1;
            } else {
                uart_rd_ptr = 0;
                uart_buf_bytes = 0;
            }

        } else if (FF_mode == 1) {
            if (a == 0xFF) {
                FF_mode = 2;
            } else {
                uart_rd_ptr = 1;
                uart_buf_bytes = 1;
            }

        } else if (FF_mode == 2) {
            if (uart_rcv_complete == 0) {
                if (uart_buf_bytes == 5) LEN = a;
                if (uart_buf_bytes >= LEN + 5) {
                    uart_rcv_complete = 1;
                    RA4 = 1;
                    uart_rd_ptr = 0;
                    FF_mode = 0;
                }
            }
        }

        if (OERR) {     // Over Run Error
            CREN = 0;   // Disable Receiver
            CREN = 1;   // Enable  Receiver           
        }
        RCIF = 0;
    }
    CREN = 1;
    RCIE = 1;
    PEIE = 1;
    INTCONbits.GIE = 1; // Level Permission

}

void COMM_RS485(void) {
    static unsigned char devaddr, regaddr, i2cdata, i2cdata2;
    unsigned char N, i, j, k, ch, I2Cdata2, ASD_type, ASD_num;
    unsigned char RSP, Resp, data;

    // Push button Switch interpreter

    // RS485 command interpreter
    //
    //  Packet format ( RS485 network MASTER <-> SLAVE )
    //            aa[0]    aa[1]   aa[2]    aa[3]   aa[4]     aa[2+LEN]
    //   | ID  |  LEN   |  CMND   |  PRM0  | PRM1 | ------- | PRMx   | 
    //                     0x00 : RESET
    //                     0x01 : PING
    //                     0x02 : READ
    //                     0x03 : WRITE
    //  RS485 Command state machine
    //     Send Command  PC>MASTER>SLAVE :
    //     Receive Reply SLAVE>MASTER>PC :
    //

    // RS485 Ack packet proccessor
    if (uart_buf_bytes > 0) {
    }

    if (uart_rcv_complete) {
        Resp = Rcv_RS485();
        RA4 = 0;
        for (i = 0; i < 10; i++) {
            for (j = 0; j < 10; j++);
        }

        //  Send_RS485(Resp,ID,LEN);
        if ((ID == 0xFE) || (ID == myID)) {
            switch (CMND) {

                case Reset_RS485:       // 0
                    if (ID == myID) {
                        if (Resp == 0) {
                            RSP = 0x41; // ACK
                        } else {
                            RSP = 0x4E; // NAK
                        }
                        Send_RS485(RSP, ID, LEN);
                    }
                    break;

                case Ping:              // 1
                    if (ID == myID) {
                        if (Resp == 0) {
                            RSP = 0x41; // ACK
                        } else {
                            RSP = 0x4E; // NAK
                        }
                        Send_RS485(RSP, ID, LEN);
                    }
                    break;

                case Read:                 // 2
                    if (ID == myID) {
                        if (Resp == 0) {
                            ADCON0 = 0x0B; // ADC=AN2, GODONE, ADC ON
                            while ((ADCON0 & 0x02) == 1); // ADC Complete?
                            UART_rd_buffer[5] = ADRESH;
                            UART_rd_buffer[6] = ADRESL;
                            RSP = 0x41; // ACK
                        } else {
                            RSP = 0x4E; // NAK
                        }
                        Send_RS485(RSP, ID, LEN);
                    }
                    break;

                case Write:             // 3
                    if (ID == myID) {
                        if (Resp == 0) {
                            RSP = 0x41; // ACK
                        } else {
                            RSP = 0x4E; // NAK
                        }
                        Send_RS485(RSP, ID, LEN);
                    }
                    break;
            } // switch   
        } // if ((ID == 0xFE) || (ID == myID))
    } //     if (uart_rcv_complete)
    
    for (i = 0; i < 10; i++) {
        for (j = 0; j < 10; j++);
    }

    // UART TX serialization
    if (TXSTAbits.TRMT == 1) { // TSR Empty
        if (n_uart_wrbuf == 0) {
            RA5 = 0; // RX Mode
        } else {
            RA5 = 1; // TX Mode
        }
        if (n_uart_wrbuf != 0) {
            TXREG = UART_wr_buffer[uart_wr_ptr];
            uart_wr_ptr++;
            if (uart_wr_ptr > uart_ptr_max) uart_wr_ptr = 0;
            n_uart_wrbuf--;
            if (n_uart_wrbuf == 0) uart_wr_ptr = 0;
        }
    }

} //end of COMM_RS485

void InitializeUSART(void) {
    char c;
    //      UART_TRISRx=1;    // RX
    //      UART_TRISTx=0;    // TX
    TXSTA = 0x24; // TX enable,BRGH=1
    RCSTA = 0x90; // SP enable,Single Character RX
    // SPBRG = 0x67;
    // SPBRGH = 0x00; // 0x067 for 16MHz -> 19200 baud
    SPBRGL = 0xA0;
    SPBRGH = 0x01; // 0x1A0 for 16MHz -> 9600 baud
    BAUDCON = 0x08; // BRG16 = 1
    //        c = RCREG;    // read 
}

void put_UART(byte data) {
    UART_wr_buffer[uart_wr_ptr] = data;
    uart_wr_ptr++;
    n_uart_wrbuf++;
}

void Send_RS485(byte CMND, byte ID, byte n) {
    byte i, j, data, chksum;
    chksum = 0;

    RA5 = 1; // Change LTC485 to TX mode
    RA4 = 1;
    put_UART(0xFF);
    put_UART(0xFF);
    put_UART(CMND);
    chksum += CMND;
    put_UART(ID);
    chksum += ID;
    put_UART(n);
    chksum += n;
    for (i = 0; i < n; i++) {
        data = UART_rd_buffer[i + 5];
        put_UART(data);
        chksum += data;
    }
    put_UART(chksum & 0xFF);
    uart_wr_ptr = 0;
    
    for (i = 0; i < 10; i++) {
        for (j = 0; j < 10; j++);
    }
}

byte get_UART(void) {
    byte data;

    if (uart_buf_bytes > 0) {
        data = UART_rd_buffer[uart_rd_ptr];
        uart_rd_ptr++;
        uart_buf_bytes--;
        return data;
    } else {
        return 0xFF;
    }
}

byte Rcv_RS485(void) {
    byte i, ret, data;
    ret = 0;
    uart_rd_ptr = 0;
    data = get_UART();

    if (data != 0xFF) {
        while (data != 0xFF) {
            data = get_UART();
        }
    }

    data = get_UART();
    if (data != 0xFF) {
        uart_rd_ptr = 0;
        uart_buf_bytes = 0;
        uart_rcv_complete = 0;
        return 1;
    }

    data = get_UART();

    if ((0 <= data)&&(data < 5)) {
        CMND = data;
    } else {
        uart_rd_ptr = 0;
        uart_buf_bytes = 0;
        uart_rcv_complete = 0;
        return 3;
    }

    ID = get_UART();
    a = get_UART();
    for (i = 0; i < LEN + 4; i++) {
        UART_wr_buffer[i] = UART_rd_buffer[i];
        msg[i] = data;
    }

    uart_rd_ptr = 0;
    uart_buf_bytes = 0;
    uart_rcv_complete = 0;

    return 0;
}

void main() {
    unsigned short LED;
    int i, j, k;

    OPTION_REG = 0x80; // prescaler 1:2
    OSCCON = 0x7A; // 16MHz
    APFCON = 0x00; // TXD(RA0) , RXD(RA1)
    ANSELA = 0x04; // AN2(RA2)
    CCP1CON = 0x00; // Capture , Compare , PWM disabled
    TRISA = 0x06; // RXD(RA1) , AN2(RA2)
    PORTA = 0x00;

    TMR0 = 0;
    INTCON = 0x00; // GIE:1,T0IE:1
    ANSELA = 0x04; // RA2=Analog Input
    ADCON0 = 0x09; // AN2 , GO , ADC_ON
    ADCON1 = 0x00; // ADFM:Left , FOSC/2 , VREF=VDD
    ADGO = 0;      // AD Conversion Gone

    InitializeUSART();

    uart_rcv_complete = 0;
    RCIE = 1;
    PEIE = 1;
    GIE = 1;
    RA5 = 1; // Change LTC485 to TX mode
    RA5 = 0; // Change LTC485 to RX mode
    RA4 = 0;
    LEN = 3;
    FF_mode = 0;

    while (1) {
        COMM_RS485();
        for (i = 0; i < 10; i++) {
            for (j = 0; j < 10; j++);
        }
    }
} // end of main
