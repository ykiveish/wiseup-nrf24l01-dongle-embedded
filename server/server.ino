#include <SoftwareSerial.h>
#include <SPInterface.h>
#include <nRFModule.h>
#include <wise_rfcomm.h>
#include <wise_common.h>

#define UART_REQUEST_DONGLE_STATE           0x5
#define UART_RESPONSE_DONGLE_STATE          0x6
#define UART_REQUEST_CHECK_DEVICE_ID        0x9
#define UART_RESPONSE_CHECK_DEVICE_ID       0x10
#define UART_REQUEST_SEND_SENSOR_DATA       0x11

#define UART_WISEUP_DEVICE                  0xFABA
#define UART_MAGIC_NUMBER                   0xAD

const byte rxPin = 2;
const byte txPin = 3;

uint8_t rx[32];
uint8_t tx[32];

SoftwareSerial serialDebug (rxPin, txPin);
spi_interface 	spi	  = spi_interface();
NRF24L01    	nrf	  = NRF24L01(&spi);

uint8_t BROADCAST_ADDR[5] = {0xFA, 0xFA, 0xFA, 0xFA, 0xFA};
uint8_t LOCAL_ADDR[5] 	  = {0x01, 0x02, 0x03, 0x04, 0x05};
uint8_t dest_addr[5] 	  = {0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t check_uuid[5]	  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

long long send_address_timeout = 0;

enum STATES {
	SEND_ADDRESS,
	WORKING,
    UART,
	IDLE,
	SLEEPING,
	CHECK_FOR_ID
};
enum STATES state = IDLE;

enum UART_STATES {
        UART_IDLE,
        MAGIC,
        COMMAND,
        LENGTH,
        DATA
};
enum UART_STATES uart_state = MAGIC;

uint8_t data_byte;
uint8_t command;
uint8_t length;
uint8_t uart_buffer[32];
uint8_t buffer_uart_index = 0;
uint8_t uart_timeslotes = 0;

void
hardware_layer_data_arrived_handler () {
	rfcomm_data * packet = (rfcomm_data *) rx;
	serialDebug.print ("# [server] Data Arrived ... ");

	// print_buffer (rx, 32);
    for (uint8_t i = 0; i < 32; i++) {
        serialDebug.print(rx[i]);
        serialDebug.print(" ");
    } serialDebug.println ();

	switch (packet->data_information.data_type) {
        case CLIENT_ID_REQUEST: {
            serialDebug.println ("# [server] CLIENT_ID_REQUEST");
            state = CHECK_FOR_ID;
			memcpy (check_uuid, &packet->data_frame, 5);
			break;
        }
        case CLIENT_SENSOR_DATA: {
            serialDebug.println ("# [server] CLIENT_SENSOR_DATA");
            uart_buffer[0] = UART_MAGIC_NUMBER;
            uart_buffer[1] = UART_REQUEST_SEND_SENSOR_DATA;
            uart_buffer[2] = 9;
            memcpy (&uart_buffer[3], &packet->data_frame, 9);
            for (uint8_t i = 0; i < 12; i++) {
                Serial.print((char)uart_buffer[i]);
            }
			break;
        }
		default:
			break;
	}
}

void setup () {
	Serial.begin(9600);
    serialDebug.begin(9600);
	nrf.init (10, 9); // default csn = 7, ce = 8 (10, 9 on imall board)
	nrf.setSourceAddress		((byte *) LOCAL_ADDR);
	nrf.setDestinationAddress	((byte *) BROADCAST_ADDR);

	nrf.setPayload (16);
	nrf.setChannel (99);
	nrf.setSpeedRate (NRF_250KBPS);
	nrf.configure ();

	nrf.rx_buffer_ptr = rx;
	nrf.tx_buffer_ptr = tx;
	nrf.dataRecievedHandler = hardware_layer_data_arrived_handler;
    
	serialDebug.println("# [server] Initialized ...");
}

void
uard_data_handler () {
    serialDebug.println("# [server] uard_data_handler ...");
    
    switch (command) {
        case UART_REQUEST_DONGLE_STATE: {
            serialDebug.println("# [server] UART_REQUEST_DONGLE_STATE");
            memcpy (uart_buffer, 0x0, 32);
            uart_buffer[0] = UART_MAGIC_NUMBER;
            uart_buffer[1] = UART_RESPONSE_DONGLE_STATE;
            uart_buffer[2] = 2;
            uart_buffer[3] = 0xFA;
            uart_buffer[4] = 0xBA;            
            for (uint8_t i = 0; i < 5; i++) {
                Serial.print((char)uart_buffer[i]);
            }
            break;
        }
        case UART_RESPONSE_CHECK_DEVICE_ID: {
            serialDebug.println("# [server] UART_RESPONSE_CHECK_DEVICE_ID");
            break;
        }
    }
}

void loop () {
	switch (state) {
		case SEND_ADDRESS: {
			rfcomm_data * packet = (rfcomm_data *) tx;
			memcpy (dest_addr, BROADCAST_ADDR, 5);
			nrf.setDestinationAddress ((byte *) dest_addr);

			build_packet (tx, SERVER_ADDRESS_BROADCAST);
			memcpy (&packet->data_frame, LOCAL_ADDR, 5);

			nrf.send ();
			send_address_timeout = millis ();
			state = WORKING;
			serialDebug.println("# [server] SEND_ADDRESS");
			break;
		}
		case WORKING: {
            state = UART;
			nrf.pollListener ();
            
			if (abs (millis () - send_address_timeout > 5000)) {
				state = SEND_ADDRESS;
			}
			break;
		}
        case UART: {
            if (Serial.available() > 0) {
                data_byte = Serial.read();
                switch (uart_state) {
                    case UART_IDLE: {
                        serialDebug.println("# [server] UART_IDLE");
                        uart_state = MAGIC;
                        break;
                    }
                    case MAGIC: {
                        serialDebug.println("# [server] MAGIC");
                        if (data_byte == 0xAD) {
                            uart_state = COMMAND;
                        }
                        break;
                    }
                    case COMMAND: {
                        serialDebug.println("# [server] COMMAND");
                        command = data_byte;
                        uart_state = LENGTH;
                        break;
                    }
                    case LENGTH: {
                        serialDebug.println("# [server] LENGTH");
                        length = data_byte;
                        if (length == 0) {
                            uart_state = MAGIC;
                            uard_data_handler ();
                        } else {
                            uart_state = DATA;
                        }
                        break;
                    }
                    case DATA: {
                        serialDebug.println("# [server] DATA");
                        uart_buffer[buffer_uart_index] = data_byte;
                        if (buffer_uart_index + 1 == length) {
                            buffer_uart_index = 0;
                            uart_state = MAGIC;
                            uard_data_handler ();
                        }
                        buffer_uart_index++;
                        break;
                    }
                }
            }
            
            if (uart_timeslotes == 10) {
                uart_timeslotes = 0;
                state = WORKING;
            } else {
                uart_timeslotes++;
            }
			break;
		}
		case IDLE:
			state = UART;
			serialDebug.println("# [server] IDLE");
			break;
		case SLEEPING:
			break;
		case CHECK_FOR_ID: {
            serialDebug.println("# [server] CHECK_FOR_ID");
            memcpy (uart_buffer, 0x0, 32);
            uart_buffer[0] = UART_MAGIC_NUMBER;
            uart_buffer[1] = UART_REQUEST_CHECK_DEVICE_ID;
            uart_buffer[2] = 4;
            memcpy (&uart_buffer[3], check_uuid, 4);
            for (uint8_t i = 0; i < 7; i++) {
                Serial.print((char)uart_buffer[i]);
            }
            state = WORKING;
			break;
        }
	}

	delay (100);
}
