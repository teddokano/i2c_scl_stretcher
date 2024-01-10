#include "mbed.h"

// #pragma GCC optimize ("O3")

/*
 *  Constants
 */

constexpr int stretch_byte = 1;
constexpr int stretch_bit = 6;


constexpr int SDA_PIN = 0;
constexpr int SCL_PIN = 1;

DigitalInOut sda_pin(I2C_SDA);
DigitalInOut scl_pin(I2C_SCL);


DigitalOut vital(D8);
DigitalOut led(LED1);

constexpr int TRANSACTION_BUFFER_DEPTH = 32;
constexpr int TRANSACTION_MAX_BYTE_LENGTH = 128;
constexpr int CAPTURE_LENGTH = 10;

enum pa_status : int {
	FREE = 0,
	START,
};

/*
 *  Allocating static (global) memory for data capturing
 */

typedef struct {
	uint8_t data;
	uint8_t ack;
} data_ack;

typedef struct {
	data_ack data_byte[TRANSACTION_MAX_BYTE_LENGTH];
	int length;
	int repeated_start;
	int stop;
} transaction;

transaction tr[TRANSACTION_BUFFER_DEPTH];
volatile int transaction_count = 0;

volatile int captured = 0;

int prev_sda = 1;
int prev_scl = 1;
int total_count = 0;

/*
 *  Prototypes
 */

inline void pin_state_change(int sda, int startstop = false, int stretch = false);
void show_transactions(int length);
void clock_stretch( int wait );

#define SAMPLINF_MONITOR_PERIOD 0xF

void core1_main();
inline void pin_state_change(int sda, int ss);
void show_transactions(int length);

/*
 *  Program body
 */

int main() {
	printf("I2C protocol analyzer started\n");
	printf("  transaction captureing depth	= %6d\n", TRANSACTION_BUFFER_DEPTH);
	printf("  transaction maximum byte length = %6d\n", TRANSACTION_MAX_BYTE_LENGTH);
	printf("  memory size for data capturing  = %6d\n", sizeof(tr));
	printf("[%d] captureing transactions\n", total_count++);

	core1_main();
}

void core1_main() {
	int all;
	int sda;
	int scl;
	int count = 0;
	int toggle = false;

	scl_pin.input();
	sda_pin.input();

	while (true) {
		sda = sda_pin.read();
		scl = scl_pin.read();

		if ((prev_sda != sda) && (prev_scl && scl))
			pin_state_change(sda, true, false);
		else if (!prev_scl && scl)
			pin_state_change(sda, false, false);
		else if (prev_scl && !scl)
			pin_state_change(sda, false, true);

		prev_sda = sda;
		prev_scl = scl;

		if (!(count++ & SAMPLINF_MONITOR_PERIOD))
			vital = toggle = !toggle;
	}
}

inline void pin_state_change( int sda, int ss, int stretch ) {
	static pa_status state = FREE;
	static int bit_count;
	static int byte_count;
	static transaction *t;
	static int repeated_start_flag = 0;

    if ( stretch ) {
        if ( (byte_count == stretch_byte) && (bit_count == stretch_bit) )
            clock_stretch( 100 );
        
        return;
    }

	if (ss) {
		if (sda) {
			state = FREE;

			(tr + transaction_count)->stop = true;

			if (CAPTURE_LENGTH < transaction_count) {
				show_transactions(CAPTURE_LENGTH);
				transaction_count = 0;
				printf("[%d] captureing %d transactions\n", total_count++, CAPTURE_LENGTH);
			} else {
				transaction_count++;
			}
		} else {
			if (FREE != state) {
				transaction_count++;
				transaction_count %= TRANSACTION_BUFFER_DEPTH;

				(tr + transaction_count)->repeated_start = true;
			} else {
				(tr + transaction_count)->repeated_start = false;
			}

			state = START;
			bit_count = 0;
			byte_count = 0;

			t = tr + transaction_count;
			t->stop = false;
		}

		led = state;

		return;
	}

	if (START == state) {
		switch (bit_count) {
			case 8:
				t->data_byte[byte_count].ack |= sda;
				byte_count++;
				t->length = byte_count;
				break;
			case 0:
				t->data_byte[byte_count].data = (sda & 0x1) << 7;
				t->data_byte[byte_count].ack = repeated_start_flag;
				break;
			default:
				t->data_byte[byte_count].data |= (sda & 0x1) << (7 - bit_count);
				break;
		}
		bit_count++;
		bit_count %= 9;
	}
}

void clock_stretch( int wait )
{
    scl_pin.write( 0 );
    scl_pin.output();
    wait_us( wait );
    scl_pin.input();
}

void show_transactions(int length) {
	static uint32_t tr_num = 0;
	transaction *t;
	data_ack *addr;

	for (int i = 0; i < length; i++) {
		t = tr + i;
		addr = &(t->data_byte[0]);

		printf("#%5u (%2d) : [%c]", tr_num, t->length - 1, t->repeated_start ? 'R' : 'S');
		printf(" 0x%02X-%c[%c]", addr->data & ~0x01, (addr->data) & 0x01 ? 'R' : 'W', addr->ack ? 'N' : 'A');

		for (int j = 1; j < t->length; j++)
			printf(" 0x%02X[%c]", t->data_byte[j].data, t->data_byte[j].ack ? 'N' : 'A');

		printf("%s\n", t->stop ? " [P]" : "");

		tr_num++;
	}
}
