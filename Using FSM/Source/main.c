#include "integer.h"
#include <MKL25Z4.h>
#include "spi_io.h"
#include "sd_io.h"
#include "sd_server.h"
#include "LEDs.h"
#include "debug.h"

#define NUM_SECTORS_TO_READ (100)

SD_DEV dev[1];          // SD device descriptor
uint8_t buffer[512];    // Buffer for SD read or write data

void Task_Makework(){
	static int n=2;
	static int done = 0;
	static double my_pi=3.0;
	double term, prev_pi=0.0;
	
	// set task debug bit
	PTB->PSOR = MASK(DBG_7);
	// Nilakantha Series to approximate pi
	if (!done) {
		term = 4.0/(n*(n+1.0)*(n+2.0));
		if (n&4) { // is multiple of four
			term = -term;
		}
		prev_pi = my_pi;
		my_pi += term;
		if (my_pi != prev_pi) {
			n += 2;
		} else {
			// Done with approximating pi
			done = 1; 
		}
	}	
	PTB->PCOR = MASK(DBG_7);
	// clear task debug bit
}

void Task_Test_SD(void) {
	// Write test data to given block (sector_num) in flash. 
	// Read it back, compute simple checksum to confirm it is correct.
	static enum {S_INIT, S_INIT_WAIT, S_TEST_READ, S_TEST_READ_WAIT, 
		S_TEST_WRITE, S_TEST_WRITE_WAIT, S_TEST_VERIFY, S_TEST_VERIFY_WAIT,
		S_ERROR} next_state = S_INIT;
	static int i;
	static DWORD sector_num = 0, read_sector_count=0; 
	static uint32_t sum=0;
	//	static char err_color_code = 0; // xxxxxRGB
	PTB->PSOR = MASK(DBG_6);
	switch (next_state) {
		case S_INIT:
			// wait until SD server is idle
			if (g_trans.Status == STAT_IDLE) {
				// Common settings for all transactions for this task
				g_trans.Device = dev;
				g_trans.Data = buffer;
				// request SD card initialization
				g_trans.Request = REQ_INIT;
				next_state = S_INIT_WAIT;
			}
			break;
		case S_INIT_WAIT:
			if ((g_trans.Status == STAT_IDLE) && (g_trans.Request == REQ_NONE)) {
				if (g_trans.ErrorCode == SD_OK) {
					Control_RGB_LEDs(0, 1, 1); // Cyan: initialized OK
					next_state = S_TEST_READ;
				} else {
					next_state = S_ERROR;
				}
			} // else keep waiting in this state, since server not done
			break;
		case S_TEST_READ:
			// wait until SD server is idle
			if (g_trans.Status == STAT_IDLE) {
				// erase buffer
				for (i=0; i<SD_BLK_SIZE; i++)
					buffer[i] = 0;
				// request SD card read
				g_trans.Sector = sector_num;
				g_trans.Request = REQ_READ;				
				next_state = S_TEST_READ_WAIT;
			}
			break;
		case S_TEST_READ_WAIT:
			if ((g_trans.Status == STAT_IDLE) && (g_trans.Request == REQ_NONE)) {
				if (g_trans.ErrorCode == SD_OK) { // Read was OK
					Control_RGB_LEDs(0, 0, 1); // Blue: Read OK
					if (++read_sector_count < NUM_SECTORS_TO_READ) {
						next_state = S_TEST_READ;
					} else {
						next_state = S_TEST_WRITE;
						read_sector_count = 0;
						sector_num++; // Advance to next sector
					}
				} else {
					next_state = S_ERROR;
				}
			} // else keep waiting in this state, since server not done
			break;
		case S_TEST_WRITE:
			// wait until SD server is idle
			if (g_trans.Status == STAT_IDLE) {
				// Initialize data buffer
				for (i=0; i<SD_BLK_SIZE; i++)
					buffer[i] = 0;
				// Load sample data into buffer
				*(uint64_t *)(&buffer[0]) = 0xFEEDDC0D;
				*(uint64_t *)(&buffer[508]) = 0xACE0FC0D;
				// Write the data into given sector
				// request SD card write
				g_trans.Sector = sector_num;
				g_trans.Request = REQ_WRITE;				
				next_state = S_TEST_WRITE_WAIT;
			}
			break;
		case S_TEST_WRITE_WAIT:			
			if ((g_trans.Status == STAT_IDLE) && (g_trans.Request == REQ_NONE)) {
				if (g_trans.ErrorCode == SD_OK) {
					Control_RGB_LEDs(1, 0, 1);// Magenta: Wrote OK
					next_state = S_TEST_VERIFY;
				} else {
					next_state = S_ERROR;
				}
			} // else keep waiting in this state, since server not done
			break;
		case S_TEST_VERIFY:
			// wait until SD server is idle
			if (g_trans.Status == STAT_IDLE) {
				// erase buffer
				for (i=0; i<SD_BLK_SIZE; i++)
					buffer[i] = 0;
				// request SD card read
				g_trans.Sector = sector_num;
				g_trans.Request = REQ_READ;				
				next_state = S_TEST_VERIFY_WAIT;
			}
			break;
		case S_TEST_VERIFY_WAIT:
			if ((g_trans.Status == STAT_IDLE) && (g_trans.Request == REQ_NONE)) {
				if (g_trans.ErrorCode == SD_OK) { // Read was OK
					Control_RGB_LEDs(0, 0, 1);// Blue: Read OK
					for (i = 0, sum = 0; i < SD_BLK_SIZE; i++)
						sum += buffer[i];			
					if (sum == 0x0569) { // Checksum is OK
						Control_RGB_LEDs(1, 1, 1); // White: Checksum OK
						next_state = S_TEST_READ;
						sector_num++; // Advance to next sector
					} else {
						next_state = S_ERROR;
					}
				}
			} // else keep waiting in this state, since server not done
			break;
		default:
		case S_ERROR:
			Control_RGB_LEDs(1,0,0);
			while (1)
				;
			break;
	}
	PTB->PCOR = MASK(DBG_6);
	// clear task debug bit
}

void Scheduler(void) {
	while (1) {
		
		Task_SD_Server();
		Task_Test_SD();
		Task_Makework();
		
	}
}

int main(void) {
	Init_Debug_Signals();
	Init_RGB_LEDs();
	Control_RGB_LEDs(1,1,0);	// Yellow - starting up

	Scheduler();  
}
