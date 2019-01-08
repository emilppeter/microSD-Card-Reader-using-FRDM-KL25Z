/*
 *  File: sd_io.c
 *  Author: Nelson Lombardo
 *  Year: 2015
 *  e-mail: nelson.lombardo@gmail.com
 *  License at the end of file.
 */
 
// Modified 2017 by Alex Dean (agdean@ncsu.edu) for teaching FSMs
// - Removed support for PC development (_M_IX86)
// - Split single-line loops & conditionals in source code for readability
// - Fused loops in SD_Read 
// _ Inlined __SD_Write_Block into SD_Write

#include "sd_io.h"
#include <MKL25Z4.h>
#include "debug.h"
#include "sd_server.h"

/* Results of SD functions */
char SD_Errors[7][8] = {
    "OK",      
    "NOINIT",      /* 1: SD not initialized    */
    "ERROR",       /* 2: Disk error            */
    "PARERR",      /* 3: Invalid parameter     */
    "BUSY",        /* 4: Programming busy      */
    "REJECT",      /* 5: Reject data           */
    "NORESP"       /* 6: No response           */
};

/******************************************************************************
 Private Methods Prototypes - Direct work with SD card
******************************************************************************/

/**
    \brief Simple function to calculate power of two.
    \param e Exponent.
    \return Math function result.
*/
DWORD __SD_Power_Of_Two(BYTE e);
FSM Read;
FSM Write;
FSM Init;
/**
     \brief Assert the SD card (SPI CS low).
 */
inline void __SD_Assert (void);

/**
    \brief Deassert the SD (SPI CS high).
 */
inline void __SD_Deassert (void);

/**
    \brief Change to max the speed transfer.
    \param throttle
 */
void __SD_Speed_Transfer (BYTE throttle);

/**
    \brief Send SPI commands.
    \param cmd Command to send.
    \param arg Argument to send.
    \return R1 response.
 */
BYTE __SD_Send_Cmd(BYTE cmd, DWORD arg);

/**
    \brief Get the total numbers of sectors in SD card.
    \param dev Device descriptor.
    \return Quantity of sectors. Zero if fail.
 */
DWORD __SD_Sectors (SD_DEV *dev);

/******************************************************************************
 Private Methods - Direct work with SD card
******************************************************************************/

DWORD __SD_Power_Of_Two(BYTE e)
{
    DWORD partial = 1;
    BYTE idx;
    for(idx=0; idx!=e; idx++) partial *= 2;
    return(partial);
}

inline void __SD_Assert(void){
    SPI_CS_Low();
}

inline void __SD_Deassert(void){
    SPI_CS_High();
}

void __SD_Speed_Transfer(BYTE throttle) {
    if(throttle == HIGH) SPI_Freq_High();
    else SPI_Freq_Low();
}

BYTE __SD_Send_Cmd(BYTE cmd, DWORD arg)
{
    BYTE crc, res;
	// ACMD«n» is the command sequense of CMD55-CMD«n»
    if(cmd & 0x80) {
        cmd &= 0x7F;
        res = __SD_Send_Cmd(CMD55, 0);
        if (res > 1) 
					return (res);
    }

    // Select the card
    __SD_Deassert();
    SPI_RW(0xFF);
    __SD_Assert();
    SPI_RW(0xFF);

    // Send complete command set
    SPI_RW(cmd);                        // Start and command index
    SPI_RW((BYTE)(arg >> 24));          // Arg[31-24]
    SPI_RW((BYTE)(arg >> 16));          // Arg[23-16]
    SPI_RW((BYTE)(arg >> 8 ));          // Arg[15-08]
    SPI_RW((BYTE)(arg >> 0 ));          // Arg[07-00]

    // CRC?
    crc = 0x01;                         // Dummy CRC and stop
    if(cmd == CMD0) 
			crc = 0x95;         // Valid CRC for CMD0(0)
    if(cmd == CMD8) 
			crc = 0x87;         // Valid CRC for CMD8(0x1AA)
    SPI_RW(crc);

    // Receive command response
    // Wait for a valid response in timeout of 5 milliseconds
    SPI_Timer_On(5);
    do {
        res = SPI_RW(0xFF);
    } while((res & 0x80)&&(SPI_Timer_Status()==TRUE));
    SPI_Timer_Off();
		
    
		// Return with the response value
    return(res);
}

DWORD __SD_Sectors (SD_DEV *dev)
{
    BYTE csd[16];
    BYTE idx;
    DWORD ss = 0;
    WORD C_SIZE = 0;
    BYTE C_SIZE_MULT = 0;
    BYTE READ_BL_LEN = 0;
    if(__SD_Send_Cmd(CMD9, 0)==0) 
    {
        // Wait for response
        while (SPI_RW(0xFF) == 0xFF);
        for (idx=0; idx!=16; idx++) 
					csd[idx] = SPI_RW(0xFF);
        // Dummy CRC
        SPI_RW(0xFF);
        SPI_RW(0xFF);
        SPI_Release();
        if(dev->cardtype & SDCT_SD1)
        {
            ss = csd[0];
            // READ_BL_LEN[83:80]: max. read data block length
            READ_BL_LEN = (csd[5] & 0x0F);
            // C_SIZE [73:62]
            C_SIZE = (csd[6] & 0x03);
            C_SIZE <<= 8;
            C_SIZE |= (csd[7]);
            C_SIZE <<= 2;
            C_SIZE |= ((csd[8] >> 6) & 0x03);
            // C_SIZE_MULT [49:47]
            C_SIZE_MULT = (csd[9] & 0x03);
            C_SIZE_MULT <<= 1;
            C_SIZE_MULT |= ((csd[10] >> 7) & 0x01);
        }
        else if(dev->cardtype & SDCT_SD2)
        {
						// READ_BL_LEN = 9;
            // C_SIZE [69:48]
            C_SIZE = (csd[7] & 0x3F);
            C_SIZE <<= 8;
            C_SIZE |= (csd[8] & 0xFF);
            C_SIZE <<= 8;
            C_SIZE |= (csd[9] & 0xFF);
            C_SIZE_MULT = 8; // AD changed
        }
        ss = (C_SIZE + 1);
        ss *= __SD_Power_Of_Two(C_SIZE_MULT + 2);
        ss *= __SD_Power_Of_Two(READ_BL_LEN);
        // ss /= SD_BLK_SIZE; ?? Bug in original code?

        return (ss);
    } else return (0); // Error
}

/******************************************************************************
 Public Methods - Direct work with SD card
******************************************************************************/

SDRESULTS SD_Init(SD_DEV *dev)
{
    static BYTE n, cmd, ct, ocr[4];
    static BYTE idx;
    static BYTE init_trys;
		static enum {S1,S2,S3,S4,S5,S6,S7,S8,S9,S10,S11,S12,S13} next_state = S1;
		if (Init.set_fsm==1)
		{
			ct=0;
			init_trys=0;
			Init.set_fsm++;
		}
		//PTB->PSOR = MASK(DBG_4);
		switch(next_state)
		{
			case S1:	PTB->PTOR = MASK(DBG_4);
								if((init_trys!=SD_INIT_TRYS)&&(!ct))
								{	
									SPI_Init();// Initialize SPI for use with the memory card
									SPI_CS_High();
									SPI_Freq_Low();
									next_state=S2;
									init_trys++;
									Init.Status_fsm=STAT_BUSY;
									Init.Start_fsm=0;
									// 80 dummy clocks
									for(idx = 0; idx != 10; idx++) 
										SPI_RW(0xFF);
									SPI_Timer_On(500);
								}
								else
								{
									Init.Status_fsm=STAT_BUSY;
									Init.Start_fsm=0;
									next_state=S11;
								}
							PTB->PTOR = MASK(DBG_4);
							break;
			
			case S2:  
							PTB->PTOR = MASK(DBG_4);
							if(SPI_Timer_Status()==TRUE)
							{
								next_state=S2;
								//PTB->PTOR = MASK(DBG_4);
								//PTB->PTOR = MASK(DBG_4);
							}
							else
							{
								SPI_Timer_Off();
								dev->mount = FALSE;
								next_state=S3;
								SPI_Timer_On(500);
							}
							PTB->PTOR = MASK(DBG_4);
							break;
			case S3:				
							PTB->PTOR = MASK(DBG_4);
							if ((__SD_Send_Cmd(CMD0, 0) != 1)&&(SPI_Timer_Status()==TRUE))
							{
								next_state=S3;
								//PTB->PTOR = MASK(DBG_4);
								//PTB->PTOR = MASK(DBG_4);
							}
							else
							{
								next_state=S4;
								SPI_Timer_Off();
							}
							break;
							PTB->PTOR = MASK(DBG_4);
			case S4:
							PTB->PTOR = MASK(DBG_4);
				      // Idle state
							if (__SD_Send_Cmd(CMD0, 0) == 1) 
							{
								next_state=S5;
							}
							else
							{
								next_state=S1;
							}
							PTB->PTOR = MASK(DBG_4);
							break;
			case S5:
							PTB->PTOR = MASK(DBG_4);
							// SD version 2?
							if (__SD_Send_Cmd(CMD8, 0x1AA) == 1) 
							{
								next_state=S7;
							}
							else
							{
								next_state=S6;
							}
							PTB->PTOR = MASK(DBG_4);
							break;
			case S6:
							PTB->PTOR = MASK(DBG_4);
							// SD version 1 or MMC?
              if (__SD_Send_Cmd(ACMD41, 0) <= 1)
              {
                // SD version 1
                ct = SDCT_SD1; 
                cmd = ACMD41;
              }
							else 
							{
								// MMC version 3
                ct = SDCT_MMC; 
                cmd = CMD1;
              }
                // Wait for leaving idle state
              SPI_Timer_On(250);
              while((SPI_Timer_Status()==TRUE)&&(__SD_Send_Cmd(cmd, 0))) 
							{
									//PTB->PTOR = MASK(DBG_4);
									//PTB->PTOR = MASK(DBG_4);
							}
              SPI_Timer_Off();
              if(SPI_Timer_Status()==FALSE) 
							ct = 0;
              if(__SD_Send_Cmd(CMD59, 0))   
							ct = 0;   // Deactivate CRC check (default)
              if(__SD_Send_Cmd(CMD16, 512)) 
							ct = 0;   // Set R/W block length to 512 bytes
							next_state=S1;
							PTB->PTOR = MASK(DBG_4);
							break;
			case S7:
							PTB->PTOR = MASK(DBG_4);
							// Get trailing return value of R7 resp
							for (n = 0; n < 4; n++) 
								ocr[n] = SPI_RW(0xFF);
								// VDD range of 2.7-3.6V is OK?  
              if ((ocr[2] == 0x01)&&(ocr[3] == 0xAA))
              {
								// Wait for leaving idle state (ACMD41 with HCS bit)...
                SPI_Timer_On(1000);
								next_state=S8;
							}
							else
							{
								next_state=S1;
							}
							PTB->PTOR = MASK(DBG_4);
							break;
			case S8:
							PTB->PTOR = MASK(DBG_4);
							__SD_Speed_Transfer(HIGH);
							if ((SPI_Timer_Status()==TRUE)&&(__SD_Send_Cmd(ACMD41, 1UL << 30)))
							{
											next_state=S8;
											//PTB->PTOR = MASK(DBG_4);
											//PTB->PTOR = MASK(DBG_4);
							}
							else 
							{
								next_state=S9;
							}
							PTB->PTOR = MASK(DBG_4);
							break;
			case S9:
							PTB->PTOR = MASK(DBG_4);
							SPI_Timer_Off(); 
              // CCS in the OCR? 
							// AGD: Delete SPI_Timer_Status call?
							if ((SPI_Timer_Status()==TRUE)&&(__SD_Send_Cmd(CMD58, 0) == 0))
              {
								next_state=S10;
							}
							else
							{
								next_state=S1;								
							}
							PTB->PTOR = MASK(DBG_4);
							break;
			case S10:
							PTB->PTOR = MASK(DBG_4);
							for (n = 0; n < 4; n++) 
								ocr[n] = SPI_RW(0xFF);
              // SD version 2?
              ct = (ocr[0] & 0x40) ? SDCT_SD2 | SDCT_BLOCK : SDCT_SD2;
							next_state=S1;
							PTB->PTOR = MASK(DBG_4);
							break;
      case S11:
							PTB->PTOR = MASK(DBG_4);
							if(ct) 
							{
								dev->cardtype = ct;
								dev->mount = TRUE;
								dev->last_sector = __SD_Sectors(dev) - 1;
								dev->debug.read = 0;
								dev->debug.write = 0;
								//__SD_Speed_Transfer(HIGH);
								// High speed transfer
							}
							next_state=S12;
							PTB->PTOR = MASK(DBG_4);
							break;
			case S12:
							PTB->PTOR = MASK(DBG_4);
							SPI_Release();
							Init.Status_fsm=STAT_IDLE;
							Init.ErrorCode_fsm=ct ? SD_OK : SD_NOINIT;
							Init.Start_fsm=1;
							Init.set_fsm=0;
							next_state=S1;
							PTB->PTOR = MASK(DBG_4);
							break;
			default:
							Init.Status_fsm=STAT_IDLE;
							next_state=S1;
							break;
		}
		PTB->PCOR = MASK(DBG_4);
	}

#pragma push
#pragma diag_suppress 1441
void SD_Read_FSM(SD_DEV *dev, void *dat, DWORD sector, WORD ofs, WORD cnt)
{
    static SDRESULTS res;
    static BYTE tkn, data;
    static WORD byte_num;
		static void *temp;
		static enum {S1,S2,S3,S4,S5} next_state = S1;
		//PTB->PSOR = MASK(DBG_2);
    switch(next_state)
		{	
			case S1:PTB->PTOR = MASK(DBG_2);
							if (Read.Status_fsm==STAT_IDLE)
							{
							res = SD_ERROR;
							temp=dat;
							if ((sector > dev->last_sector)||(cnt == 0)) 
							{	
								next_state= S1;
								Read.Start_fsm=1;
								Read.ErrorCode_fsm=SD_PARERR;
								break;
							}		
							// Convert sector number to byte address (sector * SD_BLK_SIZE)
							// if (__SD_Send_Cmd(CMD17, sector * SD_BLK_SIZE) == 0) { // Only for SDSC
							if (__SD_Send_Cmd(CMD17, sector ) == 0)		// Only for SDHC or SDXC   
							{
								SPI_Timer_On(100);// Wait for data packet (timeout of 100ms)
								Read.Status_fsm=STAT_BUSY;
								Read.Start_fsm=0;
								next_state=S2;
								PTB->PTOR = MASK(DBG_2);
								break;
							}
							else
							{
								Read.Start_fsm=0;
								Read.Status_fsm=STAT_BUSY;
								next_state=S5;
								PTB->PTOR = MASK(DBG_2);
								break;
							}
							}
			case S2:
							PTB->PTOR = MASK(DBG_2);
							tkn = SPI_RW(0xFF);
							//PTB->PTOR = MASK(DBG_2);
							//PTB->PTOR = MASK(DBG_2);		
							if ((tkn==0xFF)&&(SPI_Timer_Status()==TRUE))
							{
								next_state=S2;
								PTB->PTOR = MASK(DBG_2);
								break;
							}
							else
							{
								next_state=S3;
								PTB->PTOR = MASK(DBG_2);
								break;
							}
			case S3:  
							PTB->PTOR = MASK(DBG_2);
							SPI_Timer_Off();
							// Token of single block?
							if(tkn==0xFE) { // AGD: Loop fusion to simplify FSM formation
								byte_num = 0;
								next_state = S4;
								PTB->PTOR = MASK(DBG_2);
								break;
							}
							else
							{
								next_state= S5;
								PTB->PTOR = MASK(DBG_2);
								break;
							}
			case S4:
							/*do
							{*/
							PTB->PTOR = MASK(DBG_2);
							data = SPI_RW(0xff);
							if ((byte_num >= ofs) && (byte_num < ofs+cnt))
							{
               *(BYTE*)temp = data;
               ((BYTE *) temp)++;
							}
							//}else discard bytes before and after data
						  if (++byte_num < SD_BLK_SIZE + 2 )//; // 512 byte block + 2 byte CRC
							{
								next_state= S4;
								PTB->PTOR = MASK(DBG_2);
								break;
							}
							else
							{
								res = SD_OK;
								next_state = S5;
								PTB->PTOR = MASK(DBG_2);
								break;
							}
			
			case S5:
							PTB->PTOR = MASK(DBG_2);
							SPI_Release();
							dev->debug.read++;
							next_state=S1;
							Read.Status_fsm=STAT_IDLE;
							Read.ErrorCode_fsm=res;
							Read.Start_fsm=1;
							PTB->PTOR = MASK(DBG_2);
							break;
			default:next_state=S1;
							Read.Status_fsm=STAT_IDLE;
							break;
				}
		PTB->PCOR = MASK(DBG_2);
			}
#pragma pop

void SD_Write_FSM(SD_DEV *dev, void *dat, DWORD sector)
{
    static WORD idx;
    static BYTE line;
		static enum {S1,S2,S3,S4,S5} next_state = S1;
		//PTB->PSOR = MASK(DBG_3);

		// Query invalid?
		switch(next_state)
		{
			case S1:
							PTB->PTOR = MASK(DBG_3);
							if (Write.Status_fsm==STAT_IDLE)
							{
								if(sector > dev->last_sector)
								{
									next_state=S1;
									Write.Status_fsm=STAT_IDLE;
									Write.Start_fsm=1;
									Write.ErrorCode_fsm=SD_PARERR;
									break;
								}
								// Convert sector number to bytes address (sector * SD_BLK_SIZE)
								//    if(__SD_Send_Cmd(CMD24, sector * SD_BLK_SIZE)==0) { // Only for SDSC
								if(__SD_Send_Cmd(CMD24, sector)==0) 
								{ // Only for SDHC or SDXC   
									// Send token (single block write)
									SPI_RW(0xFE);// Send block data
									next_state=S2;
									idx=0;
									Write.Status_fsm=STAT_BUSY;
								}
								else
								{
									next_state=S1;
									Write.Start_fsm=1;
									Write.Status_fsm=STAT_IDLE;
									Write.ErrorCode_fsm=SD_ERROR;
								}
								PTB->PTOR = MASK(DBG_3);
								break;
							}
			case S2:
							PTB->PTOR = MASK(DBG_3);
							SPI_RW(*((BYTE*)dat + idx));
							idx++;
							if (idx!=SD_BLK_SIZE)
							{
								next_state=S2;
							}	
							else
							{
								next_state=S3;
							}
							PTB->PTOR = MASK(DBG_3);
							break;
							
			case S3:	
							PTB->PTOR = MASK(DBG_3);
							/* Dummy CRC */
							SPI_RW(0xFF);
							SPI_RW(0xFF);
							// If not accepted, returns the reject error
							if((SPI_RW(0xFF) & 0x1F) != 0x05) 
							{
								next_state=S1;
								Write.Start_fsm=1;
								Write.ErrorCode_fsm=SD_REJECT;
								Write.Status_fsm=STAT_IDLE;
								PTB->PTOR = MASK(DBG_3);
								break;
							}		
							// Waits until finish of data programming with a timeout
							SPI_Timer_On(SD_IO_WRITE_TIMEOUT_WAIT);
							next_state=S4;
							PTB->PTOR = MASK(DBG_3);
							break;
			
			case S4:
							PTB->PTOR = MASK(DBG_3);
							line = SPI_RW(0xFF);
							//PTB->PTOR = MASK(DBG_3);
							//PTB->PTOR = MASK(DBG_3);
							if ((line==0)&&(SPI_Timer_Status()==TRUE))
							{
								next_state=S4;
							}
							else
							{
								next_state=S5;
							}
							PTB->PTOR = MASK(DBG_3);
							break;
							//PTB->PTOR = MASK(DBG_3);
			case S5:
							SPI_Timer_Off();
							dev->debug.write++;
							//PTB->PTOR = MASK(DBG_3);
							//PTB->PCOR = MASK(DBG_3);
							next_state=S1;
							if(line==0)
							{
								Write.Status_fsm=STAT_IDLE;
								Write.ErrorCode_fsm=SD_BUSY;
								Write.Start_fsm=1;
							}	
							else 
							{
								Write.Status_fsm=STAT_IDLE;
								Write.ErrorCode_fsm=SD_OK;
								Write.Start_fsm=1;
							}
							PTB->PTOR = MASK(DBG_3);
							break;
			default:
							Write.Status_fsm=STAT_IDLE;
							next_state=S1;
							break;
			}
		PTB->PCOR = MASK(DBG_3);
}

SDRESULTS SD_Status(SD_DEV *dev)
{
    return(__SD_Send_Cmd(CMD0, 0) ? SD_OK : SD_NORESPONSE);
}

// «sd_io.c» is part of:
/*----------------------------------------------------------------------------/
/  ulibSD - Library for SD cards semantics            (C)Nelson Lombardo, 2015
/-----------------------------------------------------------------------------/
/ ulibSD library is a free software that opened under license policy of
/ following conditions.
/
/ Copyright (C) 2015, ChaN, all right reserved.
/
/ 1. Redistributions of source code must retain the above copyright notice,
/    this condition and the following disclaimer.
/
/ This software is provided by the copyright holder and contributors "AS IS"
/ and any warranties related to this software are DISCLAIMED.
/ The copyright owner or contributors be NOT LIABLE for any damages caused
/ by use of this software.
/----------------------------------------------------------------------------*/

// Derived from Mister Chan works on FatFs code (http://elm-chan.org/fsw/ff/00index_e.html):
/*----------------------------------------------------------------------------/
/  FatFs - FAT file system module  R0.11                 (C)ChaN, 2015
/-----------------------------------------------------------------------------/
/ FatFs module is a free software that opened under license policy of
/ following conditions.
/
/ Copyright (C) 2015, ChaN, all right reserved.
/
/ 1. Redistributions of source code must retain the above copyright notice,
/    this condition and the following disclaimer.
/
/ This software is provided by the copyright holder and contributors "AS IS"
/ and any warranties related to this software are DISCLAIMED.
/ The copyright owner or contributors be NOT LIABLE for any damages caused
/ by use of this software.
/----------------------------------------------------------------------------*/
