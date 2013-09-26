/* Barc_Flash.h - flash driver header file */ 

 
/* Copyright Mistral Software Pvt. Ltd.*/ 

/* 
modification history 
-------------------- 
01a,8oct03,ak written. 
*/ 
 
#ifndef __INCBarc_Flashh 
#define __INCBarc_Flashh 

#include <linux/ioctl.h>

#ifndef __INCBarc_Typesh
#include <barcelona/Barc_Types.h>
#endif /* __INCBarc_Typesh */

#ifdef __cplusplus 
extern "C" { 
#endif /* __cplusplus */ 

/*************************defines ******************************************/

/* Device specific defines */	
					
#define FLASH_DEVNAME		"BarcFlash" 	/* Device name */
#define FLASH_DRIVER_MAGIC	'F' 			/* Flash driver magic number */

/*Data size in the record, which can be changed according to need*/

#define EEPROM_RECORD_SIZE 	0x010 			/* Data size in the record in 16 bit words */
#define VALID_MAX_NO_RECORD	0xFFFFFFFF		/* Maximum no of record in the sector */
#define BOOTLOADER_VERSION_SIZE 16			/* Bootloader version string buffer size (bytes!!) */

/* Ioctl calls */

#define FLASH_READ_RECORD		    _IOWR ('F', 0, USER_DATA)	
#define FLASH_UPDATE_RECORD		    _IOWR ('F', 1, USER_DATA)
#define FLASH_INIT_SECTOR  		    _IO   ('F', 2)
#define FLASH_FORMAT_SECTOR		    _IOR  ('F', 3, UINT32)
#define FLASH_READ_FACTORYDATA	    _IOWR ('F', 4, FACTORY_DATA)
#define FLASH_SECTOR_STATUS		    _IOWR ('F', 5, UINT8 )
#define FLASH_BOOTLOADER_VERSION	_IOWR ('F', 6, UINT32)

/* Sector status definitions */

#define ERASED                          0xFF                    /* Sector not initialized yet */
#define RECEIVE_DATA            0xFE                    /* sector marked for swap */
#define VALID__SECTOR           0xFC                    /* sector contains valid data */
#define TRANSFER_COMPLETE       0xF8                    /* Transferred data to other
                                                                                        sector */

/* Record status definitions */

#define UNINITIALIZED   0xFF                            /* No data in the record */
#define VALID_DATA              0xFE                            /* Record has valid data */
#define UPDATE_DATA             0xFC                            /* Data update in progress */
#define SUPERSEDED              0xF8                            /* Invalid data */

/*******************************************************************************
*  FLASH STATUS:
*******************************************************************************/ 
typedef enum {
BARC_FLASH_STATUS_ERROR = -1,                         /* Failure */
BARC_FLASH_STATUS_OK,                            /* Success */
BARC_FLASH_STATUS_RET_VAL,                       /* Default value */	
BARC_FLASH_STATUS_SECTOR_FULL,       				
BARC_FLASH_STATUS_FORMAT_FAILED,			    
BARC_FLASH_STATUS_ILLEGAL_RECORD_NUMBER,	    
BARC_FLASH_INVALID_COMMMAND,
BARC_FLASH_STATUS_INVALID_SECTOR_STATUS,			
BARC_FLASH_STATUS_INVALID_SECTOR_ID,
BARC_FLASH_STATUS_SECTOR_ERASE_FAILED,		        
BARC_FLASH_STATUS_FLASH_WRITE_FAILED,
BARC_FLASH_STATUS_FLASH_READ_FAILED,  
BARC_FLASH_STATUS_INVALID_ADDRESS,			        
BARC_FLASH_STATUS_INVALID_SECTOR_STATE,	    
BARC_FLASH_STATUS_FLASH_ACCESS_FAILED		        
} BARC_FLASH_STATUS;

/*****************************typedefs***************************************/


typedef struct userData
	{
	UINT32	u32LocationId;					/* Identifies the record */
	UINT16	*pu16BufferAddr; 				/* Buffer to write/read data */
	} USER_DATA;

typedef struct recordInfo
	{
	UINT32 u32MaxRecords;					/* Maximum no of records */
	UINT32 u32RecLength;					/* Size of the user data */
	} RECORD_INFO;

typedef struct factoryData 
	{
	UINT32	 u32DataSize;    				/* Number of bytes to be read */	
	UINT16  	*pu16DataBuffer; 				/* Buffer to store/read data */
	} FACTORY_DATA;


#ifdef __cplusplus
} 
#endif /* __cplusplus */ 
 
#endif /* __INCBarc_Flashh */	
