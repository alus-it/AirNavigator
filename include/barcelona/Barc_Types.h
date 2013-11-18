/* Barc_Types.h - data types */ 

 
/* Copyright Mistral Software Pvt. Ltd.*/ 

/* 
modification history 
-------------------- 
01a,3oct03,rks��written. 
*/ 
 
#ifndef __INCBarc_Typesh
#define __INCBarc_Typesh

#ifdef __cplusplus 
extern "C" { 
#endif /* __cplusplus */ 

#define LOCAL static
#define S3C2410_DEBUG

#undef PDEBUG /* undef it, just in case */

#ifdef S3C2410_DEBUG
# ifdef __KERNEL__

# define PDEBUG(fmt, args...) printk( KERN_DEBUG fmt,	## args)

# else

/* This one for user space */

//# define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)

# endif
#else
# define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

typedef unsigned int	UINT32;
typedef signed int		INT32;

typedef unsigned short	UINT16;
typedef signed short	SINT16;

typedef unsigned char	UINT8;
typedef signed char		SINT8;

typedef unsigned long	ULONG;

#ifdef __cplusplus
} 
#endif /* __cplusplus */ 
 
#endif /* __INCBarc_Typesh */
