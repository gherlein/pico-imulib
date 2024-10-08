/**
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BNO_COMMON_H
#define NO_COMMON_H

/****************************************************************/
/**\name    DATA TYPES INCLUDES     */
/************************************************************/

/*!
 * @brief The following definition uses for define the data types
 *
 * @note While porting the API please consider the following
 * @note Please check the version of C standard
 * @note Are you using Linux platform
 */

/*!
 * @brief For the Linux platform support
 * Please use the types.h for your data types definitions
 */
#ifdef __KERNEL__

#include <linux/types.h>

/* singed integer type*/
typedef int8_t s8;     /**< used for signed 8bit */
typedef int16_t s16;   /**< used for signed 16bit */
typedef int32_t s32;   /**< used for signed 32bit */
typedef int64_t s64;   /**< used for signed 64bit */
typedef u_int8_t u8;   /**< used for unsigned 8bit */
typedef u_int16_t u16; /**< used for unsigned 16bit */
typedef u_int32_t u32; /**< used for unsigned 32bit */
typedef u_int64_t u64; /**< used for unsigned 64bit */

#else /* ! __KERNEL__ */

/**********************************************************
 * These definition uses for define the C
 * standard version data types
 ***********************************************************/
#if !defined(__STDC_VERSION__)

/************************************************
 * compiler is C11 C standard
 ************************************************/
#if (__STDC_VERSION__ == 201112L)

/************************************************/
#include <stdint.h>

/************************************************/
/*unsigned integer types*/
typedef uint8_t u8;   /**< used for unsigned 8bit */
typedef uint16_t u16; /**< used for unsigned 16bit */
typedef uint32_t u32; /**< used for unsigned 32bit */
typedef uint64_t u64; /**< used for unsigned 64bit */
/*signed integer types*/
typedef int8_t s8;   /**< used for signed 8bit */
typedef int16_t s16; /**< used for signed 16bit */
typedef int32_t s32; /**< used for signed 32bit */
typedef int64_t s64; /**< used for signed 64bit */

/************************************************
 * compiler is C99 C standard
 ************************************************/

#elif (__STDC_VERSION__ == 199901L)

/* stdint.h is a C99 supported c library.
 *  which is used to fixed the integer size*/

/************************************************/
#include <stdint.h>

/************************************************/
/*unsigned integer types*/
typedef uint8_t u8;   /**< used for unsigned 8bit */
typedef uint16_t u16; /**< used for unsigned 16bit */
typedef uint32_t u32; /**< used for unsigned 32bit */
typedef uint64_t u64; /**< used for unsigned 64bit */
/*signed integer types*/
typedef int8_t s8;   /**< used for signed 8bit */
typedef int16_t s16; /**< used for signed 16bit */
typedef int32_t s32; /**< used for signed 32bit */
typedef int64_t s64; /**< used for signed 64bit */

/************************************************
 * compiler is C89 or other C standard
 ************************************************/

#else /*  !defined(__STDC_VERSION__) */

/*!
 * @brief By default it is defined as 32 bit machine configuration
 *  define your data types based on your
 *  machine/compiler/controller configuration
 */
#define MACHINE_32_BIT

/*! @brief
 *  If your machine support 16 bit
 *  define the MACHINE_16_BIT
 */
#ifdef MACHINE_16_BIT
#include <limits.h>

/*signed integer types*/
typedef signed char s8;       /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed long int s32;  /**< used for signed 32bit */

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
typedef long int s64;          /**< used for signed 64bit */
typedef unsigned long int u64; /**< used for unsigned 64bit */
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
typedef long long int s64;          /**< used for signed 64bit */
typedef unsigned long long int u64; /**< used for unsigned 64bit */
#else
#warning Either the correct data type for signed 64 bit integer \
    could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
    please set s64 manually.
#endif

/*unsigned integer types*/
typedef unsigned char u8;       /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned long int u32;  /**< used for unsigned 32bit */

/* If your machine support 32 bit
 * define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT

/*signed integer types*/
typedef signed char s8;           /**< used for signed 8bit */
typedef signed short int s16;     /**< used for signed 16bit */
typedef signed int s32;           /**< used for signed 32bit */
typedef signed long long int s64; /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char u8;           /**< used for unsigned 8bit */
typedef unsigned short int u16;     /**< used for unsigned 16bit */
typedef unsigned int u32;           /**< used for unsigned 32bit */
typedef unsigned long long int u64; /**< used for unsigned 64bit */

/* If your machine support 64 bit
 * define the MACHINE_64_BIT*/
#elif defined MACHINE_64_BIT

/*signed integer types*/
typedef signed char s8;       /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed int s32;       /**< used for signed 32bit */
typedef signed long int s64;  /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char u8;       /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned int u32;       /**< used for unsigned 32bit */
typedef unsigned long int u64;  /**< used for unsigned 64bit */

#else
#warning The data types defined above which not supported \
    define the data types manually
#endif
#endif

/*** This else will execute for the compilers
 *  which are not supported the C standards
 *  Like C89/C99/C11***/
#else

/*!
 * @brief By default it is defined as 32 bit machine configuration
 *  define your data types based on your
 *  machine/compiler/controller configuration
 */
#define MACHINE_32_BIT

/* If your machine support 16 bit
 * define the MACHINE_16_BIT*/
#ifdef MACHINE_16_BIT
#include <limits.h>

/*signed integer types*/
typedef signed char s8;       /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed long int s32;  /**< used for signed 32bit */

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
typedef long int s64;          /**< used for signed 64bit */
typedef unsigned long int u64; /**< used for unsigned 64bit */
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
typedef long long int s64;          /**< used for signed 64bit */
typedef unsigned long long int u64; /**< used for unsigned 64bit */
#else
#warning Either the correct data type for signed 64 bit integer \
    could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
    please set s64 manually.
#endif

/*unsigned integer types*/
typedef unsigned char u8;       /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned long int u32;  /**< used for unsigned 32bit */

/*! @brief If your machine support 32 bit
 * define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT

/*signed integer types*/
typedef signed char s8;           /**< used for signed 8bit */
typedef signed short int s16;     /**< used for signed 16bit */
typedef signed int s32;           /**< used for signed 32bit */
typedef signed long long int s64; /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char u8;           /**< used for unsigned 8bit */
typedef unsigned short int u16;     /**< used for unsigned 16bit */
typedef unsigned int u32;           /**< used for unsigned 32bit */
typedef unsigned long long int u64; /**< used for unsigned 64bit */

/* If your machine support 64 bit
 * define the MACHINE_64_BIT*/
#elif defined MACHINE_64_BIT

/*signed integer types*/
typedef signed char s8;       /**< used for signed 8bit */
typedef signed short int s16; /**< used for signed 16bit */
typedef signed int s32;       /**< used for signed 32bit */
typedef signed long int s64;  /**< used for signed 64bit */
/*unsigned integer types*/
typedef unsigned char u8;       /**< used for unsigned 8bit */
typedef unsigned short int u16; /**< used for unsigned 16bit */
typedef unsigned int u32;       /**< used for unsigned 32bit */
typedef unsigned long int u64;  /**< used for unsigned 64bit */

#else
#warning The data types defined above which not supported \
    define the data types manually
#endif
#endif
#endif
#endif