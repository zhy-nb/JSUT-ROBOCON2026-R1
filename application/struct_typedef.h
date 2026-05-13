#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

/**
  *************************** JSUT-ROBOCON2026-R1 ***************************
  * @file       struct_typedef.h
  * @brief      Project-wide fixed-width type aliases.
  * @brief      工程通用定宽类型别名。
  * @note       Kept small on purpose so low-level and application modules can
  *             share integer, boolean and floating-point aliases consistently.
  * @note       保持轻量，用于让底层和应用层统一使用整数、布尔和浮点类型别名。
  *************************** JSUT-ROBOCON2026-R1 ***************************
  */

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;


#endif


