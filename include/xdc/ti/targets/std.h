/*
 *  Copyright (c) 2008 Texas Instruments and others.
 *  All rights reserved. This program and the accompanying materials
 *  are made available under the terms of the Eclipse Public License v1.0
 *  which accompanies this distribution, and is available at
 *  http://www.eclipse.org/legal/epl-v10.html
 *
 *  Contributors:
 *      Texas Instruments - initial implementation
 *
 * */
/*
 *  ======== gnu/targets/std.h ========
 *
 */

#ifndef gnu_targets_STD_
#define gnu_targets_STD_

/* include target-specific "portable" macros */
#if defined(xdc_target_name__) & !defined(xdc_target_macros_include__)
#include xdc__local_include(xdc_target_name__)
#endif

/*
 * xdc__LONGLONG__ indicates if compiler supports 'long long' type
 * xdc__BITS<n> __ indicates if compiler supports 'uint<n>_t' type
 */
#define xdc__LONGLONG__
#define xdc__BITS8__
#define xdc__BITS16__
#define xdc__BITS32__
#define xdc__BITS64__
#define xdc__INT64__

/*
 *  ======== [U]Int<n> ========
 */
typedef signed char         xdc_Int8;
typedef unsigned char       xdc_UInt8;
typedef short               xdc_Int16;
typedef unsigned short      xdc_UInt16;
typedef long                xdc_Int32;
typedef unsigned long       xdc_UInt32;

__extension__ typedef long long           xdc_Int64;
__extension__ typedef unsigned long long  xdc_UInt64;

/*
 *  ======== Bits<n> ========
 */
typedef unsigned char       xdc_Bits8;
typedef unsigned short      xdc_Bits16;
typedef unsigned long       xdc_Bits32;
__extension__ typedef unsigned long long  xdc_Bits64;

/*
 *  ======== [IU]Arg ========
 */
typedef long            xdc_IArg;
typedef unsigned long   xdc_UArg;

#define xdc__ARG__
typedef xdc_IArg xdc_Arg;       /* deprecated, but compatible with BIOS 5.x */

/*
 *  ======== xdc__META ========
 */
#define xdc__META(n,s) __attribute__ ((section ("xdc.meta"))) const char (n)[] = {s}

#endif /* gnu_targets_STD_ */

/*
 *  @(#) gnu.targets; 1, 0, 1,310; 6-10-2009 11:24:15; /db/atree/library/trees/xdctargets/xdctargets-b08x/src/
 */

