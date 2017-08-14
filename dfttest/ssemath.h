
// This code was taken from:
//
// Approximate Math (AM) Library
// Release 2.0, October 2000
// Copyright (c) Intel Corporation 1998 - 2000

#include <xmmintrin.h>

#define	ASM_MOVE_H2L(src)    \
			__asm	shufps	src, src, _MM_SHUFFLE(3,2,3,2)

#define	ASM_MOVE_L2H(src)    \
			__asm	shufps	src, src, _MM_SHUFFLE(1,0,1,0)

#define _PS_CONST(Name, Val) \
static const _MM_ALIGN16 float _ps_##Name[4] = { Val, Val, Val, Val }

#define _PS_EXTERN_CONST(Name, Val) \
const _MM_ALIGN16 float _ps_##Name[4] = { Val, Val, Val, Val }

#define _PS_EXTERN_CONST_TYPE(Name, Type, Val) \
const _MM_ALIGN16 Type _ps_##Name[4] = { Val, Val, Val, Val }; \

#define _EPI32_CONST(Name, Val) \
static const _MM_ALIGN16 __int32 _epi32_##Name[4] = { Val, Val, Val, Val }

__m128 __stdcall pow_sse(__m128 x, __m128 y);
__m128 __stdcall pow_sse2(__m128 x, __m128 y);