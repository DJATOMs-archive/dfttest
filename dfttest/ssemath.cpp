
// This code was taken from:
//
// Approximate Math (AM) Library
// Release 2.0, October 2000
// Copyright (c) Intel Corporation 1998 - 2000

#include "ssemath.h"

_PS_CONST(exp2_hi, 127.4999961853f);
_PS_CONST(exp2_lo, -127.4999961853f);

_PS_CONST(exp2_p0, 2.30933477057345225087e-2f);
_PS_CONST(exp2_p1, 2.02020656693165307700e1f);
_PS_CONST(exp2_p2, 1.51390680115615096133e3f);

_PS_CONST(exp2_q0, 2.33184211722314911771e2f);
_PS_CONST(exp2_q1, 4.36821166879210612817e3f);

_PS_CONST(log_p0, -7.89580278884799154124e-1f);
_PS_CONST(log_p1, 1.63866645699558079767e1f);
_PS_CONST(log_p2, -6.41409952958715622951e1f);

_PS_CONST(log_q0, -3.56722798256324312549e1f);
_PS_CONST(log_q1, 3.12093766372244180303e2f);
_PS_CONST(log_q2, -7.69691943550460008604e2f);

_PS_CONST(log_c0, 0.693147180559945f);
_PS_CONST(log2_c0, 1.44269504088896340735992f);

_PS_EXTERN_CONST(am_1, 1.0f);
_PS_EXTERN_CONST(am_0p5, 0.5f);
_PS_EXTERN_CONST_TYPE(am_min_norm_pos, __int32, 0x00800000);
_PS_EXTERN_CONST_TYPE(am_inv_mant_mask, __int32, ~0x7f800000);

_EPI32_CONST(1, 1);
_EPI32_CONST(0x7f, 0x7f);
#define _pi32_0x7f	_epi32_0x7f

__m128 __declspec(naked) __stdcall pow_sse(__m128 x, __m128 y)
{
	__asm
	{
		xorps	xmm4, xmm4
		cmpltps	xmm4, xmm0
		maxps	xmm0, _ps_am_min_norm_pos  // cut off denormalized stuff
		mov		ecx, esp
		movaps	xmm7, _ps_am_inv_mant_mask
		and		ecx, ~15
		movaps	xmm3, _ps_am_1
		movaps	[ecx - 16], xmm0

		andps	xmm0, xmm7
		orps	xmm0, xmm3
		movaps	xmm7, xmm0

		subps	xmm0, xmm3
		addps	xmm7, xmm3
		movq	mm0, [ecx - 16]
		rcpps	xmm7, xmm7  
		mulps	xmm0, xmm7
		movq	mm1, [ecx - 16 + 8]
		addps	xmm0, xmm0

		movq	mm7, _pi32_0x7f
		psrld	mm0, 23
		psrld	mm1, 23
		movaps	[ecx - 32], xmm4

		movaps	xmm2, xmm0
		psubd	mm0, mm7
		mulps	xmm2, xmm2
		psubd	mm1, mm7

		movaps	xmm4, _ps_log_p0
		movaps	xmm6, _ps_log_q0

		mulps	xmm4, xmm2
		movaps	xmm5, _ps_log_p1
		mulps	xmm6, xmm2
		movaps	xmm7, _ps_log_q1

		addps	xmm4, xmm5
		addps	xmm6, xmm7

		movaps	xmm5, _ps_log_p2
		mulps	xmm4, xmm2
		cvtpi2ps	xmm3, mm1
		movaps	xmm7, _ps_log_q2
		mulps	xmm6, xmm2

		ASM_MOVE_L2H(xmm3)
		addps	xmm4, xmm5
		addps	xmm6, xmm7

		movaps	xmm5, _ps_log2_c0
		mulps	xmm4, xmm2
		cvtpi2ps	xmm3, mm0
		rcpps	xmm6, xmm6  

		mulps	xmm5, xmm1
		mulps	xmm4, xmm6
		movaps	xmm6, _ps_exp2_hi
		mulps	xmm4, xmm0
		addps	xmm0, xmm4
		movaps	xmm4, _ps_exp2_lo
		mulps	xmm3, xmm1
		mulps	xmm0, xmm5
		movaps	xmm5, _ps_am_1
		xorps	xmm7, xmm7

		addps	xmm0, xmm3
		movaps	xmm3, _ps_am_0p5

		minps	xmm0, xmm6
		maxps	xmm0, xmm4

		addps	xmm3, xmm0

		movaps	xmm2, xmm3

		cvttps2pi	mm0, xmm3
		cmpltps	xmm2, xmm7
		ASM_MOVE_H2L(xmm3)
		andps	xmm2, xmm5
		cvttps2pi	mm1, xmm3
		movq	mm5, _pi32_0x7f

		cvtps2pi	mm2, xmm2  // needn't truncate
		ASM_MOVE_H2L(xmm2)
		cvtps2pi	mm3, xmm2  // needn't truncate

		psubd	mm0, mm2
		psubd	mm1, mm3

		cvtpi2ps	xmm3, mm1
		ASM_MOVE_L2H(xmm3)
		paddd	mm1, mm5
		cvtpi2ps	xmm3, mm0
		paddd	mm0, mm5

		subps	xmm0, xmm3

		movaps	xmm2, xmm0
		mulps	xmm2, xmm2

		movaps	xmm6, _ps_exp2_q0
		movaps	xmm4, _ps_exp2_p0

		mulps	xmm6, xmm2
		movaps	xmm7, _ps_exp2_q1
		mulps	xmm4, xmm2
		movaps	xmm5, _ps_exp2_p1

		addps	xmm6, xmm7
		pslld	mm0, 23
		addps	xmm4, xmm5

		movaps	xmm5, _ps_exp2_p2
		mulps	xmm4, xmm2
		pslld	mm1, 23
		movaps	xmm3, [ecx - 32]

		addps	xmm4, xmm5
		movq	[ecx - 16], mm0

		mulps	xmm4, xmm0
		movq	[ecx - 16 + 8], mm1

		subps	xmm6, xmm4
		movaps	xmm7, _ps_am_1
		rcpps	xmm6, xmm6  
		mulps	xmm4, xmm6
		movaps	xmm0, [ecx - 16]
		addps	xmm4, xmm4
		addps	xmm4, xmm7

		mulps	xmm0, xmm4
		andps	xmm0, xmm3

		ret
	}
}

__m128 __declspec(naked) __stdcall pow_sse2(__m128 x, __m128 y)
{
	__asm
	{
		xorps	xmm5, xmm5
		cmpltps	xmm5, xmm0
		mov		ecx, esp
		maxps	xmm0, _ps_am_min_norm_pos  // cut off denormalized stuff
		movaps	xmm7, _ps_am_1
		movaps	xmm3, xmm0
		and		ecx, ~15

		andps	xmm0, _ps_am_inv_mant_mask
		orps	xmm0, xmm7

		movaps	[ecx - 16], xmm5

		movaps	xmm4, xmm0
		subps	xmm0, xmm7
		addps	xmm4, xmm7
		psrld	xmm3, 23
		rcpps	xmm4, xmm4
		mulps	xmm0, xmm4
		psubd	xmm3, _epi32_0x7f
		addps	xmm0, xmm0

		movaps	xmm2, xmm0
		mulps	xmm0, xmm0

		movaps	xmm4, _ps_log_p0
		movaps	xmm6, _ps_log_q0

		mulps	xmm4, xmm0
		movaps	xmm5, _ps_log_p1
		mulps	xmm6, xmm0
		movaps	xmm7, _ps_log_q1

		addps	xmm4, xmm5
		addps	xmm6, xmm7

		movaps	xmm5, _ps_log_p2
		mulps	xmm4, xmm0
		movaps	xmm7, _ps_log_q2
		mulps	xmm6, xmm0

		addps	xmm4, xmm5
		movaps	xmm5, _ps_log2_c0
		addps	xmm6, xmm7
		cvtdq2ps	xmm7, xmm3

		mulps	xmm0, xmm4
		rcpps	xmm6, xmm6

		mulps	xmm0, xmm6
		movaps	xmm4, _ps_exp2_hi
		mulps	xmm0, xmm2
		movaps	xmm6, _ps_exp2_lo
		mulps	xmm2, xmm5
		mulps	xmm0, xmm5
		addps	xmm2, xmm7
		movaps	xmm3, _ps_am_0p5
		addps	xmm0, xmm2
		xorps	xmm2, xmm2

		mulps	xmm0, xmm1

		minps	xmm0, xmm4
		movaps	xmm4, _ps_exp2_p0
		maxps	xmm0, xmm6
		movaps	xmm6, _ps_exp2_q0

		addps	xmm3, xmm0

		cmpnltps	xmm2, xmm3
		pand	xmm2, _epi32_1

		cvttps2dq	xmm3, xmm3

		psubd	xmm3, xmm2
		movaps	xmm5, _ps_exp2_p1

		cvtdq2ps	xmm2, xmm3
		movaps	xmm7, _ps_exp2_q1

		subps	xmm0, xmm2

		movaps	xmm2, xmm0
		mulps	xmm0, xmm0

		paddd	xmm3, _epi32_0x7f

		mulps	xmm4, xmm0
		mulps	xmm6, xmm0
		addps	xmm4, xmm5
		addps	xmm6, xmm7

		mulps	xmm4, xmm0
		movaps	xmm5, [ecx - 16]
		pslld	xmm3, 23
		addps	xmm4, _ps_exp2_p2

		mulps	xmm2, xmm4

		movaps	xmm0, _ps_am_1
		subps	xmm6, xmm2
		andps	xmm3, xmm5
		rcpps	xmm6, xmm6
		mulps	xmm2, xmm6
		addps	xmm2, xmm2
		addps	xmm0, xmm2

		mulps	xmm0, xmm3

		ret
	}
}