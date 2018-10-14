/*
**                    dfttest v1.9.4.3 for Avisynth+
**
**   2D/3D frequency domain denoiser.
**
**   Copyright (C) 2007-2010 Kevin Stone, 2017 (C) DJATOM
**
**   This program is free software; you can redistribute it and/or modify
**   it under the terms of the GNU General Public License as published by
**   the Free Software Foundation; either version 2 of the License, or
**   (at your option) any later version.
**
**   This program is distributed in the hope that it will be useful,
**   but WITHOUT ANY WARRANTY; without even the implied warranty of
**   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**   GNU General Public License for more details.
**
**   You should have received a copy of the GNU General Public License
**   along with this program; if not, write to the Free Software
**   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifdef AVX_BUILD
#include "dfttest_avx.h"

void removeMean_AVX(float *dftc, const float *dftgc, const int ccnt, float *dftc2)
{
	const float gf = dftc[0] / dftgc[0];
	auto gf_asm = _mm256_broadcast_ss(reinterpret_cast<const float*>(&gf));

	for (int h = 0; h < ccnt; h += 8)
	{
		auto dftc_loop = _mm256_loadu_ps(dftc + h);
		auto dftgc_loop = _mm256_loadu_ps(dftgc + h);
		auto dftc2_loop  = _mm256_loadu_ps(dftc2 + h);
		auto dftc2_result = _mm256_mul_ps(gf_asm, dftgc_loop);
		_mm256_storeu_ps(dftc2 + h, dftc2_result);
		auto dftc_result = _mm256_sub_ps(dftc_loop, dftc2_result);
		_mm256_storeu_ps(dftc + h, dftc_result);
	}
	_mm256_zeroupper();
}

void addMean_AVX(float *dftc, const int ccnt, const float *dftc2)
{
	for (int h = 0; h < ccnt; h += 8)
	{
		auto dftc_loop = _mm256_loadu_ps(dftc + h);
		auto dftc2_loop = _mm256_loadu_ps(dftc2 + h);
		auto dftc_result = _mm256_add_ps(dftc2_loop, dftc_loop);
		_mm256_storeu_ps(dftc + h, dftc_result);
	}
	_mm256_zeroupper();
}
#endif

