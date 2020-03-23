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

/*
Modifications:
2018.10.14 - DJATOM
     - Fixed one nasty bug, causing crash on non-AVX CPUs.

2017.09.04 - DJATOM
     - Adaptive MT mode: MT_MULTI_INSTANCE for threads=1 and MT_SERIALIZED for > 1 internal
   - Compilation: silence #693 for Intel Compiler

2017.08.14 - DJATOM
  Changes from 1.9.4:
   - x64 ready: ported almost all inline asm to intrinsics (dropped some SSE functions, we're in 2017 now).
   - AddMean and RemoveMean got their AVX codepaths.
   - PlanarFrame updated from JPSDR's NNEDI3 repo (x64 friendly, HBD ready).
   - proc0_16 got SSE2 codepath (I can see noticeable speed-up against old version).
   - opt parameter change: 2 - SSE/SSE2, 3 - SSE/SSE2/AVX codepath is used.

2009.12.25 - Firesledge
  Added the bool "lsb" parameter. It outputs 16-bit pixel components by
  separating most significant bytes (MSB) and least significant bytes (LSB).
  When the flag is set, the output frame height is doubled. The top part of
  the frame contains the MSB and the bottom part the LSB.
  Default: false.

2011.05.15 - Firesledge
  Added the bool "lsb_in" parameter to allow 16-bit input clips as stacked
  MSB-LSB.
  Default: false.

2011.11.28 - Firesledge
  Added the quiet parameter to deactivate the filter spectrum output.

2012.03.11 - Firesledge
  Fixed a stupid regression (from v1.8 mod16a) on the dither parameter.

2012.03.23 - Firesledge
  The quiet parameter is not true by default.

2012.04.20 - Firesledge
  Does no longer issue an error with null-length clips.

2013.07.31 - Firesledge
  Compatible with the new Avisynth 2.6 colorspaces, excepted Y8.
*/

#ifdef __INTEL_COMPILER
#pragma warning(disable : 693) 
#endif
#pragma warning(disable : 4305)

#include "dfttest.h"
#include "dfttest_avx.h"
#include <cassert>

PVideoFrame __stdcall dfttest::GetFrame(int n, IScriptEnvironment* env)
{
  if (tbsize == 1)
    return GetFrame_S(n, env);
  return GetFrame_T(n, env);
}

unsigned __stdcall threadPool(void* ps)
{
  const PS_INFO* pss = (PS_INFO*)ps;
  while (true)
  {
    WaitForSingleObject(pss->nextJob, INFINITE);
    if (pss->type < 0)
      return 0;
    if (!(pss->type & 2)) // tbsize=1
      func_0(ps);
    else
      func_1(ps);
    ResetEvent(pss->nextJob);
    SetEvent(pss->jobFinished);
  }
}

void proc0_C(const unsigned char* s0, const float* s1, float* d,
  const int p0, const int p1, const int /*offset_lsb*/)
{
  for (int u = 0; u < p1; ++u)
  {
    for (int v = 0; v < p1; ++v)
      d[v] = s0[v] * s1[v];
    s0 += p0;
    s1 += p1;
    d += p1;
  }
}

void proc0_SSE2_4(const unsigned char* s0, const float* s1, float* d,
  const int p0, const int p1, const int /*offset_lsb*/)
{
  auto zero = _mm_setzero_si128();
  for (int u = 0; u < p1; ++u)
  {
    for (int v = 0; v < p1; v += 4)
    {
      /* Intel's compiller works fine with aligned versions, MS randomly crashing */
      auto s0_load = _mm_loadu_si128(reinterpret_cast<const __m128i*>(s0 + v));
      auto s0_unp_lo1 = _mm_unpacklo_epi8(s0_load, zero);
      auto s0_unp_lo2 = _mm_unpacklo_epi8(s0_unp_lo1, zero);
      auto s0_loop = _mm_cvtepi32_ps(s0_unp_lo2);
      auto s1_loop = _mm_loadu_ps(s1 + v);
      auto d_reslt = _mm_mul_ps(s0_loop, s1_loop);
      _mm_storeu_ps(d + v, d_reslt);
    }
    s0 += p0;
    s1 += p1;
    d += p1;
  }
}

void proc0_SSE2_8(const unsigned char* s0, const float* s1, float* d,
  const int p0, const int p1, const int /*offset_lsb*/)
{
  auto zero = _mm_setzero_si128();
  for (int u = 0; u < p1; ++u)
  {
    for (int v = 0; v < p1; v += 8)
    {
      auto s0_load = _mm_loadu_si128(reinterpret_cast<const __m128i*>(s0 + v));
      auto s0_u_lo1 = _mm_unpacklo_epi8(s0_load, zero);
      auto s0_u_lo2 = _mm_unpacklo_epi8(s0_u_lo1, zero);
      auto s0_u_hi2 = _mm_unpackhi_epi8(s0_u_lo1, zero);
      auto s0_loop_lo = _mm_cvtepi32_ps(s0_u_lo2);
      auto s0_loop_hi = _mm_cvtepi32_ps(s0_u_hi2);
      auto s1_loop_lo = _mm_loadu_ps(s1 + v);
      auto s1_loop_hi = _mm_loadu_ps(s1 + v + 4);
      auto d_result1 = _mm_mul_ps(s0_loop_lo, s1_loop_lo);
      auto d_result2 = _mm_mul_ps(s0_loop_hi, s1_loop_hi);
      _mm_storeu_ps(d + v, d_result1);
      _mm_storeu_ps(d + v + 4, d_result2);
    }
    s0 += p0;
    s1 += p1;
    d += p1;
  }
}

void proc0_16_C(const unsigned char* s0, const float* s1, float* d,
  const int p0, const int p1, const int offset_lsb)
{
  for (int u = 0; u < p1; ++u)
  {
    for (int v = 0; v < p1; ++v)
    {
      const int		msb = s0[v];
      const int		lsb = s0[v + offset_lsb];
      const int		val = (msb << 8) + lsb;
      d[v] = val * s1[v] * (1.0f / 256);
    }
    s0 += p0;
    s1 += p1;
    d += p1;
  }
}

void proc0_16_SSE2(const unsigned char* s0, const float* s1, float* d,
  const int p0, const int p1, const int offset_lsb)
{
  auto zero = _mm_setzero_si128();
  auto round = _mm_set_ps1(1.0f / 256);
  for (int u = 0; u < p1; ++u)
  {
    for (int v = 0; v < p1; v += 4)
    {
      auto msb = _mm_loadu_si128(reinterpret_cast<const __m128i*>(s0 + v));
      auto lsb = _mm_loadu_si128(reinterpret_cast<const __m128i*>(s0 + v + offset_lsb));
      auto int_lo = _mm_unpacklo_epi8(lsb, msb);
      auto int_hi = _mm_unpackhi_epi8(lsb, msb);
      auto shift = _mm_unpacklo_epi8(_mm_slli_epi16(int_hi, 8), zero);
      auto add = _mm_add_epi16(shift, int_lo);
      auto s0_loop_i = _mm_unpacklo_epi16(add, zero);
      auto s0_loop = _mm_cvtepi32_ps(s0_loop_i);
      auto s1_loop = _mm_loadu_ps(s1 + v);
      auto d_result = _mm_mul_ps(s0_loop, s1_loop);
      d_result = _mm_mul_ps(d_result, round);
      _mm_storeu_ps(d + v, d_result);
    }
    s0 += p0;
    s1 += p1;
    d += p1;
  }
}

void proc1_C(const float* s0, const float* s1, float* d,
  const int p0, const int p1)
{
  for (int u = 0; u < p0; ++u)
  {
    for (int v = 0; v < p0; ++v)
      d[v] += s0[v] * s1[v];
    s0 += p0;
    s1 += p0;
    d += p1;
  }
}

void proc1_SSE_4(const float* s0, const float* s1, float* d,
  const int p0, const int p1)
{
  auto zero = _mm_setzero_si128();
  for (int u = 0; u < p0; ++u)
  {
    for (int v = 0; v < p0; v += 4)
    {
      auto s0_loop = _mm_loadu_ps(s0 + v);
      auto s1_loop = _mm_loadu_ps(s1 + v);
      auto d_loop = _mm_loadu_ps(d + v);
      auto d_mul = _mm_mul_ps(s0_loop, s1_loop);
      auto d_reslt = _mm_add_ps(d_loop, d_mul);
      _mm_storeu_ps(d + v, d_reslt);
    }
    s0 += p0;
    s1 += p0;
    d += p1;
  }
}

void proc1_SSE_8(const float* s0, const float* s1, float* d,
  const int p0, const int p1)
{
  for (int u = 0; u < p0; ++u)
  {
    for (int v = 0; v < p0; v += 8)
    {
      auto s0_loop1 = _mm_loadu_ps(s0 + v);
      auto s0_loop2 = _mm_loadu_ps(s0 + v + 4);
      auto s1_loop1 = _mm_loadu_ps(s1 + v);
      auto s1_loop2 = _mm_loadu_ps(s1 + v + 4);
      auto d_loop1 = _mm_loadu_ps(d + v);
      auto d_loop2 = _mm_loadu_ps(d + v + 4);
      auto d_mul1 = _mm_mul_ps(s0_loop1, s1_loop1);
      auto d_mul2 = _mm_mul_ps(s0_loop2, s1_loop2);
      auto d_reslt1 = _mm_add_ps(d_loop1, d_mul1);
      auto d_reslt2 = _mm_add_ps(d_loop2, d_mul2);
      _mm_storeu_ps(d + v, d_reslt1);
      _mm_storeu_ps(d + v + 4, d_reslt2);
    }
    s0 += p0;
    s1 += p0;
    d += p1;
  }
}

inline bool writeOk(const int* a, const int nthreads, const int x, const int y,
  const int sbsize)
{
  for (int i = 0; i < nthreads; ++i)
  {
    if (a[i * 2] != INT_MIN && abs(a[i * 2] - x) < sbsize &&
      abs(a[i * 2 + 1] - y) < sbsize)
      return false;
  }
  return true;
}

void func_0(void* ps)
{
  PS_INFO* pss = (PS_INFO*)ps;
  const int b = pss->b;
  const int sheight = pss->sheight[b];
  const int eheight = pss->eheight[b];
  const int width = pss->pf->GetWidth(b);
  const int src_pitch = pss->pf->GetPitch(b);
  const unsigned char* srcp = pss->pf->GetPtr(b) + sheight * src_pitch;
  float* ebp_saved = pss->ebuff[b] + width * sheight;
  const int sbsize = pss->sbsize;
  const int sbd1 = pss->sbsize >> 1;
  const int sosize = pss->sosize;
  LPCRITICAL_SECTION csect = pss->csect;
  const int nthreads = pss->nthreads;
  const int tidx = pss->tidx;
  int* cty = pss->cty;
  int* ctyT = cty + tidx * 2;
  float* dftr = pss->dftr;
  const float* hw = pss->hw;
  fftwf_complex* dftgc = pss->dftgc;
  fftwf_complex* dftc = pss->dftc;
  fftwf_complex* dftc2 = pss->dftc2;
  const bool zmean = pss->zmean;
  const int ccnt = pss->ccnt;
  const float f0beta = pss->f0beta;
  const bool uf0b = fabsf(f0beta - 1.0f) < 0.00005f ? false : true;
  const int inc = (pss->type & 1) ? sbsize - sosize : 1;
  const int offset_lsb = pss->ofs_lsb[b];
  for (int y = sheight; y < eheight; y += inc)
  {
    const bool tcheck = (nthreads > 1 && min(y - sheight, eheight - y) < sbsize) ? true : false;
    for (int x = 0; x <= width - sbsize; x += inc)
    {
      pss->proc0(srcp + x, hw, dftr, src_pitch, sbsize, offset_lsb);
      pss->fftwf_execute_dft_r2c(pss->ft, dftr, dftc);
      if (zmean)
        pss->removeMean((float*)dftc, (float*)dftgc, ccnt, (float*)dftc2);
      pss->filterCoeffs((float*)dftc, pss->sigmas, ccnt,
        uf0b ? &f0beta : pss->pmins, pss->pmaxs, pss->sigmas2);
      if (zmean)
        pss->addMean((float*)dftc, ccnt, (float*)dftc2);
      pss->fftwf_execute_dft_c2r(pss->fti, dftc, dftr);
      if (pss->type & 1) // spatial overlapping
      {
        if (tcheck) // potential thread conflict updating ebp
        {
        wloop:
          EnterCriticalSection(csect);
          // are any other threads updating pixels in this block?
          if (writeOk(cty, nthreads, x, y, sbsize))
          {
            ctyT[0] = x; // if not, set that this thread is
            ctyT[1] = y;
            LeaveCriticalSection(csect);
          }
          else // if so, wait
          {
            LeaveCriticalSection(csect);
            goto wloop;
          }
        }
        pss->proc1(dftr, hw, ebp_saved + x, sbsize, width);
        ctyT[0] = INT_MIN; // finished
      }
      else
        ebp_saved[x + sbd1 * width + sbd1] =
        dftr[sbd1 * sbsize + sbd1] * hw[sbd1 * sbsize + sbd1];
    }
    srcp += src_pitch * inc;
    ebp_saved += width * inc;
  }
}

void func_1(void* ps)
{
  PS_INFO* pss = (PS_INFO*)ps;
  nlCache* fc = pss->fc;
  const int b = pss->b;
  PlanarFrame* src = fc->frames[0]->ppf;
  const int sheight = pss->sheight[b];
  const int eheight = pss->eheight[b];
  const int width = src->GetWidth(b);
  const int src_pitch = src->GetPitch(b);
  float** ebp = pss->ebuff;
  const int sbsize = pss->sbsize;
  const int sbd1 = pss->sbsize >> 1;
  const int sosize = pss->sosize;
  LPCRITICAL_SECTION csect = pss->csect;
  const int nthreads = pss->nthreads;
  const int tidx = pss->tidx;
  int* cty = pss->cty;
  int* ctyT = cty + tidx * 2;
  const int stopz = pss->stopz;
  const int barea = pss->barea;
  const int pos = pss->pos;
  float* dftr = pss->dftr;
  const float* hw = pss->hw;
  fftwf_complex* dftgc = pss->dftgc;
  fftwf_complex* dftc = pss->dftc;
  fftwf_complex* dftc2 = pss->dftc2;
  const bool zmean = pss->zmean;
  const int ccnt = pss->ccnt;
  const float f0beta = pss->f0beta;
  const bool uf0b = fabsf(f0beta - 1.0f) < 0.00005f ? false : true;
  const unsigned char** pfplut = pss->pfplut;
  for (int i = 0; i < fc->size; ++i)
    pfplut[i] = fc->frames[fc->getCachePos(i)]->ppf->GetPtr(b) + src_pitch * sheight;
  const int inc = (pss->type & 1) ? sbsize - sosize : 1;
  const int offset_lsb = pss->ofs_lsb[b];
  for (int y = sheight; y < eheight; y += inc)
  {
    const bool tcheck = (nthreads > 1 && min(y - sheight, eheight - y) < sbsize) ? true : false;
    for (int x = 0; x <= width - sbsize; x += inc)
    {
      for (int z = 0; z <= stopz; ++z)
        pss->proc0(pfplut[z] + x, hw + sbsize * sbsize * z,
          dftr + sbsize * sbsize * z, src_pitch, sbsize, offset_lsb);
      pss->fftwf_execute_dft_r2c(pss->ft, dftr, dftc);
      if (zmean)
        pss->removeMean((float*)dftc, (float*)dftgc, ccnt, (float*)dftc2);
      pss->filterCoeffs((float*)dftc, pss->sigmas, ccnt,
        uf0b ? &f0beta : pss->pmins, pss->pmaxs, pss->sigmas2);
      if (zmean)
        pss->addMean((float*)dftc, ccnt, (float*)dftc2);
      pss->fftwf_execute_dft_c2r(pss->fti, dftc, dftr);
      if (pss->type & 1) // spatial overlapping
      {
        if (tcheck) // potential thread conflict updating ebp
        {
        wloop:
          EnterCriticalSection(csect);
          // are any other threads updating pixels this block?
          if (writeOk(cty, nthreads, x, y, sbsize))
          {
            ctyT[0] = x; // if not, set that this thread is
            ctyT[1] = y;
            LeaveCriticalSection(csect);
          }
          else // if so, wait
          {
            LeaveCriticalSection(csect);
            goto wloop;
          }
        }
        if (pss->type & 4) // temporal overlapping
        {
          for (int z = 0; z <= stopz; ++z)
          {
            pss->proc1(dftr + z * barea, hw + z * barea,
              ebp[z * 3 + b] + y * width + x, sbsize, width);
          }
        }
        else
        {
          pss->proc1(dftr + pos * barea, hw + pos * barea,
            ebp[b] + y * width + x, sbsize, width);
        }
        ctyT[0] = INT_MIN; // finished
      }
      else
      {
        if (pss->type & 4) // temporal overlapping
        {
          for (int z = 0; z <= stopz; ++z)
            ebp[z * 3 + b][(y + sbd1) * width + x + sbd1] +=
            dftr[z * barea + sbd1 * sbsize + sbd1] * hw[z * barea + sbd1 * sbsize + sbd1];
        }
        else
        {
          ebp[b][(y + sbd1) * width + x + sbd1] =
            dftr[pos * barea + sbd1 * sbsize + sbd1] * hw[pos * barea + sbd1 * sbsize + sbd1];
        }
      }
    }
    for (int q = 0; q < fc->size; ++q)
      pfplut[q] += src_pitch * inc;
  }
}

void intcast_C(const float* p, unsigned char* dst, const int src_height,
  const int src_width, const int dst_pitch, const int width)
{
  for (int y = 0; y < src_height; ++y)
  {
    for (int x = 0; x < src_width; ++x)
      dst[x] = min(max((int)(p[x] + 0.5f), 0), 255);
    p += width;
    dst += dst_pitch;
  }
}

void intcast_C_16_bits(const float* p, unsigned char* dst, unsigned char* dst_lsb, const int src_height,
  const int src_width, const int dst_pitch, const int width)
{
  const int		saved_rounding_mode = ::_controlfp(RC_NEAR, _MCW_RC);

  for (int y = 0; y < src_height; ++y)
  {
    for (int x = 0; x < src_width; ++x)
    {
      const float		vf = p[x] * 256;
      int				v;
      v = int(vf + 0.5f);
      v = min(max(v, 0), 65535);
      dst[x] = static_cast <unsigned char> (v >> 8);
      dst_lsb[x] = static_cast <unsigned char> (v);
    }
    p += width;
    dst += dst_pitch;
    dst_lsb += dst_pitch;
  }

  ::_controlfp(saved_rounding_mode, _MCW_RC);
}

void dither_C(const float* p, unsigned char* dst, const int src_height,
  const int src_width, const int dst_pitch, const int width, const int mode)
{
  float* dither = (float*)malloc(2 * width * sizeof(float));
  float* dc = dither;
  float* dn = dither + width;
  const float scale = (mode - 1) + 0.5f;
  const float off = scale * 0.5f;
  MTRand mtr;
  memset(dc, 0, width * sizeof(float));
  for (int y = 0; y < src_height; ++y)
  {
    memset(dn, 0, width * sizeof(float));
    for (int x = 0; x < src_width; ++x)
    {
      const int v = mode == 1 ?
        (int)(p[x] + dc[x] + 0.5f) :
        (int)(p[x] + mtr.randf() * scale - off + dc[x] + 0.5f);
      dst[x] = min(max(v, 0), 255);
      const float qerror = p[x] - dst[x];
      if (x != 0)
        dn[x - 1] += qerror * 0.1875f;
      dn[x] += qerror * 0.3125f;
      if (x != src_width - 1)
      {
        dc[x + 1] += qerror * 0.4375f;
        dn[x + 1] += qerror * 0.0625f;
      }
    }
    p += width;
    dst += dst_pitch;
    float* tn = dn;
    dn = dc;
    dc = tn;
  }
  free(dither);
}

void intcast_SSE2_8(const float* p, unsigned char* dst, const int src_height,
  const int src_width, const int dst_pitch, const int width)
{
  auto sse2_05 = _mm_set_ps1(0.5f);
  auto sse2_255 = _mm_set_ps1(255);
  auto sse2_0 = _mm_set_ps1(0);
  for (int y = 0; y < src_height; ++y)
  {
    for (int x = 0; x < src_width; x += 8)
    {
      auto p_loop = _mm_loadu_ps(p + x);
      auto p_loop2 = _mm_loadu_ps(p + x + 4);
      auto add_loop = _mm_add_ps(sse2_05, p_loop);
      auto add_loop2 = _mm_add_ps(sse2_05, p_loop2);
      auto max_loop = _mm_max_ps(add_loop, sse2_0);
      auto max_loop2 = _mm_max_ps(add_loop2, sse2_0);
      auto min_loop = _mm_min_ps(max_loop, sse2_255);
      auto min_loop2 = _mm_min_ps(max_loop2, sse2_255);
      auto int1_loop = _mm_cvttps_epi32(min_loop);
      auto int1_loop2 = _mm_cvttps_epi32(min_loop2);
      auto packs_loop = _mm_packs_epi32(int1_loop, int1_loop2);
      auto result = _mm_packus_epi16(packs_loop, packs_loop);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(dst + x), result);
    }
    p += width;
    dst += dst_pitch;
  }
}

PVideoFrame dfttest::GetFrame_S(int n, IScriptEnvironment* env)
{
  PVideoFrame src = child->GetFrame(mapn(n), env);
  nlf->pf->copyFrom(src, vi_src);
  copyPad(nlf->pf, nlf->ppf, env);
  for (int b = 0; b < 3; ++b)
  {
    if ((b == 0 && !Y) || (b == 1 && !U) || (b == 2 && !V))
    {
      bypass_plane(*(nlf->pf), b);
      continue;
    }
    const int height = nlf->ppf->GetHeight(b) / lsb_in_hmul;
    const int width = nlf->ppf->GetWidth(b);
    memset(ebuff[b], 0, height * width * sizeof(float));
    for (int i = 0; i < threads; ++i)
    {
      pssInfo[i]->b = b;
      ResetEvent(pssInfo[i]->jobFinished);
      SetEvent(pssInfo[i]->nextJob);
    }
    for (int i = 0; i < threads; ++i)
      WaitForSingleObject(pssInfo[i]->jobFinished, INFINITE);
    conv_result_plane_to_int(width, height, b, b, env);
  }
  return build_output_frame(env);
}

PVideoFrame dfttest::GetFrame_T(int n, IScriptEnvironment* env)
{
  int pos = 0;
  PlanarFrame* src = fc->frames[0]->ppf;
  if (tmode == 0)
  {
    for (int b = 0; b < 3; ++b)
    {
      if ((b == 0 && !Y) || (b == 1 && !U) || (b == 2 && !V))
        continue;
      memset(ebuff[b], 0, (src->GetHeight(b) / lsb_in_hmul) * src->GetWidth(b) * sizeof(float));
    }
    pos = tbsize >> 1;
    fc->resetCacheStart(n - pos, n + pos);
    for (int i = n - pos; i <= n + pos; ++i)
    {
      nlFrame* nl = fc->frames[fc->getCachePos(i - n + pos)];
      if (nl->fnum != i)
      {
        PVideoFrame src = child->GetFrame(mapn(i), env);
        nl->pf->copyFrom(src, vi_src);
        copyPad(nl->pf, nl->ppf, env);
        nl->setFNum(i);
      }
    }
    processTemporalBlock(pos);
  }
  else
  {
    int fbframe = -max((tbsize - tosize), tosize);
    for (; fbframe <= n - tbsize; fbframe += (tbsize - tosize));
    int lbframe = fbframe;
    for (; lbframe <= n - (tbsize - tosize); lbframe += (tbsize - tosize));
    if (ebframe >= fbframe && ebframe <= lbframe) // cached
    {
      pos = n - ebframe;
      fbframe = ebframe + (tbsize - tosize);
    }
    else
    {
      fbframe = -max((tbsize - tosize), tosize);
      for (; fbframe <= lbframe - tbsize; fbframe += (tbsize - tosize));
      for (int q = tbsize - tosize; q < tbsize; ++q)
      {
        for (int b = 0; b < 3; ++b)
        {
          if ((b == 0 && !Y) || (b == 1 && !U) || (b == 2 && !V))
            continue;
          memset(ebuff[q * 3 + b], 0, (src->GetHeight(b) / lsb_in_hmul) * src->GetWidth(b) * sizeof(float));
        }
      }
    }
    for (int z = fbframe; z <= lbframe; z += (tbsize - tosize))
    {
      for (int q = 0; q < tbsize - tosize; ++q)
      {
        float* tempb[3] = { ebuff[0], ebuff[1], ebuff[2] };
        for (int k = 0; k < tbsize - 1; ++k)
        {
          ebuff[k * 3 + 0] = ebuff[(k + 1) * 3 + 0];
          ebuff[k * 3 + 1] = ebuff[(k + 1) * 3 + 1];
          ebuff[k * 3 + 2] = ebuff[(k + 1) * 3 + 2];
        }
        ebuff[(tbsize - 1) * 3 + 0] = tempb[0];
        ebuff[(tbsize - 1) * 3 + 1] = tempb[1];
        ebuff[(tbsize - 1) * 3 + 2] = tempb[2];
      }
      for (int q = tosize; q < tbsize; ++q)
      {
        for (int b = 0; b < 3; ++b)
        {
          if ((b == 0 && !Y) || (b == 1 && !U) || (b == 2 && !V))
            continue;
          memset(ebuff[q * 3 + b], 0, (src->GetHeight(b) / lsb_in_hmul) * src->GetWidth(b) * sizeof(float));
        }
      }
      pos = n - z;
      fc->resetCacheStart(z, z + tbsize - 1);
      for (int i = z; i <= z + tbsize - 1; ++i)
      {
        nlFrame* nl = fc->frames[fc->getCachePos(i - z)];
        if (nl->fnum != i)
        {
          PVideoFrame src = child->GetFrame(mapn(i), env);
          nl->pf->copyFrom(src, vi_src);
          copyPad(nl->pf, nl->ppf, env);
          nl->setFNum(i);
        }
      }
      processTemporalBlock(pos);
    }
    ebframe = lbframe;
  }
  for (int b = 0; b < 3; ++b)
  {
    if ((b == 0 && !Y) || (b == 1 && !U) || (b == 2 && !V))
    {
      bypass_plane(*(fc->frames[fc->getCachePos(pos)]->pf), b);
      continue;
    }
    const int height = src->GetHeight(b) / lsb_in_hmul;
    const int width = src->GetWidth(b);
    conv_result_plane_to_int(width, height, b, tmode == 0 ? b : (pos * 3 + b), env);
  }
  return build_output_frame(env);
}

PVideoFrame dfttest::build_output_frame(IScriptEnvironment* env)
{
  PVideoFrame dst = env->NewVideoFrame(vi);
  if (lsb_out_flag)
  {
    merge_msb_lsb();
    dstPF_all->copyTo(dst, vi);
  }
  else
  {
    dstPF->copyTo(dst, vi);
  }

  return dst;
}

void dfttest::bypass_plane(PlanarFrame& frame, int plane)
{
  if (lsb_in_flag)
  {
    if (lsb_out_flag)
    {
      frame.copyPlaneTo16Part(*dstPF, plane, 0);
      frame.copyPlaneTo16Part(*dstPF_lsb, plane, 1);
    }
    else
    {
      frame.copyPlaneTo16Round(*dstPF, plane);
    }
  }
  else
  {
    frame.copyPlaneTo(*dstPF, plane);
    if (lsb_out_flag)
    {
      unsigned char* plane_ptr = dstPF_lsb->GetPtr(plane);
      const int		dp = dstPF_lsb->GetPitch(plane);
      const int		dh = dstPF_lsb->GetHeight(plane);
      memset(plane_ptr, 0, dp * dh);
    }
  }
}

void dfttest::conv_result_plane_to_int(int width, int height, int b, int ebuff_index, IScriptEnvironment* env)
{
  unsigned char* dstp = dstPF->GetPtr(b);
  const int dst_pitch = dstPF->GetPitch(b);
  const int src_width = dstPF->GetWidth(b);
  const int src_height = dstPF->GetHeight(b);
  float* ebp = ebuff[ebuff_index] + width * ((height - src_height) >> 1) + ((width - src_width) >> 1);
  long cpuflags = env->GetCPUFlags();
  if (lsb_out_flag)
  {
    assert(dstPF_lsb != 0);
    unsigned char* dst_lsb_p = dstPF_lsb->GetPtr(b);
    intcast_C_16_bits(
      ebp,
      dstp, dst_lsb_p,
      src_height, src_width,
      dst_pitch, width
      );
  }
  else
  {
    if (dither)
      dither_C(ebp, dstp, src_height, src_width, dst_pitch, width, dither);
    else if (!(src_width & 7) && (((cpuflags & CPUF_SSE2) && opt == 0) || opt == 3 || opt == 2))
      intcast_SSE2_8(ebp, dstp, src_height, src_width, dst_pitch, width);
    else
      intcast_C(ebp, dstp, src_height, src_width, dst_pitch, width);
  }
}

void dfttest::merge_msb_lsb()
{
  for (int b = 0; b < 3; ++b)
  {
    const int		width = dstPF->GetWidth(b);
    const int		height = dstPF->GetHeight(b);

    const int		msb_pitch = dstPF->GetPitch(b);
    const unsigned char* msb_ptr = dstPF->GetPtr(b);

    const int		lsb_pitch = dstPF_lsb->GetPitch(b);
    const unsigned char* lsb_ptr = dstPF_lsb->GetPtr(b);

    const int		all_pitch = dstPF_all->GetPitch(b);
    unsigned char* all_ptr = dstPF_all->GetPtr(b);

    for (int y = 0; y < height; ++y)
    {
      memcpy(
        all_ptr + y * all_pitch,
        msb_ptr + y * msb_pitch,
        width
        );
      memcpy(
        all_ptr + (height + y) * all_pitch,
        lsb_ptr + y * lsb_pitch,
        width
        );
    }
  }
}

void dfttest::processTemporalBlock(int pos)
{
  for (int i = 0; i < threads; ++i)
    pssInfo[i]->pos = pos;
  for (int b = 0; b < 3; ++b)
  {
    if ((b == 0 && !Y) || (b == 1 && !U) || (b == 2 && !V))
      continue;
    for (int i = 0; i < threads; ++i)
    {
      pssInfo[i]->b = b;
      ResetEvent(pssInfo[i]->jobFinished);
      SetEvent(pssInfo[i]->nextJob);
    }
    for (int i = 0; i < threads; ++i)
      WaitForSingleObject(pssInfo[i]->jobFinished, INFINITE);
  }
}

void removeMean_C(float* dftc, const float* dftgc, const int ccnt, float* dftc2)
{
  const float gf = dftc[0] / dftgc[0];
  for (int h = 0; h < ccnt; h += 2)
  {
    dftc2[h + 0] = gf * dftgc[h + 0];
    dftc2[h + 1] = gf * dftgc[h + 1];
    dftc[h + 0] -= dftc2[h + 0];
    dftc[h + 1] -= dftc2[h + 1];
  }
}

void removeMean_SSE(float* dftc, const float* dftgc, const int ccnt, float* dftc2)
{
  const float gf = dftc[0] / dftgc[0];
  auto gf_asm = _mm_load_ps1(reinterpret_cast<const float*>(&gf));
  for (int h = 0; h < ccnt; h += 4)
  {
    auto dftc_loop = _mm_loadu_ps(dftc + h);
    auto dftgc_loop = _mm_loadu_ps(dftgc + h);
    auto dftc2_loop = _mm_loadu_ps(dftc2 + h);
    auto dftc2_result = _mm_mul_ps(gf_asm, dftgc_loop);
    auto dftc_result = _mm_sub_ps(dftc_loop, dftc2_result);
    _mm_storeu_ps(dftc2 + h, dftc2_result);
    _mm_storeu_ps(dftc + h, dftc_result);
  }
}

void addMean_C(float* dftc, const int ccnt, const float* dftc2)
{
  for (int h = 0; h < ccnt; h += 2)
  {
    dftc[h + 0] += dftc2[h + 0];
    dftc[h + 1] += dftc2[h + 1];
  }
}

void addMean_SSE(float* dftc, const int ccnt, const float* dftc2)
{
  for (int h = 0; h < ccnt; h += 4)
  {
    auto dftc_loop = _mm_loadu_ps(dftc + h);
    auto dftc2_loop = _mm_loadu_ps(dftc2 + h);
    auto dftc_result = _mm_add_ps(dftc2_loop, dftc_loop);
    _mm_storeu_ps(dftc + h, dftc_result);
  }
}

void filter_0_C(float* dftc, const float* sigmas, const int ccnt,
  const float* pmin, const float* pmax, const float* sigmas2)
{
  for (int h = 0; h < ccnt; h += 2)
  {
    const float psd = dftc[h + 0] * dftc[h + 0] + dftc[h + 1] * dftc[h + 1];
    const float coeff = max((psd - sigmas[h]) / (psd + 1e-15f), 0.0f);
    dftc[h + 0] *= coeff;
    dftc[h + 1] *= coeff;
  }
}

void filter_0_SSE(float* dftc, const float* sigmas, const int ccnt,
  const float* pmin, const float* pmax, const float* sigmas2)
{
  auto zero = _mm_setzero_ps();
  auto sse_1em15 = _mm_set_ps1(1e-15f);
  for (int h = 0; h < ccnt; h += 4)
  {
    auto dftc_loop = _mm_loadu_ps(dftc + h);
    auto sigmas_loop = _mm_loadu_ps(sigmas + h);
    auto mul1_loop = _mm_mul_ps(dftc_loop, dftc_loop);
    auto shuff_loop = _mm_shuffle_ps(mul1_loop, mul1_loop, 177);
    auto add1_loop = _mm_add_ps(mul1_loop, shuff_loop);
    auto sub1_loop = _mm_sub_ps(add1_loop, sigmas_loop);
    auto add2_loop = _mm_add_ps(add1_loop, sse_1em15);
    auto rcp1_loop = _mm_rcp_ps(add2_loop);
    auto mul2_loop = _mm_mul_ps(sub1_loop, rcp1_loop);
    auto max1_loop = _mm_max_ps(zero, mul2_loop);
    auto mul3_loop = _mm_mul_ps(dftc_loop, max1_loop);
    _mm_storeu_ps(dftc + h, mul3_loop);
  }
}

void filter_1_C(float* dftc, const float* sigmas, const int ccnt,
  const float* pmin, const float* pmax, const float* sigmas2)
{
  for (int h = 0; h < ccnt; h += 2)
  {
    const float psd = dftc[h + 0] * dftc[h + 0] + dftc[h + 1] * dftc[h + 1];
    if (psd < sigmas[h])
      dftc[h + 0] = dftc[h + 1] = 0.0f;
  }
}

void filter_1_SSE(float* dftc, const float* sigmas, const int ccnt,
  const float* pmin, const float* pmax, const float* sigmas2)
{
  for (int h = 0; h < ccnt; h += 4)
  {
    auto dftc_loop = _mm_loadu_ps(dftc + h);
    auto sigmas_loop = _mm_loadu_ps(sigmas + h);
    auto mul1_loop = _mm_mul_ps(dftc_loop, dftc_loop);
    auto shuff_loop = _mm_shuffle_ps(mul1_loop, mul1_loop, 177);
    auto add1_loop = _mm_add_ps(mul1_loop, shuff_loop);
    auto cmple1_loop = _mm_cmple_ps(sigmas_loop, add1_loop);
    auto and1_loop = _mm_and_ps(cmple1_loop, dftc_loop);
    _mm_storeu_ps(dftc + h, and1_loop);
  }
}

void filter_2_C(float* dftc, const float* sigmas, const int ccnt,
  const float* pmin, const float* pmax, const float* sigmas2)
{
  for (int h = 0; h < ccnt; h += 2)
  {
    dftc[h + 0] *= sigmas[h];
    dftc[h + 1] *= sigmas[h];
  }
}

void filter_2_SSE(float* dftc, const float* sigmas, const int ccnt,
  const float* pmin, const float* pmax, const float* sigmas2)
{
  for (int h = 0; h < ccnt; h += 4)
  {
    auto dftc_loop = _mm_loadu_ps(dftc + h);
    auto sigmas_loop = _mm_loadu_ps(sigmas + h);
    auto mul_loop = _mm_mul_ps(dftc_loop, sigmas_loop);
    _mm_storeu_ps(dftc + h, mul_loop);
  }
}

void filter_3_C(float* dftc, const float* sigmas, const int ccnt,
  const float* pmin, const float* pmax, const float* sigmas2)
{
  for (int h = 0; h < ccnt; h += 2)
  {
    const float psd = dftc[h + 0] * dftc[h + 0] + dftc[h + 1] * dftc[h + 1];
    if (psd >= pmin[h] && psd <= pmax[h])
    {
      dftc[h + 0] *= sigmas[h];
      dftc[h + 1] *= sigmas[h];
    }
    else
    {
      dftc[h + 0] *= sigmas2[h];
      dftc[h + 1] *= sigmas2[h];
    }
  }
}

void filter_3_SSE(float* dftc, const float* sigmas, const int ccnt,
  const float* pmin, const float* pmax, const float* sigmas2)
{
  auto sse_ones = _mm_set_ps1(0xFFFFFFFFFFFFFFFF);
  for (int h = 0; h < ccnt; h += 4)
  {
    auto dftc_loop = _mm_loadu_ps(dftc + h);
    auto sigmas_loop = _mm_loadu_ps(sigmas + h);
    auto sigmas2_loop = _mm_loadu_ps(sigmas2 + h);
    auto pmin_loop = _mm_loadu_ps(pmin + h);
    auto pmax_loop = _mm_loadu_ps(pmax + h);
    auto mul1_loop = _mm_mul_ps(dftc_loop, dftc_loop);
    auto shuff_loop = _mm_shuffle_ps(mul1_loop, mul1_loop, 177);
    auto add1_loop = _mm_add_ps(mul1_loop, shuff_loop);
    auto cmpnle1_loop = _mm_cmpnle_ps(pmax_loop, add1_loop);
    auto cmple1_loop = _mm_cmple_ps(pmin_loop, add1_loop);
    auto and1_loop = _mm_and_ps(cmple1_loop, cmpnle1_loop);
    auto and2_loop = _mm_and_ps(sigmas_loop, and1_loop);
    auto xor1_loop = _mm_xor_ps(sse_ones, and1_loop);
    auto and3_loop = _mm_and_ps(xor1_loop, sigmas2_loop);
    auto  or1_loop = _mm_or_ps(and3_loop, and2_loop);
    auto mul2_loop = _mm_mul_ps(or1_loop, dftc_loop);
    _mm_storeu_ps(dftc + h, mul2_loop);
  }
}

void filter_4_C(float* dftc, const float* sigmas, const int ccnt,
  const float* pmin, const float* pmax, const float* sigmas2)
{
  for (int h = 0; h < ccnt; h += 2)
  {
    const float psd = dftc[h + 0] * dftc[h + 0] + dftc[h + 1] * dftc[h + 1] + 1e-15f;
    const float mult = sigmas[h] * sqrtf((psd * pmax[h]) / ((psd + pmin[h]) * (psd + pmax[h])));
    dftc[h + 0] *= mult;
    dftc[h + 1] *= mult;
  }
}

void filter_4_SSE(float* dftc, const float* sigmas, const int ccnt,
  const float* pmin, const float* pmax, const float* sigmas2)
{
  auto sse_1em15 = _mm_set_ps1(1e-15f);
  for (int h = 0; h < ccnt; h += 4)
  {
    auto dftc_loop = _mm_loadu_ps(dftc + h);
    auto sigmas_loop = _mm_loadu_ps(sigmas + h);
    auto pmin_loop = _mm_loadu_ps(pmin + h);
    auto pmax_loop = _mm_loadu_ps(pmax + h);
    auto mul1_loop = _mm_mul_ps(dftc_loop, dftc_loop);
    auto shuff_loop = _mm_shuffle_ps(mul1_loop, mul1_loop, 177);
    auto add1_loop = _mm_add_ps(sse_1em15, mul1_loop);
    auto add2_loop = _mm_add_ps(add1_loop, shuff_loop);
    auto add3_loop = _mm_add_ps(add2_loop, pmin_loop);
    auto add4_loop = _mm_add_ps(add2_loop, pmax_loop);
    auto mul2_loop = _mm_mul_ps(add2_loop, pmax_loop);
    auto mul3_loop = _mm_mul_ps(add4_loop, add3_loop);
    auto rcp1_loop = _mm_rcp_ps(mul3_loop);
    auto mul4_loop = _mm_mul_ps(rcp1_loop, mul2_loop);
    auto sqrt_loop = _mm_sqrt_ps(mul4_loop);
    auto mul5_loop = _mm_mul_ps(sigmas_loop, sqrt_loop);
    auto mul6_loop = _mm_mul_ps(mul5_loop, dftc_loop);
    _mm_storeu_ps(dftc + h, mul6_loop);
  }
}

void filter_5_C(float* dftc, const float* sigmas, const int ccnt,
  const float* pmin, const float* pmax, const float* sigmas2)
{
  const float beta = pmin[0];
  for (int h = 0; h < ccnt; h += 2)
  {
    const float psd = dftc[h + 0] * dftc[h + 0] + dftc[h + 1] * dftc[h + 1];
    const float coeff = powf(max((psd - sigmas[h]) / (psd + 1e-15f), 0.0f), beta);
    dftc[h + 0] *= coeff;
    dftc[h + 1] *= coeff;
  }
}

void filter_5_SSE2(float* dftc, const float* sigmas, const int ccnt,
  const float* pmin, const float* pmax, const float* sigmas2)
{
  auto zero = _mm_setzero_ps();
  auto sse_1em15 = _mm_set_ps1(1e-15f);
  auto pmin_zero = _mm_set_ps1(pmin[0]);
  for (int h = 0; h < ccnt; h += 4)
  {
    auto dftc_loop = _mm_loadu_ps(dftc + h);
    auto sigmas_loop = _mm_loadu_ps(sigmas + h);
    auto mul1_loop = _mm_mul_ps(dftc_loop, dftc_loop);
    auto shuff_loop = _mm_shuffle_ps(mul1_loop, mul1_loop, 177);
    auto add1_loop = _mm_add_ps(shuff_loop, mul1_loop);
    auto sub1_loop = _mm_sub_ps(add1_loop, sigmas_loop);
    auto add2_loop = _mm_add_ps(sse_1em15, add1_loop);
    auto rcp1_loop = _mm_rcp_ps(add2_loop);
    auto mul3_loop = _mm_mul_ps(rcp1_loop, sub1_loop);
    auto max1_loop = _mm_max_ps(zero, mul3_loop);
    auto pow1_loop = fmath::pow_ps(max1_loop, pmin_zero); // DJATOM: fmath is the most accurate in comparison to powf() (from what I tested)
    auto mul4_loop = _mm_mul_ps(dftc_loop, pow1_loop);
    _mm_storeu_ps(dftc + h, mul4_loop);
  }
}

void filter_6_C(float* dftc, const float* sigmas, const int ccnt,
  const float* pmin, const float* pmax, const float* sigmas2)
{
  for (int h = 0; h < ccnt; h += 2)
  {
    const float psd = dftc[h + 0] * dftc[h + 0] + dftc[h + 1] * dftc[h + 1];
    const float coeff = sqrtf(max((psd - sigmas[h]) / (psd + 1e-15f), 0.0f));
    dftc[h + 0] *= coeff;
    dftc[h + 1] *= coeff;
  }
}

void filter_6_SSE(float* dftc, const float* sigmas, const int ccnt,
  const float* pmin, const float* pmax, const float* sigmas2)
{
  auto sse_1em15 = _mm_set_ps1(1e-15f);
  for (int h = 0; h < ccnt; h += 4)
  {
    auto dftc_loop = _mm_loadu_ps(dftc + h);
    auto sigmas_loop = _mm_loadu_ps(sigmas + h);
    auto mul1_loop = _mm_mul_ps(dftc_loop, dftc_loop);
    auto shuff_loop = _mm_shuffle_ps(mul1_loop, mul1_loop, 177);
    auto add1_loop = _mm_add_ps(shuff_loop, mul1_loop);
    auto sub1_loop = _mm_sub_ps(add1_loop, sigmas_loop);
    auto add2_loop = _mm_add_ps(sse_1em15, add1_loop);
    auto rcp1_loop = _mm_rcp_ps(add2_loop);
    auto mul2_loop = _mm_mul_ps(rcp1_loop, sub1_loop);
    auto max1_loop = _mm_max_ps(sse_1em15, mul2_loop);
    auto rsq1_loop = _mm_rsqrt_ps(max1_loop);
    auto rcp2_loop = _mm_rcp_ps(rsq1_loop);
    auto mul3_loop = _mm_mul_ps(dftc_loop, rcp2_loop);
    _mm_storeu_ps(dftc + h, mul3_loop);
  }
}

void dfttest::copyPad(PlanarFrame* src, PlanarFrame* dst, IScriptEnvironment* env)
{
  for (int b = 0; b < 3; ++b)
  {
    if ((b == 0 && !Y) || (b == 1 && !U) || (b == 2 && !V))
      continue;

    for (int part = 0; part < lsb_in_hmul; ++part)
    {
      const int		src_width = src->GetWidth(b);
      const int		src_height = src->GetHeight(b) / lsb_in_hmul;
      const int		src_pitch = src->GetPitch(b);
      const int		dst_width = dst->GetWidth(b);
      const int		dst_height = dst->GetHeight(b) / lsb_in_hmul;
      const int		dst_pitch = dst->GetPitch(b);
      const int		offy = (dst_height - src_height) >> 1;
      const int		offx = (dst_width - src_width) >> 1;
      const unsigned char* srcp_base = src->GetPtr(b) + src_pitch * src_height * part;
      unsigned char* dstp_base = dst->GetPtr(b) + dst_pitch * dst_height * part;
      unsigned char* dstp = dstp_base + dst_pitch * offy;
      env->BitBlt(
        dstp + offx, dst_pitch,
        srcp_base, src_pitch,
        src_width, src_height
        );
      for (int y = offy; y < src_height + offy; ++y)
      {
        int w = offx * 2;
        for (int x = 0; x < offx; ++x, --w)
        {
          dstp[x] = dstp[w];
        }
        w = offx + src_width - 2;
        for (int x = offx + src_width; x < dst_width; ++x, --w)
        {
          dstp[x] = dstp[w];
        }
        dstp += dst_pitch;
      }
      int w = offy * 2;
      for (int y = 0; y < offy; ++y, --w)
      {
        env->BitBlt(
          dstp_base + dst_pitch * y, dst_pitch,
          dstp_base + dst_pitch * w, dst_pitch,
          dst_width, 1
          );
      }
      w = offy + src_height - 2;
      for (int y = offy + src_height; y < dst_height; ++y, --w)
      {
        env->BitBlt(
          dstp_base + dst_pitch * y, dst_pitch,
          dstp_base + dst_pitch * w, dst_pitch,
          dst_width, 1
          );
      }
    }
  }
}

int dfttest::mapn(int n)
{
  if (n < 0) return 0;
  if (n >= vi_src.num_frames) return vi_src.num_frames - 1;
  return n;
}

nlFrame::nlFrame()
{
  fnum = -20;
  pf = ppf = NULL;
}

nlFrame::nlFrame(PlanarFrame* tp, VideoInfo& vi)
{
  fnum = -20;
  pf = new PlanarFrame(vi);
  ppf = new PlanarFrame();
  ppf->createFromPlanar(*tp);
}

nlFrame::~nlFrame()
{
  if (pf) delete pf;
  if (ppf) delete ppf;
}

void nlFrame::setFNum(int i)
{
  fnum = i;
}

nlCache::nlCache()
{
  frames = NULL;
  start_pos = size = -20;
}

nlCache::nlCache(int _size, PlanarFrame* tp, VideoInfo& vi)
{
  frames = NULL;
  start_pos = size = -20;
  if (_size > 0)
  {
    start_pos = 0;
    size = _size;
    frames = (nlFrame**)malloc(size * sizeof(nlFrame*));
    memset(frames, 0, size * sizeof(nlFrame*));
    for (int i = 0; i < size; ++i)
      frames[i] = new nlFrame(tp, vi);
  }
}

nlCache::~nlCache()
{
  if (frames)
  {
    for (int i = 0; i < size; ++i)
    {
      if (frames[i])
        delete frames[i];
    }
    free(frames);
  }
}

void nlCache::resetCacheStart(int first, int last)
{
  for (int j = first; j <= last; ++j)
  {
    for (int i = 0; i < size; ++i)
    {
      if (frames[i]->fnum == j)
      {
        start_pos = i - j + first;
        if (start_pos < 0) start_pos += size;
        else if (start_pos >= size) start_pos -= size;
        return;
      }
    }
  }
}

int nlCache::getCachePos(int n)
{
  return (start_pos + n) % size;
}

dfttest::dfttest(PClip _child, bool _Y, bool _U, bool _V, int _ftype, float _sigma,
  float _sigma2, float _pmin, float _pmax, int _sbsize, int _smode, int _sosize,
  int _tbsize, int _tmode, int _tosize, int _swin, int _twin, double _sbeta,
  double _tbeta, bool _zmean, const char* _sfile, const char* _sfile2,
  const char* _pminfile, const char* _pmaxfile, float _f0beta, const char* _nfile,
  int _threads, int _opt, const char* _nstring, const char* _sstring,
  const char* _ssx, const char* _ssy, const char* _sst, const int _dither,
  bool _lsb_in_flag, bool _lsb_out_flag, bool _quiet_flag,
  IScriptEnvironment* env)
  : GenericVideoFilter(_child), Y(_Y), U(_U), V(_V),
  ftype(_ftype), sigma(_sigma), sigma2(_sigma2), pmin(_pmin), pmax(_pmax),
  sbsize(_sbsize), smode(_smode), sosize(_sosize), tbsize(_tbsize), tmode(_tmode),
  tosize(_tosize), swin(_swin), twin(_twin), sbeta(_sbeta), tbeta(_tbeta),
  zmean(_zmean), sfile(_sfile), sfile2(_sfile2), pminfile(_pminfile),
  pmaxfile(_pmaxfile), opt(_opt), threads(_threads), dither(_dither),
  lsb_in_flag(_lsb_in_flag), lsb_out_flag(_lsb_out_flag),
  quiet_flag(_quiet_flag)
{
  ft = fti = ftg = NULL;
  ebuff = NULL;
  hw = sigmas = sigmas2 = NULL;
  pmins = pmaxs = NULL;
  dftgc = NULL;
  dstPF = NULL;
  dstPF_lsb = NULL;
  dstPF_all = NULL;
  fc = NULL;
  nlf = NULL;
  hLib = NULL;
  ebframe = INT_MIN;
  vi_src = vi;
  vi_byte = vi;

  proc_height = vi.height;
  lsb_in_hmul = 1;
  if (lsb_in_flag)
  {
    if ((proc_height & 1) != 0)
    {
      env->ThrowError("dfttest:  actual clip height must be even!");
    }
    proc_height >>= 1;
    vi_byte.height = proc_height;
    vi.height = proc_height;
    lsb_in_hmul = 2;
  }

  if (!vi.IsYUY2() && !(vi.IsYUV() && vi.IsPlanar() && !vi.IsY8()))
    env->ThrowError("dfttest:  only YUV and YUY2 input supported!");
  if (ftype < 0 || ftype > 4)
    env->ThrowError("dfttest:  ftype must be set to 0, 1, 2, 3, or 4!");
  if (tbsize < 1)
    env->ThrowError("dfttest:  tbsize must be >= 1!");
  if (tmode != 0 && tmode != 1)
    env->ThrowError("dfttest:  tmode must be set to 0 or 1!");
  if (!(tbsize & 1) && tmode == 0)
    env->ThrowError("dfttest:  tbsize must be odd when using tmode=0!");
  if (tmode == 0)
    tosize = 0;
  if (tosize < 0)
    env->ThrowError("dfttest:  tosize must be >= 0!");
  if (tosize >= tbsize)
    env->ThrowError("dfttest:  tosize must be < tbsize!");
  if (tbsize > vi.num_frames && vi.num_frames > 0)
    env->ThrowError("dfttest:  tbsize must be less than or equal to the " \
      "number of frames in the clip!");
  if (tosize > (tbsize >> 1) && (tbsize % (tbsize - tosize)) != 0)
    env->ThrowError("dfttest:  temporal overlap greater than 50%c " \
      "requires that (tbsize-tosize) is a divisor of tbsize!", '%');
  if (twin < 0 || twin > 11)
    env->ThrowError("dfttest:  twin must be between 0 and 11 (inclusive)!");
  if (sbsize < 1)
    env->ThrowError("dfttest:  sbsize must be >= 1!");
  if (smode != 0 && smode != 1)
    env->ThrowError("dfttest:  smode must be set to 0 or 1!");
  if (!(sbsize & 1) && smode == 0)
    env->ThrowError("dfttest:  sbsize must be odd when using smode=0!");
  if (smode == 0)
    sosize = 0;
  if (sosize < 0)
    env->ThrowError("dfttest:  sosize must be >= 0!");
  if (sosize >= sbsize)
    env->ThrowError("dfttest:  sosize must be < sbsize!");
  if (sosize > (sbsize >> 1) && (sbsize % (sbsize - sosize)) != 0)
    env->ThrowError("dfttest:  spatial overlap greater than 50%c " \
      "requires that (sbsize-sosize) is a divisor of sbsize!", '%');
  if (swin < 0 || swin > 11)
    env->ThrowError("dfttest:  swin must be between 0 and 11 (inclusive)!");
  if (threads < 0 || threads > 16)
    env->ThrowError("dfttest:  threads must be between 0 and 16 (inclusive)!");
  if (opt < 0 || opt > 3)
    env->ThrowError("dfttest:  opt must be set to 0, 1, 2, or 3!");
  if (dither < 0 || dither > 100)
    env->ThrowError("dfttest:  invalid dither value!\n");
  if (threads == 0)
    threads = num_processors();
  hLib = LoadLibrary("libfftw3f-3.dll");
  if (hLib)
  {
    fftwf_destroy_plan =
      (fftwf_destroy_plan_proc)GetProcAddress(hLib, "fftwf_destroy_plan");
    fftwf_plan_dft_r2c_3d =
      (fftwf_plan_dft_r2c_3d_proc)GetProcAddress(hLib, "fftwf_plan_dft_r2c_3d");
    fftwf_plan_dft_c2r_3d =
      (fftwf_plan_dft_c2r_3d_proc)GetProcAddress(hLib, "fftwf_plan_dft_c2r_3d");
    fftwf_plan_dft_r2c_2d =
      (fftwf_plan_dft_r2c_2d_proc)GetProcAddress(hLib, "fftwf_plan_dft_r2c_2d");
    fftwf_plan_dft_c2r_2d =
      (fftwf_plan_dft_c2r_2d_proc)GetProcAddress(hLib, "fftwf_plan_dft_c2r_2d");
    fftwf_execute_dft_r2c =
      (fftwf_execute_dft_r2c_proc)GetProcAddress(hLib, "fftwf_execute_dft_r2c");
    fftwf_execute_dft_c2r =
      (fftwf_execute_dft_c2r_proc)GetProcAddress(hLib, "fftwf_execute_dft_c2r");
  }
  if (!hLib || !fftwf_destroy_plan || !fftwf_execute_dft_r2c ||
    !fftwf_execute_dft_c2r || !fftwf_plan_dft_r2c_3d || !fftwf_plan_dft_c2r_3d ||
    !fftwf_plan_dft_r2c_2d || !fftwf_plan_dft_c2r_2d)
    env->ThrowError("dfttest:  unable to load libfftw3f-3.dll!");
  child->SetCacheHints(CACHE_NOTHING, 0);
  barea = sbsize * sbsize;
  bvolume = barea * tbsize;
  norm = 1.0f / float(bvolume);
  ccnt = ((sbsize >> 1) + 1) * sbsize * tbsize;
  const int ssxuv = vi.GetPlaneWidthSubsampling(PLANAR_U);
  const int ssyuv = vi.GetPlaneHeightSubsampling(PLANAR_U);
  if (smode == 0)
  {
    const int ae = (sbsize >> 1) << 1;
    noxl = vi.width + ae;
    noyl = proc_height + ae;
    noxc = (vi.width >> ssxuv) + ae;
    noyc = (proc_height >> ssyuv) + ae;
  }
  else
  {
    const int ae = max(sbsize - sosize, sosize) * 2;
    noxl = vi.width + EXTRA(vi.width, sbsize) + ae;
    noyl = proc_height + EXTRA(proc_height, sbsize) + ae;
    noxc = (vi.width >> ssxuv) + EXTRA(vi.width >> ssxuv, sbsize) + ae;
    noyc = (proc_height >> ssyuv) + EXTRA(proc_height >> ssyuv, sbsize) + ae;
  }
  PlanarFrame* padPF = new PlanarFrame();
  padPF->createPlanar(noyl * lsb_in_hmul, noyc * lsb_in_hmul, noxl, noxc, false, false, 1, 8); // DJATOM: line updated with hardcoded defaults. Now it works with 8-bit frame using updated PlanarFrame
  if (tbsize > 1)
    fc = new nlCache(tbsize, padPF, vi_src);
  else
    nlf = new nlFrame(padPF, vi_src);
  const int ebcount = (tbsize > 1 && tmode == 1) ? tbsize : 1;
  ebuff = (float**)calloc(ebcount * 3, sizeof(float*));
  for (int q = 0; q < ebcount; ++q)
  {
    for (int b = 0; b < 3; ++b)
    {
      if ((b == 0 && !Y) || (b == 1 && !U) || (b == 2 && !V))
        continue;
      ebuff[q * 3 + b] = (float*)_aligned_malloc(padPF->GetWidth(b) *
      (padPF->GetHeight(b) / lsb_in_hmul) * sizeof(float), 16);
    }
  }
  delete padPF;
  dstPF = new PlanarFrame(vi_byte);
  if (lsb_out_flag)
  {
    dstPF_lsb = new PlanarFrame(vi_byte);
    assert(dstPF_lsb->GetPitch(PLANAR_Y) == dstPF->GetPitch(PLANAR_Y));
    assert(dstPF_lsb->GetPitch(PLANAR_U) == dstPF->GetPitch(PLANAR_U));
    assert(dstPF_lsb->GetPitch(PLANAR_V) == dstPF->GetPitch(PLANAR_V));
  }
  int w = 0;
  hw = (float*)_aligned_malloc(bvolume * sizeof(float), 16);
  createWindow(hw, tmode, tbsize, tosize, twin, tbeta, smode, sbsize, sosize, swin, sbeta);
  float* dftgr = (float*)_aligned_malloc(bvolume * sizeof(float), 16);
  dftgc = (fftwf_complex*)_aligned_malloc(sizeof(fftwf_complex) * (ccnt + 11), 16);
  if (tbsize > 1)
    ftg = fftwf_plan_dft_r2c_3d(tbsize, sbsize, sbsize, dftgr, dftgc,
      FFTW_PATIENT | FFTW_DESTROY_INPUT);
  else
    ftg = fftwf_plan_dft_r2c_2d(sbsize, sbsize, dftgr, dftgc,
      FFTW_PATIENT | FFTW_DESTROY_INPUT);
  float wscale = 0.0f;
  const float* hwT = hw;
  float* dftgrT = dftgr;
  for (int s = 0; s < tbsize; ++s)
  {
    for (int i = 0; i < sbsize; ++i)
    {
      for (int k = 0; k < sbsize; ++k)
      {
        dftgrT[k] = 255.0f * hwT[k];
        wscale += hwT[k] * hwT[k];
      }
      dftgrT += sbsize;
      hwT += sbsize;
    }
  }
  wscale = 1.0f / wscale;
  const float wscalef = ftype < 2 ? wscale : 1.0f;
  char buf[512];
  sprintf(buf, "dfttest:  scaling factor = %f\n", wscale);
  OutputDebugString(buf);
  fftwf_execute_dft_r2c(ftg, dftgr, dftgc);
  sigmas = (float*)_aligned_malloc((ccnt * 2 + 11) * sizeof(float), 16);
  sigmas2 = (float*)_aligned_malloc((ccnt * 2 + 11) * sizeof(float), 16);
  pmins = (float*)_aligned_malloc((ccnt * 2 + 11) * sizeof(float), 16);
  pmaxs = (float*)_aligned_malloc((ccnt * 2 + 11) * sizeof(float), 16);
  if (_sstring[0] || _ssx[0] || _ssy[0] || _sst[0])
    sigmaFromString(sigmas, _sstring, _ssx, _ssy, _sst, wscalef, env);
  else if (*sfile)
    loadFile(sigmas, sfile, wscalef, env);
  else
  {
    for (int i = 0; i < ccnt * 2; ++i)
      sigmas[i] = sigma / wscalef;
  }
  if (*sfile2)
    loadFile(sigmas2, sfile2, wscalef, env);
  else
  {
    for (int i = 0; i < ccnt * 2; ++i)
      sigmas2[i] = sigma2 / wscalef;
  }
  if (*pminfile)
    loadFile(pmins, pminfile, wscale, env);
  else
  {
    for (int i = 0; i < ccnt * 2; ++i)
      pmins[i] = pmin / wscale;
  }
  if (*pmaxfile)
    loadFile(pmaxs, pmaxfile, wscale, env);
  else
  {
    for (int i = 0; i < ccnt * 2; ++i)
      pmaxs[i] = pmax / wscale;
  }
  fftwf_complex* ta = (fftwf_complex*)_aligned_malloc(sizeof(fftwf_complex) * (ccnt + 3), 16);
  if (tbsize > 1)
  {
    ft = fftwf_plan_dft_r2c_3d(tbsize, sbsize, sbsize, dftgr, ta, FFTW_PATIENT | FFTW_DESTROY_INPUT);
    fti = fftwf_plan_dft_c2r_3d(tbsize, sbsize, sbsize, ta, dftgr, FFTW_PATIENT | FFTW_DESTROY_INPUT);
  }
  else
  {
    ft = fftwf_plan_dft_r2c_2d(sbsize, sbsize, dftgr, ta, FFTW_PATIENT | FFTW_DESTROY_INPUT);
    fti = fftwf_plan_dft_c2r_2d(sbsize, sbsize, ta, dftgr, FFTW_PATIENT | FFTW_DESTROY_INPUT);
  }
  _aligned_free(ta);
  _aligned_free(dftgr);
  InitializeCriticalSectionAndSpinCount(&csect, 0x80000400);
  tids = (unsigned*)malloc(threads * sizeof(unsigned));
  thds = (HANDLE*)malloc(threads * sizeof(HANDLE));
  pssInfo = (PS_INFO**)malloc(threads * sizeof(PS_INFO*));
  PlanarFrame* pf = tbsize <= 1 ? nlf->ppf : fc->frames[0]->ppf;
  for (int i = 0; i < threads; ++i)
  {
    pssInfo[i] = (PS_INFO*)calloc(1, sizeof(PS_INFO));
    pssInfo[i]->type = (tmode * 4) + (tbsize > 1 ? 2 : 0) + smode;
    pssInfo[i]->zmean = zmean;
    pssInfo[i]->ccnt = ccnt << 1;
    pssInfo[i]->stopz = tbsize - 1;
    pssInfo[i]->barea = barea;
    pssInfo[i]->sbsize = sbsize;
    pssInfo[i]->sosize = sosize;
    pssInfo[i]->csect = &csect;
    pssInfo[i]->nthreads = threads;
    pssInfo[i]->tidx = i;
    pssInfo[i]->cty = i == 0 ? (int*)malloc(threads * 2 * sizeof(int)) :
      pssInfo[0]->cty;
    if (i == 0)
    {
      for (int j = 0; j < threads * 2; ++j)
        pssInfo[0]->cty[j] = INT_MIN;
    }
    pssInfo[i]->hw = hw;
    pssInfo[i]->sigmas = sigmas;
    pssInfo[i]->sigmas2 = sigmas2;
    pssInfo[i]->pmins = pmins;
    pssInfo[i]->pmaxs = pmaxs;
    pssInfo[i]->f0beta = ftype == 0 ? _f0beta : 1.0f;
    pssInfo[i]->ebuff = ebuff;
    pssInfo[i]->dftgc = dftgc;
    pssInfo[i]->ft = ft;
    pssInfo[i]->fti = fti;
    if (tbsize <= 1)
      pssInfo[i]->pf = nlf->ppf;
    else
      pssInfo[i]->pfplut = (const unsigned char**)_aligned_malloc(
        fc->size * sizeof(const unsigned char*), 16);
    pssInfo[i]->fc = fc;
    pssInfo[i]->fftwf_execute_dft_r2c = fftwf_execute_dft_r2c;
    pssInfo[i]->fftwf_execute_dft_c2r = fftwf_execute_dft_c2r;
    pssInfo[i]->dftr = (float*)_aligned_malloc(bvolume * sizeof(float), 16);
    pssInfo[i]->dftc = (fftwf_complex*)_aligned_malloc(sizeof(fftwf_complex) * (ccnt + 11), 16);
    pssInfo[i]->dftc2 = (fftwf_complex*)_aligned_malloc(sizeof(fftwf_complex) * (ccnt + 11), 16);
    for (int b = 0; b < 3; ++b)
    {
      const int height = smode == 0 ? pf->GetHeight(b) / lsb_in_hmul - ((sbsize >> 1) << 1) :
        (pf->GetHeight(b) / lsb_in_hmul - sosize) / (sbsize - sosize);
      const int hslice = height / threads;
      const int hremain = height % threads;
      if (smode == 0)
      {
        pssInfo[i]->sheight[b] = i * hslice;
        pssInfo[i]->eheight[b] =
          ((i + 1) * hslice + (i == threads - 1 ? hremain : 0));
      }
      else
      {
        pssInfo[i]->sheight[b] = i * hslice * (sbsize - sosize);
        pssInfo[i]->eheight[b] =
          ((i + 1) * hslice + (i == threads - 1 ? hremain : 0)) * (sbsize - sosize);
      }
      if (lsb_in_flag)
      {
        pssInfo[i]->ofs_lsb[b] = (pf->GetHeight(b) / lsb_in_hmul) * pf->GetPitch(b);
      }
      else
      {
        pssInfo[i]->ofs_lsb[b] = 0;
      }
    }

#ifdef AVX_BUILD
    if (((env->GetCPUFlags() & CPUF_AVX) && opt == 0) || opt == 3)
    {
      if (!(sbsize & 7))
      {
        pssInfo[i]->proc0 = proc0_SSE2_8;
        pssInfo[i]->proc1 = proc1_SSE_8;
      }
      else if (!(sbsize & 3))
      {
        pssInfo[i]->proc0 = proc0_SSE2_4;
        pssInfo[i]->proc1 = proc1_SSE_4;
      }
      else
      {
        pssInfo[i]->proc0 = proc0_C;
        pssInfo[i]->proc1 = proc1_C;
      }
      pssInfo[i]->removeMean = removeMean_AVX;
      pssInfo[i]->addMean = addMean_AVX;
      if (ftype == 0)
      {
        if (fabsf(_f0beta - 1.0f) < 0.00005f)
          pssInfo[i]->filterCoeffs = filter_0_SSE;
        else if (fabsf(_f0beta - 0.5f) < 0.00005f)
          pssInfo[i]->filterCoeffs = filter_6_SSE;
        else
          pssInfo[i]->filterCoeffs = filter_5_SSE2;
      }
      else if (ftype == 1) pssInfo[i]->filterCoeffs = filter_1_SSE;
      else if (ftype == 2) pssInfo[i]->filterCoeffs = filter_2_SSE;
      else if (ftype == 3) pssInfo[i]->filterCoeffs = filter_3_SSE;
      else pssInfo[i]->filterCoeffs = filter_4_SSE;
    }
    else if (((env->GetCPUFlags() & CPUF_SSE2) && opt == 0) || opt == 2)
#else
    if (((env->GetCPUFlags() & CPUF_SSE2) && opt == 0) || opt == 2)
#endif
    {
      if (!(sbsize & 7))
      {
        pssInfo[i]->proc0 = proc0_SSE2_8;
        pssInfo[i]->proc1 = proc1_SSE_8;
      }
      else if (!(sbsize & 3))
      {
        pssInfo[i]->proc0 = proc0_SSE2_4;
        pssInfo[i]->proc1 = proc1_SSE_4;
      }
      else
      {
        pssInfo[i]->proc0 = proc0_C;
        pssInfo[i]->proc1 = proc1_C;
      }
      pssInfo[i]->removeMean = removeMean_SSE;
      pssInfo[i]->addMean = addMean_SSE;
      if (ftype == 0)
      {
        if (fabsf(_f0beta - 1.0f) < 0.00005f)
          pssInfo[i]->filterCoeffs = filter_0_SSE;
        else if (fabsf(_f0beta - 0.5f) < 0.00005f)
          pssInfo[i]->filterCoeffs = filter_6_SSE;
        else
          pssInfo[i]->filterCoeffs = filter_5_SSE2;
      }
      else if (ftype == 1) pssInfo[i]->filterCoeffs = filter_1_SSE;
      else if (ftype == 2) pssInfo[i]->filterCoeffs = filter_2_SSE;
      else if (ftype == 3) pssInfo[i]->filterCoeffs = filter_3_SSE;
      else pssInfo[i]->filterCoeffs = filter_4_SSE;
    }
    else
    {
      pssInfo[i]->proc0 = proc0_C;
      pssInfo[i]->proc1 = proc1_C;
      pssInfo[i]->removeMean = removeMean_C;
      pssInfo[i]->addMean = addMean_C;
      if (ftype == 0)
      {
        if (fabsf(_f0beta - 1.0f) < 0.00005f)
          pssInfo[i]->filterCoeffs = filter_0_C;
        else if (fabsf(_f0beta - 0.5f) < 0.00005f)
          pssInfo[i]->filterCoeffs = filter_6_C;
        else
          pssInfo[i]->filterCoeffs = filter_5_C;
      }
      else if (ftype == 1) pssInfo[i]->filterCoeffs = filter_1_C;
      else if (ftype == 2) pssInfo[i]->filterCoeffs = filter_2_C;
      else if (ftype == 3) pssInfo[i]->filterCoeffs = filter_3_C;
      else pssInfo[i]->filterCoeffs = filter_4_C;
    }
    if (lsb_in_flag)
    {
      if (((env->GetCPUFlags() & CPUF_SSE2) && opt == 0) || opt >= 2)
      {
        pssInfo[i]->proc0 = proc0_16_SSE2;
      }
      else
      {
        pssInfo[i]->proc0 = proc0_16_C;
      }
    }
    pssInfo[i]->jobFinished = CreateEvent(NULL, TRUE, TRUE, NULL);
    pssInfo[i]->nextJob = CreateEvent(NULL, TRUE, FALSE, NULL);
    thds[i] = (HANDLE)_beginthreadex(0, 0, &threadPool, (void*)(pssInfo[i]), 0, &tids[i]);
  }
  if ((_nfile[0] || _nstring[0]) && ftype < 2)
    getNoiseSpectrum(_nfile, _nstring, sigmas, wscale, env);

  if (lsb_out_flag)
  {
    vi.height = proc_height * 2;
    dstPF_all = new PlanarFrame(vi);
  }
}

char* getTimeString()
{
  time_t ct = time(NULL);
  char* ts = ctime(&ct);
  char* p = ts;
  while (p[0] != '\n' && p[0] != 0)
  {
    if (p[0] == ' ') p[0] = '-';
    else if (p[0] == ':') p[0] = '_';
    ++p;
  }
  p[0] = 0;
  return ts;
}

void dfttest::outputSigmaFile(const char* fname, const float* s,
  const float scale, const bool zmean2)
{
  FILE* f = fopen(fname, "w");
  const int cpx = (sbsize >> 1) + 1;
  float npow = 0.0f;
  for (int z = 0; z < tbsize; ++z)
  {
    for (int k = 0; k < sbsize; ++k)
    {
      for (int j = 0; j < cpx; ++j)
      {
        const int pos = ((z * sbsize + k) * cpx + j) * 2;
        const float val = s[pos] * scale;
        npow += val;
        fprintf(f, "%10.3f ", val);
      }
      fprintf(f, "\n");
    }
    fprintf(f, "\n\n");
  }
  fprintf(f, "# avg power = %f\n", npow / (ccnt - (zmean2 ? 1.0f : 0.0f)));
  fclose(f);
}

struct NPINFO {
  int fn, b, y, x;
};

void dfttest::getNoiseSpectrum(const char* fname, const char* nstring,
  float* dest, const float wscale, IScriptEnvironment* env)
{
  PS_INFO* pss = pssInfo[0];
  PlanarFrame* prf = new PlanarFrame(vi_src);
  memset(dest, 0, ccnt * 2 * sizeof(float));
  float* hw2 = (float*)_aligned_malloc(bvolume * sizeof(float), 16);
  createWindow(hw2, 0, tbsize, tosize, twin, tbeta, 0, sbsize, sosize, swin, sbeta);
  fftwf_complex* dftgc2 = (fftwf_complex*)_aligned_malloc(sizeof(fftwf_complex) * (ccnt + 11), 16);
  float wscale2 = 0.0f, alpha = ftype == 0 ? 5.0f : 7.0f, * dftr = pss->dftr;
  int w = 0;
  for (int s = 0; s < tbsize; ++s)
  {
    for (int i = 0; i < sbsize; ++i)
    {
      for (int k = 0; k < sbsize; ++k, ++w)
      {
        dftr[w] = 255.0f * hw2[w];
        wscale2 += hw2[w] * hw2[w];
      }
    }
  }
  wscale2 = 1.0f / wscale2;
  fftwf_execute_dft_r2c(ftg, dftr, dftgc2);
  int nnpoints = 0;
  char buf[512];
  NPINFO* npts = (NPINFO*)malloc(500 * sizeof(NPINFO));
  if (nstring[0]) // read points from nstring
  {
    const char* q = nstring;
    if (q[0] == 'a' || q[0] == 'A')
    {
      float alphat;
      if (sscanf(q, "%*c:%f", &alphat) != 1)
        env->ThrowError("dfttest:  error reading alpha value from nstring!\n");
      if (alphat <= 0.0f)
        env->ThrowError("dfttest:  nstring - invalid alpha factor!\n");
      alpha = alphat;
      while (q[0] != ' ' && q[0] != 0) ++q;
    }
    while (true)
    {
      while ((q[0] < '0' || q[0] > '9') && q[0] != 0)
        ++q;
      int fn, b, y, x;
      if (q[0] == 0 || sscanf(q, "%d,%d,%d,%d", &fn, &b, &y, &x) != 4)
        break;
      if (fn < 0 || fn > vi_src.num_frames - tbsize)
        env->ThrowError("dfttest:  invalid frame number in nstring (%d)!\n", fn);
      if (b < 0 || b > 2)
        env->ThrowError("dfttest:  invalid plane number in nstring (%d)!\n", b);
      const int height = proc_height >> vi.GetPlaneHeightSubsampling(b);
      if (y < 0 || y > height - sbsize)
        env->ThrowError("dfttest:  invalid y pos in nstring (%d)!\n", y);
      const int width = vi_src.width >> vi.GetPlaneWidthSubsampling(b);
      if (x < 0 || x > width - sbsize)
        env->ThrowError("dfttest:  invalid x pos in nstring (%d)!\n", x);
      if (nnpoints >= 300)
        env->ThrowError("dfttest:  maximum number of entries in nstring is 500!\n");
      npts[nnpoints].fn = fn;
      npts[nnpoints].b = b;
      npts[nnpoints].y = y;
      npts[nnpoints].x = x;
      ++nnpoints;
      while (q[0] != ' ' && q[0] != 0) ++q;
    }
  }
  else // read points from nfile
  {
    FILE* f = fopen(fname, "r");
    if (!f)
      env->ThrowError("dfttest:  unable to open nfile!\n");
    while (fgets(buf, 512, f) != 0)
    {
      int fn, b, y, x;
      if (buf[0] == '#')
        continue;
      if (buf[0] == 'a' || buf[0] == 'A' || sscanf(buf, "%d,%d,%d,%d", &fn, &b, &y, &x) != 4)
      {
        float alphat;
        if ((buf[0] != 'a' && buf[0] != 'A') || sscanf(buf, "%*c=%f", &alphat) != 1)
          continue;
        if (alphat <= 0.0f)
          env->ThrowError("dfttest:  nfile - invalid alpha factor!\n");
        alpha = alphat;
        continue;
      }
      if (fn < 0 || fn > vi_src.num_frames - tbsize)
        env->ThrowError("dfttest:  invalid frame number in nfile (%d)!\n", fn);
      if (b < 0 || b > 2)
        env->ThrowError("dfttest:  invalid plane number in nfile (%d)!\n", b);
      const int height = proc_height >> vi.GetPlaneHeightSubsampling(b);
      if (y < 0 || y > height - sbsize)
        env->ThrowError("dfttest:  invalid y pos in nfile (%d)!\n", y);
      const int width = vi_src.width >> vi.GetPlaneWidthSubsampling(b);
      if (x < 0 || x > width - sbsize)
        env->ThrowError("dfttest:  invalid x pos in nfile (%d)!\n", x);
      if (nnpoints >= 300)
        env->ThrowError("dfttest:  maximum number of entries in nfile is 500!\n");
      npts[nnpoints].fn = fn;
      npts[nnpoints].b = b;
      npts[nnpoints].y = y;
      npts[nnpoints].x = x;
      ++nnpoints;
    }
    fclose(f);
  }
  sprintf(buf, "dfttest:  alpha = %f  nnpoints = %d\n", alpha, nnpoints);
  OutputDebugString(buf);
  for (int ct = 0; ct < nnpoints; ++ct)
  {
    float* dftc = (float*)pss->dftc;
    for (int z = 0; z < tbsize; ++z)
    {
      PVideoFrame src = child->GetFrame(npts[ct].fn + z, env);
      prf->copyFrom(src, vi_src);
      const int pitch = prf->GetPitch(npts[ct].b);
      const unsigned char* srcp = prf->GetPtr(npts[ct].b) + npts[ct].y * pitch + npts[ct].x;
      const int offset_lsb = pss->ofs_lsb[npts[ct].b];
      pss->proc0(srcp, hw2 + sbsize * sbsize * z, dftr + sbsize * sbsize * z, pitch, sbsize, offset_lsb);
    }
    fftwf_execute_dft_r2c(ft, dftr, pss->dftc);
    if (zmean)
      pss->removeMean(dftc, (float*)dftgc2, ccnt * 2, (float*)pss->dftc2);
    for (int h = 0; h < ccnt * 2; h += 2)
    {
      const float psd = dftc[h + 0] * dftc[h + 0] + dftc[h + 1] * dftc[h + 1];
      dest[h + 0] += psd;
      dest[h + 1] += psd;
    }
  }
  free(npts);
  delete prf;
  _aligned_free(hw2);
  _aligned_free(dftgc2);
  if (nnpoints != 0)
  {
    const float scale = 1.0f / (float)nnpoints;
    for (int h = 0; h < ccnt * 2; ++h)
      dest[h] = dest[h] * scale * (wscale2 / wscale) * alpha;
    sprintf(buf, "noise_spectrum-%s.txt", getTimeString());
    outputSigmaFile(buf, dest, wscale / alpha, zmean);
  }
  else
    env->ThrowError("dfttest:  no noise blocks in nfile or nstring!\n");
}

float* parseString(const char* s, int& poscnt, const float sigma,
  const float pfact, IScriptEnvironment* env)
{
  float* parray;
  if (s[0] == 0)
  {
    parray = (float*)malloc(4 * sizeof(float));
    parray[0] = 0.0f; parray[2] = 1.0f;
    parray[1] = parray[3] = powf(sigma, pfact);
    poscnt = 2;
  }
  else
  {
    poscnt = 0;
    bool found[2] = { false, false };
    const char* sT = s;
    while (sT[0] != 0)
    {
      float pos, sval;
      if (sscanf(sT, "%f:%f", &pos, &sval) != 2)
        env->ThrowError("dfttest:  invalid entry in sigma string!\n");
      if (pos < 0.0f || pos > 1.0f)
        env->ThrowError("dfttest:  sigma string - invalid pos (%f)!\n", pos);
      if (pos == 0.0f)
        found[0] = true;
      else if (pos == 1.0f)
        found[1] = true;
      ++poscnt;
      while (sT[1] != 0 && sT[1] != ' ') ++sT;
      ++sT;
    }
    if (!found[0] || !found[1])
      env->ThrowError("dfttest:  sigma string - one or more end points not provided!\n");
    parray = (float*)malloc(poscnt * 2 * sizeof(float));
    sT = s;
    poscnt = 0;
    while (sT[0] != 0)
    {
      sscanf(sT, "%f:%f", &parray[poscnt * 2 + 0], &parray[poscnt * 2 + 1]);
      parray[poscnt * 2 + 1] = powf(parray[poscnt * 2 + 1], pfact);
      ++poscnt;
      while (sT[1] != 0 && sT[1] != ' ') ++sT;
      ++sT;
    }
    for (int i = 1; i < poscnt; ++i)
    {
      int j = i;
      const float t0 = parray[j * 2 + 0];
      const float t1 = parray[j * 2 + 1];
      while (j > 0 && parray[(j - 1) * 2 + 0] > t0)
      {
        parray[j * 2 + 0] = parray[(j - 1) * 2 + 0];
        parray[j * 2 + 1] = parray[(j - 1) * 2 + 1];
        --j;
      }
      parray[j * 2 + 0] = t0;
      parray[j * 2 + 1] = t1;
    }
  }
  return parray;
}

float interp(const float pf, const float* pv, const int cnt)
{
  int lidx = 0;
  for (int i = cnt - 1; i >= 0; --i)
  {
    if (pv[i * 2 + 0] <= pf)
    {
      lidx = i;
      break;
    }
  }
  int hidx = cnt - 1;
  for (int i = 0; i < cnt; ++i)
  {
    if (pv[i * 2 + 0] >= pf)
    {
      hidx = i;
      break;
    }
  }
  const float d0 = pf - pv[lidx * 2 + 0];
  const float d1 = pv[hidx * 2 + 0] - pf;
  if (hidx == lidx || d0 <= 0.0f)
    return pv[lidx * 2 + 1];
  if (d1 <= 0.0f)
    return pv[hidx * 2 + 1];
  const float tf = d0 / (d0 + d1);
  return pv[lidx * 2 + 1] * (1.0f - tf) + pv[hidx * 2 + 1] * tf;
}

float getSVal(const int pos, const int len, const float* pv,
  const int cnt, float& pf)
{
  if (len == 1)
  {
    pf = 0.0f;
    return 1.0f;
  }
  const int ld2 = len >> 1;
  if (pos > ld2)
    pf = (len - pos) / (float)ld2;
  else
    pf = pos / (float)ld2;
  return interp(pf, pv, cnt);
}

void dfttest::sigmaFromString(float* dest, const char* sstring, const char* ssx,
  const char* ssy, const char* sst, const float wscale, IScriptEnvironment* env)
{
  int ndim = 3;
  if (tbsize == 1) ndim -= 1;
  if (sbsize == 1) ndim -= 2;
  const float ndiv = 1.0f / (float)ndim;
  int tcnt = 0, sycnt = 0, sxcnt = 0;
  float* tdata, * sydata, * sxdata;
  bool edis = false;
  if (sstring[0])
  {
    const char* w = sstring;
    if (sstring[0] == '$') // fft3dfilter method
    {
      edis = true;
      while ((w[0] == '$' || w[0] == ' ') && w[0] != 0)
        ++w;
    }
    tdata = parseString(w, tcnt, sigma, edis ? 1.0f : ndiv, env);
    sydata = parseString(w, sycnt, sigma, edis ? 1.0f : ndiv, env);
    sxdata = parseString(w, sxcnt, sigma, edis ? 1.0f : ndiv, env);
  }
  else
  {
    tdata = parseString(sst, tcnt, sigma, ndiv, env);
    sydata = parseString(ssy, sycnt, sigma, ndiv, env);
    sxdata = parseString(ssx, sxcnt, sigma, ndiv, env);
  }
  const int cpx = ((sbsize >> 1) + 1);
  float pft, pfy, pfx;
  for (int z = 0; z < tbsize; ++z)
  {
    const float tval = getSVal(z, tbsize, tdata, tcnt, pft);
    for (int y = 0; y < sbsize; ++y)
    {
      const float syval = getSVal(y, sbsize, sydata, sycnt, pfy);
      for (int x = 0; x < cpx; ++x)
      {
        const float sxval = getSVal(x, sbsize, sxdata, sxcnt, pfx);
        float val;
        if (edis)
        {
          const float dw = sqrtf((pft * pft + pfy * pfy + pfx * pfx) / (float)ndim);
          val = interp(dw, tdata, tcnt);
        }
        else
          val = tval * syval * sxval;
        const int pos = ((z * sbsize + y) * cpx + x) * 2;
        dest[pos + 0] = dest[pos + 1] = val / wscale;
      }
    }
  }
  free(tdata);
  free(sydata);
  free(sxdata);
  if (!quiet_flag)
  {
    char buf[512];
    sprintf(buf, "filter_spectrum-%s.txt", getTimeString());
    outputSigmaFile(buf, dest, wscale, false);
  }
}

void dfttest::loadFile(float* dest, const char* src, const float wscale, IScriptEnvironment* env)
{
  FILE* f = fopen(src, "r");
  if (!f)
    env->ThrowError("dfttest:  unable to open (%s)!\n", src);
  int c = 0;
  char buf[4096];
  while (fgets(buf, 4096, f) != 0)
  {
    if (buf[0] == '#')
      continue;
    float temp;
    char* p = buf;
    while ((p[0] < '0' || p[0] > '9') && p[0] != '.' && p[0] != 0)
      ++p;
    while (p[0] != 0 && sscanf(p, "%f", &temp) == 1)
    {
      dest[c * 2] = dest[c * 2 + 1] = temp / wscale;
      ++c;
      while ((p[0] >= '0' && p[0] <= '9') || p[0] == '.')
        ++p;
      while ((p[0] == ',' || p[0] == ' ' || p[0] == '\t') && p[0] != 0)
        ++p;
    }
  }
  if (c != ccnt)
    env->ThrowError("dfttest:  (%s) has incorrect number of values (%d vs %d)!",
      src, c, ccnt);
}

int num_processors()
{
  int pcount = 0;
  DWORD_PTR p_aff, s_aff;
  GetProcessAffinityMask(GetCurrentProcess(), &p_aff, &s_aff);
  for (; p_aff != 0; p_aff >>= 1)
    pcount += (p_aff & 1);
  return pcount;
}

double besselI0(double p)
{
  p = p / 2;
  double n = 1, t = 1, d = 1;
  int k = 1;
  double v;
  do
  {
    n = n * p;
    d = d * k;
    v = n / d;
    t = t + v * v;
  } while (++k < 15 && v >(1.0E-8));
  return t;
}

double getWinValue(double n, double size, int win, double beta)
{
  switch (win)
  {
  case 0: // hanning
    return (0.50 - 0.50 * cos(2.0 * M_PI * n / double(size)));
  case 1: // hamming
    return (0.53836 - 0.46164 * cos(2.0 * M_PI * n / double(size)));
  case 2: // blackman
    return (0.42 - 0.50 * cos(2.0 * M_PI * n / double(size)) + 0.08 * cos(4.0 * M_PI * n / double(size)));
  case 3: // 4 term blackman-harris
    return (0.35875 - 0.48829 * cos(2.0 * M_PI * n / double(size)) +
      0.14128 * cos(4.0 * M_PI * n / double(size)) - 0.01168 * cos(6.0 * M_PI * n / double(size)));
  case 4: // kaiser-bessel
  {
    const double v = ((2.0 * n) / double(size)) - 1.0;
    return (besselI0(M_PI * beta * sqrt(1.0 - v * v)) / besselI0(M_PI * beta));
  }
  case 5: // 7 term blackman-harris
    return (0.27105140069342415 -
      0.433297939234486060 * cos(2.0 * M_PI * n / double(size)) +
      0.218122999543110620 * cos(4.0 * M_PI * n / double(size)) -
      0.065925446388030898 * cos(6.0 * M_PI * n / double(size)) +
      0.010811742098372268 * cos(8.0 * M_PI * n / double(size)) -
      7.7658482522509342E-4 * cos(10.0 * M_PI * n / double(size)) +
      1.3887217350903198E-5 * cos(12.0 * M_PI * n / double(size)));
  case 6: // flat top
    return (0.2810639 -
      0.5208972 * cos(2.0 * M_PI * n / double(size)) +
      0.1980399 * cos(4.0 * M_PI * n / double(size)));
  case 7: // rectangular
    return 1.0;
  case 8: // Bartlett
    return ((2.0 / double(size)) * ((double(size) / 2.0) - abs(n - (double(size) / 2.0))));
  case 9: // Bartlett-Hann
    return (0.62 - 0.48 * (n / double(size) - 0.5) - 0.38 * cos(2.0 * M_PI * n / double(size)));
  case 10: // Nuttall
    return (0.355768 -
      0.487396 * cos(2.0 * M_PI * n / double(size)) +
      0.144232 * cos(4.0 * M_PI * n / double(size)) -
      0.012604 * cos(6.0 * M_PI * n / double(size)));
  case 11: // Blackman-Nuttall
    return (0.3635819 -
      0.4891775 * cos(2.0 * M_PI * n / double(size)) +
      0.1365995 * cos(4.0 * M_PI * n / double(size)) -
      0.0106411 * cos(6.0 * M_PI * n / double(size)));
  }
  return 0.0;
}

void normalizeForOverlapAdd(double* hw, const int bsize, const int osize)
{
  double* nw = (double*)calloc(bsize, sizeof(double));
  const int inc = bsize - osize;
  for (int q = 0; q < bsize; ++q)
  {
    for (int h = q; h >= 0; h -= inc)
      nw[q] += hw[h] * hw[h];
    for (int h = q + inc; h < bsize; h += inc)
      nw[q] += hw[h] * hw[h];
  }
  for (int q = 0; q < bsize; ++q)
    hw[q] /= sqrt(nw[q]);
  free(nw);
}

void createWindow(float* hw, const int tmode, const int tbsize,
  const int tosize, const int twin, const double tbeta, const int smode,
  const int sbsize, const int sosize, const int swin, const double sbeta)
{
  double* tw = (double*)malloc(tbsize * sizeof(double));
  for (int j = 0; j < tbsize; ++j)
    tw[j] = getWinValue(j + 0.5, tbsize, twin, tbeta);
  if (tmode == 1)
    normalizeForOverlapAdd(tw, tbsize, tosize);
  double* sw = (double*)malloc(sbsize * sizeof(double));
  for (int j = 0; j < sbsize; ++j)
    sw[j] = getWinValue(j + 0.5, sbsize, swin, sbeta);
  if (smode == 1)
    normalizeForOverlapAdd(sw, sbsize, sosize);
  const double nscale = 1.0 / sqrt((double)(tbsize * sbsize * sbsize));
  for (int j = 0; j < tbsize; ++j)
    for (int k = 0; k < sbsize; ++k)
      for (int q = 0; q < sbsize; ++q)
        hw[(j * sbsize + k) * sbsize + q] = (float)(tw[j] * sw[k] * sw[q] * nscale);
  free(tw);
  free(sw);
}

dfttest::~dfttest()
{
  for (int i = 0; i < threads; ++i)
  {
    pssInfo[i]->type = -1;
    SetEvent(pssInfo[i]->nextJob);
  }
  WaitForMultipleObjects(threads, thds, TRUE, INFINITE);
  for (int i = 0; i < threads; ++i)
    CloseHandle(thds[i]);
  free(tids);
  free(thds);
  for (int i = 0; i < threads; ++i)
  {
    CloseHandle(pssInfo[i]->jobFinished);
    CloseHandle(pssInfo[i]->nextJob);
    _aligned_free(pssInfo[i]->dftr);
    _aligned_free(pssInfo[i]->dftc);
    _aligned_free(pssInfo[i]->dftc2);
    if (tbsize > 1)
      _aligned_free(pssInfo[i]->pfplut);
    if (i == 0)
      free(pssInfo[i]->cty);
    free(pssInfo[i]);
  }
  free(pssInfo);
  if (ft) fftwf_destroy_plan(ft);
  if (fti) fftwf_destroy_plan(fti);
  if (ftg) fftwf_destroy_plan(ftg);
  if (dftgc) _aligned_free(dftgc);
  if (dstPF) delete dstPF;
  if (dstPF_lsb) delete dstPF_lsb;
  if (dstPF_all) delete dstPF_all;
  if (hw) _aligned_free(hw);
  if (ebuff)
  {
    const int ebcount = (tbsize > 1 && tmode == 1) ? tbsize : 1;
    for (int i = 0; i < ebcount * 3; ++i)
    {
      if (ebuff[i])
        _aligned_free(ebuff[i]);
    }
    free(ebuff);
  }
  if (fc) delete fc;
  if (nlf) delete nlf;
  if (sigmas) _aligned_free(sigmas);
  if (sigmas2) _aligned_free(sigmas2);
  if (pmins) _aligned_free(pmins);
  if (pmaxs) _aligned_free(pmaxs);
  if (hLib) FreeLibrary(hLib);
  DeleteCriticalSection(&csect);
}

AVSValue __cdecl Create_dfttest(AVSValue args, void* user_data, IScriptEnvironment* env)
{
  return new dfttest(args[0].AsClip(), args[1].AsBool(true), args[2].AsBool(true),
    args[3].AsBool(true), args[4].AsInt(0), float(args[5].AsFloat(16.0)), float(args[6].AsFloat(16.0)),
    float(args[7].AsFloat(0.0f)), float(args[8].AsFloat(500.0f)), args[9].AsInt(12), args[10].AsInt(1),
    args[11].AsInt(9), args[12].AsInt(5), args[13].AsInt(0), args[14].AsInt(0),
    args[15].AsInt(0), args[16].AsInt(7), args[17].AsFloat(2.5), args[18].AsFloat(2.5),
    args[19].AsBool(true), args[20].AsString(""), args[21].AsString(""),
    args[22].AsString(""), args[23].AsString(""), float(args[24].AsFloat(1.0f)),
    args[25].AsString(""), args[26].AsInt(0), args[27].AsInt(0),
    args[28].AsString(""), args[29].AsString(""), args[30].AsString(""),
    args[31].AsString(""), args[32].AsString(""), args[33].AsInt(0),
    args[35].AsBool(false), args[34].AsBool(false), args[36].AsBool(true),
    env);
}

const AVS_Linkage* AVS_linkage = nullptr;

extern "C" __declspec(dllexport) const char* __stdcall AvisynthPluginInit3(IScriptEnvironment * env, const AVS_Linkage* const vectors)
{
  AVS_linkage = vectors;

  env->AddFunction("dfttest", "c[Y]b[U]b[V]b[ftype]i[sigma]f[sigma2]f[pmin]f" \
    "[pmax]f[sbsize]i[smode]i[sosize]i[tbsize]i[tmode]i[tosize]i[swin]i" \
    "[twin]i[sbeta]f[tbeta]f[zmean]b[sfile]s[sfile2]s[pminfile]s[pmaxfile]s" \
    "[f0beta]f[nfile]s[threads]i[opt]i[nstring]s[sstring]s[ssx]s[ssy]s[sst]s" \
    "[dither]i[lsb]b[lsb_in]b[quiet]b", Create_dfttest, 0);
  return "DFTTest for Avisynth+";
}