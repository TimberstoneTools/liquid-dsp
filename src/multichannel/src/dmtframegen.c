/*
 * Copyright (c) 2007 - 2015 Joseph Gaeddert
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

//
// dmtframegen.c
//
// DMT frame generator
//

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "liquid.internal.h"

#if HAVE_FFTW3_H
#   include <fftw3.h>
#endif

#define DEBUG_DMTFRAMEGEN            1

struct dmtframegen_s {
    unsigned int M;         // number of subcarriers
    unsigned int cp_len;    // cyclic prefix length
    unsigned char * p;      // subcarrier allocation (null, pilot, data)

    // tapering/trasition
    unsigned int taper_len; // number of samples in tapering window/overlap
    float * taper;          // tapering window
    float complex *postfix; // overlapping symbol buffer

    // constants
    unsigned int M_null;    // number of null subcarriers
    unsigned int M_pilot;   // number of pilot subcarriers
    unsigned int M_data;    // number of data subcarriers
    unsigned int M_S0;      // number of enabled subcarriers in S0
    unsigned int M_S1;      // number of enabled subcarriers in S1

    // scaling factors
    float g_data;           //

    // transform object
    FFT_PLAN ifft;          // ifft object
    float complex * X;      // frequency-domain buffer
    float complex * x;      // time-domain buffer

    // PLCP short
    float complex * s0;     // short sequence (time)

    // PLCP long
    float complex * s1;     // long sequence (time)

    // pilot sequence
    msequence ms_pilot;
};

// create DMT framing generator object
//  _M          :   number of subcarriers, >10 typical
//  _cp_len     :   cyclic prefix length
//  _taper_len  :   taper length (DMT symbol overlap)
//  _p          :   subcarrier allocation (null, pilot, data), [size: _M x 1]
dmtframegen dmtframegen_create(unsigned int    _M,
                                 unsigned int    _cp_len,
                                 unsigned int    _taper_len,
                                 unsigned char * _p)
{
    // validate input
    if (_M < 2) {
        fprintf(stderr,"error: dmtframegen_create(), number of subcarriers must be at least 2\n");
        exit(1);
    } else if (_M % 2) {
        fprintf(stderr,"error: dmtframegen_create(), number of subcarriers must be even\n");
        exit(1);
    } else if (_cp_len > _M) {
        fprintf(stderr,"error: dmtframegen_create(), cyclic prefix cannot exceed symbol length\n");
        exit(1);
    } else if (_taper_len > _cp_len) {
        fprintf(stderr,"error: dmtframegen_create(), taper length cannot exceed cyclic prefix\n");
        exit(1);
    }

    dmtframegen q = (dmtframegen) malloc(sizeof(struct dmtframegen_s));
    q->M         = _M;
    q->cp_len    = _cp_len;
    q->taper_len = _taper_len;

    // allocate memory for subcarrier allocation IDs
    q->p = (unsigned char*) malloc((q->M)*sizeof(unsigned char));
    if (_p == NULL) {
        // initialize default subcarrier allocation
        dmtframe_init_default_sctype(q->M, q->p);
    } else {
        // copy user-defined subcarrier allocation
        memcpy(q->p, _p, q->M*sizeof(unsigned char));
    }

    // validate and count subcarrier allocation
    dmtframe_validate_sctype(q->p, q->M, &q->M_null, &q->M_pilot, &q->M_data);
    if ( (q->M_pilot + q->M_data) == 0) {
        fprintf(stderr,"error: dmtframegen_create(), must have at least one enabled subcarrier\n");
        exit(1);
    } else if (q->M_data == 0) {
        fprintf(stderr,"error: dmtframegen_create(), must have at least one data subcarriers\n");
        exit(1);
    } else if (q->M_pilot < 2) {
        fprintf(stderr,"error: dmtframegen_create(), must have at least two pilot subcarriers\n");
        exit(1);
    }

    unsigned int i;

    // allocate memory for transform objects
    q->X = (float complex*) malloc((q->M)*sizeof(float complex));
    q->x = (float complex*) malloc((q->M)*sizeof(float complex));
    q->ifft = FFT_CREATE_PLAN(q->M, q->X, q->x, FFT_DIR_BACKWARD, FFT_METHOD);

    // allocate memory for PLCP arrays
    float complex *S0 = (float complex*) malloc((q->M)*sizeof(float complex));
    q->s0 = (float complex*) malloc((q->M)*sizeof(float complex));
    dmtframe_init_S0(q->p, q->M, S0, q->s0, &q->M_S0);
    free(S0);

    float complex *S1 = (float complex*) malloc((q->M)*sizeof(float complex));
    q->s1 = (float complex*) malloc((q->M)*sizeof(float complex));
    dmtframe_init_S1(q->p, q->M, S1, q->s1, &q->M_S1);
    free(S1);

    // create tapering window and transition buffer
    q->taper   = (float*)         malloc(q->taper_len * sizeof(float));
    q->postfix = (float complex*) malloc(q->taper_len * sizeof(float complex));
    for (i=0; i<q->taper_len; i++) {
        float t = ((float)i + 0.5f) / (float)(q->taper_len);
        float g = sinf(M_PI_2*t);
        q->taper[i] = g*g;
    }
#if 0
    // validate window symmetry
    for (i=0; i<q->taper_len; i++) {
        printf("    taper[%2u] = %12.8f (%12.8f)\n", i, q->taper[i],
            q->taper[i] + q->taper[q->taper_len - i - 1]);
    }
#endif

    // compute scaling factor
    q->g_data = 1.0f / sqrtf(q->M_pilot + q->M_data);

    // set pilot sequence
    q->ms_pilot = msequence_create_default(8);

    return q;
}

// free transform object memory
void dmtframegen_destroy(dmtframegen _q)
{
    // free subcarrier type array memory
    free(_q->p);

    // free transform array memory
    free(_q->X);
    free(_q->x);
    FFT_DESTROY_PLAN(_q->ifft);

    // free tapering window and transition buffer
    free(_q->taper);
    free(_q->postfix);

    // free PLCP memory arrays
    free(_q->s0);
    free(_q->s1);

    // free pilot msequence object memory
    msequence_destroy(_q->ms_pilot);

    // free main object memory
    free(_q);
}

void dmtframegen_print(dmtframegen _q)
{
    printf("dmtframegen:\n");
    printf("    num subcarriers     :   %-u\n", _q->M);
    printf("      - NULL            :   %-u\n", _q->M_null);
    printf("      - pilot           :   %-u\n", _q->M_pilot);
    printf("      - data            :   %-u\n", _q->M_data);
    printf("    cyclic prefix len   :   %-u\n", _q->cp_len);
    printf("    taper len           :   %-u\n", _q->taper_len);
    printf("    ");
    dmtframe_print_sctype(_q->p, _q->M);
}

void dmtframegen_reset(dmtframegen _q)
{
    msequence_reset(_q->ms_pilot);

    // clear internal postfix buffer
    unsigned int i;
    for (i=0; i<_q->taper_len; i++)
        _q->postfix[i] = 0.0f;
}

// write first PLCP short sequence 'symbol' to buffer
//
//  |<- 2*cp->|<-       M       ->|<-       M       ->|
//  |         |                   |                   |
//      +-----+-------------------+-------------------+
//     /      |                   |                   |
//    /  ..s0 |        s0         |        s0         |
//   /        |                   |                   |
//  +---------+-------------------+-------------------+-----> time
//  |                        |                        |
//  |<-        s0[a]       ->|<-        s0[b]       ->|
//  |        M + cp_len      |        M + cp_len      |
//
void dmtframegen_write_S0a(dmtframegen    _q,
                            float complex * _y)
{
    unsigned int i;
    unsigned int k;
    for (i=0; i<_q->M + _q->cp_len; i++) {
        k = (i + _q->M - 2*_q->cp_len) % _q->M;
        _y[i] = _q->s0[k];
    }

    // apply tapering window
    for (i=0; i<_q->taper_len; i++)
        _y[i] *= _q->taper[i];
}

void dmtframegen_write_S0b(dmtframegen _q,
                            float complex * _y)
{
    unsigned int i;
    unsigned int k;
    for (i=0; i<_q->M + _q->cp_len; i++) {
        k = (i + _q->M - _q->cp_len) % _q->M;
        _y[i] = _q->s0[k];
    }

    // copy postfix (first 'taper_len' samples of s0 symbol)
    memcpy(_q->postfix, _q->s0, _q->taper_len*sizeof(float complex));
}

void dmtframegen_write_S1(dmtframegen _q,
                           float complex * _y)
{
    // copy S1 symbol to output, adding cyclic prefix and tapering window
    memcpy(_q->x, _q->s1, (_q->M)*sizeof(float complex));
    dmtframegen_gensymbol(_q, _y);
}


// write DMT symbol
//  _q      :   framing generator object
//  _x      :   input symbols, [size: _M x 1]
//  _y      :   output samples, [size: _M x 1]
void dmtframegen_writesymbol(dmtframegen    _q,
                              float complex * _x,
                              float complex * _y)
{
    // move frequency data to internal buffer
    unsigned int i;
    unsigned int k;
    int sctype;
    for (i=0; i<_q->M; i++) {
        // start at mid-point (effective fftshift)
        k = (i + _q->M/2) % _q->M;

        sctype = _q->p[k];
        if (sctype==DMTFRAME_SCTYPE_NULL) {
            // disabled subcarrier
            _q->X[k] = 0.0f;
        } else if (sctype==DMTFRAME_SCTYPE_PILOT) {
            // pilot subcarrier
            _q->X[k] = (msequence_advance(_q->ms_pilot) ? 1.0f : -1.0f) * _q->g_data;
        } else {
            // data subcarrier
            _q->X[k] = _x[k] * _q->g_data;
        }

        //printf("X[%3u] = %12.8f + j*%12.8f;\n",i+1,crealf(_q->X[i]),cimagf(_q->X[i]));
    }

    // execute transform
    FFT_EXECUTE(_q->ifft);

    // copy result to output, adding cyclic prefix and tapering window
    dmtframegen_gensymbol(_q, _y);
}

// write tail to output
void dmtframegen_writetail(dmtframegen    _q,
                            float complex * _buffer)
{
    // write tail to output, applying tapering window
    unsigned int i;
    for (i=0; i<_q->taper_len; i++)
        _buffer[i] = _q->postfix[i] * _q->taper[_q->taper_len-i-1];
}

// 
// internal methods
//

// generate symbol (add cyclic prefix/postfix, overlap)
//
//  ->|   |<- taper_len
//    +   +-----+-------------------+
//     \ /      |                   |
//      X       |      symbol       |
//     / \      |                   |
//    +---+-----+-------------------+----> time
//    |         |                   |
//    |<- cp  ->|<-       M       ->|
//
//  _q->x           :   input time-domain symbol [size: _q->M x 1]
//  _q->postfix     :   input:  post-fix from previous symbol [size: _q->taper_len x 1]
//                      output: post-fix from this new symbol
//  _q->taper       :   tapering window
//  _q->taper_len   :   tapering window length
//
//  _buffer         :   output sample buffer [size: (_q->M + _q->cp_len) x 1]
void dmtframegen_gensymbol(dmtframegen    _q,
                            float complex * _buffer)
{
    // copy input symbol with cyclic prefix to output symbol
    memcpy( &_buffer[0],          &_q->x[_q->M-_q->cp_len], _q->cp_len*sizeof(float complex));
    memcpy( &_buffer[_q->cp_len], &_q->x[               0], _q->M    * sizeof(float complex));
    
    // apply tapering window to over-lapping regions
    unsigned int i;
    for (i=0; i<_q->taper_len; i++) {
        _buffer[i] *= _q->taper[i];
        _buffer[i] += _q->postfix[i] * _q->taper[_q->taper_len-i-1];
    }

    // copy post-fix to output (first 'taper_len' samples of input symbol)
    memcpy(_q->postfix, _q->x, _q->taper_len*sizeof(float complex));
}

