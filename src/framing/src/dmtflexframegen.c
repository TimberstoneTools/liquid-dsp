/*
  Copyright (c) 2007 - 2015 Joseph Gaeddert
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
// dmtflexframegen.c
//
// DMT flexible frame generator
//

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "liquid.internal.h"

#define DEBUG_DMTFLEXFRAMEGEN               0
#define DMTFLEXFRAME_NUM_S0                 2
#define DMTFLEXFRAME_NUM_S1                 1

// reconfigure internal buffers, objects, etc.
void dmtflexframegen_reconfigure(dmtflexframegen _q);

// encode header
void dmtflexframegen_encode_header(dmtflexframegen _q);

// write first S0 symbol
void dmtflexframegen_write_S0a(dmtflexframegen _q,
                                float complex * _buffer);

// write second S0 symbol
void dmtflexframegen_write_S0b(dmtflexframegen _q,
                                float complex * _buffer);

// write S1 symbol
void dmtflexframegen_write_S1(dmtflexframegen _q,
                               float complex * _buffer);

// write header symbol
void dmtflexframegen_write_header(dmtflexframegen _q,
                                   float complex * _buffer);

// write payload symbol
void dmtflexframegen_write_payload(dmtflexframegen _q,
                                    float complex * _buffer);

// default dmtflexframegen properties
static dmtflexframegenprops_s dmtflexframegenprops_default = {
    LIQUID_CRC_32,      // check
    LIQUID_FEC_NONE,    // fec0
    LIQUID_FEC_NONE,    // fec1
    LIQUID_MODEM_QPSK,  // mod_scheme
    //64                // block_size
};

void dmtflexframegenprops_init_default(dmtflexframegenprops_s * _props)
{
    memcpy(_props, &dmtflexframegenprops_default, sizeof(dmtflexframegenprops_s));
}

struct dmtflexframegen_s {
    unsigned int M;         // number of subcarriers
    unsigned int cp_len;    // cyclic prefix length
    unsigned int taper_len; // taper length
    unsigned char * p;      // subcarrier allocation (null, pilot, data)

    // constants
    unsigned int M_null;    // number of null subcarriers
    unsigned int M_pilot;   // number of pilot subcarriers
    unsigned int M_data;    // number of data subcarriers
    unsigned int M_S0;      // number of enabled subcarriers in S0
    unsigned int M_S1;      // number of enabled subcarriers in S1

    // buffers
    float complex * X;      // frequency-domain buffer

    // internal low-level objects
    dmtframegen fg;         // frame generator object

    // header
    modem mod_header;                   // header modulator
    packetizer p_header;                    // header packetizer
    unsigned char header[DMTFLEXFRAME_H_DEC];      // header data (uncoded)
    unsigned char header_enc[DMTFLEXFRAME_H_ENC];  // header data (encoded)

    // payload
    packetizer p_payload;                   // payload packetizer
    unsigned int payload_dec_len;       // payload length (num un-encoded bytes)
    modem mod_payload;                  // payload modulator
    unsigned char * payload_enc;        // payload data (encoded bytes)
    unsigned int payload_enc_len;       // length of encoded payload

    // counters/states
    enum {
        DMTFLEXFRAMEGEN_STATE_S0a=0,    // write S0 symbol (first)
        DMTFLEXFRAMEGEN_STATE_S0b,      // write S0 symbol (second)
        DMTFLEXFRAMEGEN_STATE_S1,       // write S1 symbol
        DMTFLEXFRAMEGEN_STATE_HEADER,   // write header symbols
        DMTFLEXFRAMEGEN_STATE_PAYLOAD   // write payload symbols
    } state;
    int frame_assembled;                // frame assembled flag
    int frame_complete;                 // frame completed flag
    unsigned int header_bit_index;      //
    unsigned int payload_bit_index;     //

    // properties
    dmtflexframegenprops_s props;
};

// create DMT flexible framing generator object
//  _M          :   number of subcarriers, >10 typical
//  _cp_len     :   cyclic prefix length
//  _taper_len  :   taper length (DMT symbol overlap)
//  _p          :   subcarrier allocation (null, pilot, data), [size: _M x 1]
//  _fgprops    :   frame properties (modulation scheme, etc.)
dmtflexframegen dmtflexframegen_create(unsigned int              _M,
                                         unsigned int              _cp_len,
                                         unsigned int              _taper_len,
                                         unsigned char *           _p,
                                         dmtflexframegenprops_s * _fgprops)
{
    // validate input
    if (_M < 2) {
        fprintf(stderr,"error: dmtflexframegen_create(), number of subcarriers must be at least 2\n");
        exit(1);
    } else if (_M % 2) {
        fprintf(stderr,"error: dmtflexframegen_create(), number of subcarriers must be even\n");
        exit(1);
    }

    dmtflexframegen q = (dmtflexframegen) malloc(sizeof(struct dmtflexframegen_s));
    q->M         = _M;          // number of subcarriers
    q->cp_len    = _cp_len;     // cyclic prefix length
    q->taper_len = _taper_len;  // taper length

    // allocate memory for transform buffers
    q->X = (float complex*) malloc((q->M)*sizeof(float complex));

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

    // create internal DMT frame generator object
    q->fg = dmtframegen_create(q->M, q->cp_len, q->taper_len, q->p);

    // create header objects
    q->mod_header = modem_create(DMTFLEXFRAME_H_MOD);
    q->p_header   = packetizer_create(DMTFLEXFRAME_H_DEC,
                                      DMTFLEXFRAME_H_CRC,
                                      DMTFLEXFRAME_H_FEC,
                                      LIQUID_FEC_NONE);
    assert(packetizer_get_enc_msg_len(q->p_header)==DMTFLEXFRAME_H_ENC);

    // initial memory allocation for payload
    q->payload_dec_len = 1;
    q->p_payload = packetizer_create(q->payload_dec_len,
                                     LIQUID_CRC_NONE,
                                     LIQUID_FEC_NONE,
                                     LIQUID_FEC_NONE);
    q->payload_enc_len = packetizer_get_enc_msg_len(q->p_payload);
    q->payload_enc = (unsigned char*) malloc(q->payload_enc_len*sizeof(unsigned char));

    // create payload modem (initially QPSK, overridden by properties)
    q->mod_payload = modem_create(LIQUID_MODEM_QPSK);

    // initialize properties
    dmtflexframegen_setprops(q, _fgprops);

    // reset
    dmtflexframegen_reset(q);

    // return pointer to main object
    return q;
}

void dmtflexframegen_destroy(dmtflexframegen _q)
{
    // destroy internal objects
    dmtframegen_destroy(_q->fg);       // DMT frame generator
    packetizer_destroy(_q->p_header);   // header packetizer
    modem_destroy(_q->mod_header);      // header modulator
    packetizer_destroy(_q->p_payload);  // payload packetizer
    modem_destroy(_q->mod_payload);     // payload modulator

    // free buffers/arrays
    free(_q->payload_enc);              // encoded payload bytes
    free(_q->X);                        // frequency-domain buffer
    free(_q->p);                        // subcarrier allocation

    // free main object memory
    free(_q);
}

void dmtflexframegen_reset(dmtflexframegen _q)
{
    // reset symbol counter and state
    _q->state = DMTFLEXFRAMEGEN_STATE_S0a;
    _q->frame_assembled = 0;
    _q->frame_complete = 0;
    _q->header_bit_index = 0;
    _q->payload_bit_index = 0;

    // free packetizer buffer

    // reset internal DMT frame generator object
    // NOTE: this is important for appropriately setting the pilot phases
    dmtframegen_reset(_q->fg);
}

// is frame assembled?
int dmtflexframegen_is_assembled(dmtflexframegen _q)
{
    return _q->frame_assembled;
}

void dmtflexframegen_print(dmtflexframegen _q)
{
    printf("dmtflexframegen:\n");
    printf("    num subcarriers     :   %-u\n", _q->M);
    printf("      * NULL            :   %-u\n", _q->M_null);
    printf("      * pilot           :   %-u\n", _q->M_pilot);
    printf("      * data            :   %-u\n", _q->M_data);
    printf("    cyclic prefix len   :   %-u\n", _q->cp_len);
    printf("    taper len           :   %-u\n", _q->taper_len);
    printf("    properties:\n");
    printf("      * mod scheme      :   %s\n", modulation_types[_q->props.mod_scheme].fullname);
    printf("      * fec (inner)     :   %s\n", fec_scheme_str[_q->props.fec0][1]);
    printf("      * fec (outer)     :   %s\n", fec_scheme_str[_q->props.fec1][1]);
    printf("      * CRC scheme      :   %s\n", crc_scheme_str[_q->props.check][1]);
    printf("    frame assembled     :   %s\n", _q->frame_assembled ? "yes" : "no");
    if (_q->frame_assembled) {
        printf("    payload:\n");
        printf("      * decoded bytes   :   %-u\n", _q->payload_dec_len);
        printf("      * encoded bytes   :   %-u\n", _q->payload_enc_len);
        printf("      * S0 symbols      :   %-u @ %u\n", DMTFLEXFRAME_NUM_S0, _q->M+_q->cp_len);
        printf("      * S1 symbols      :   %-u @ %u\n", DMTFLEXFRAME_NUM_S1, _q->M+_q->cp_len);
    }
}

// get dmtflexframegen properties
//  _q      :   frame generator object
//  _props  :   frame generator properties structure pointer
void dmtflexframegen_getprops(dmtflexframegen _q,
                               dmtflexframegenprops_s * _props)
{
    // copy properties structure to output pointer
    memcpy(_props, &_q->props, sizeof(dmtflexframegenprops_s));
}

void dmtflexframegen_setprops(dmtflexframegen _q,
                               dmtflexframegenprops_s * _props)
{
    // if properties object is NULL, initialize with defaults
    if (_props == NULL) {
        dmtflexframegen_setprops(_q, &dmtflexframegenprops_default);
        return;
    }

    // validate input
    if (_props->check == LIQUID_CRC_UNKNOWN || _props->check >= LIQUID_CRC_NUM_SCHEMES) {
        fprintf(stderr, "error: dmtflexframegen_setprops(), invalid/unsupported CRC scheme\n");
        exit(1);
    } else if (_props->fec0 == LIQUID_FEC_UNKNOWN || _props->fec1 == LIQUID_FEC_UNKNOWN) {
        fprintf(stderr, "error: dmtflexframegen_setprops(), invalid/unsupported FEC scheme\n");
        exit(1);
    } else if (_props->mod_scheme == LIQUID_MODEM_UNKNOWN ) {
        fprintf(stderr, "error: dmtflexframegen_setprops(), invalid/unsupported modulation scheme\n");
        exit(1);
    }

    // TODO : determine if re-configuration is necessary

    // copy properties to internal structure
    memcpy(&_q->props, _props, sizeof(dmtflexframegenprops_s));

    // reconfigure internal buffers, objects, etc.
    dmtflexframegen_reconfigure(_q);
}

// assemble a frame from an array of data
//  _q              :   DMT frame generator object
//  _header         :   frame header
//  _payload        :   payload data [size: _payload_len x 1]
//  _payload_len    :   payload data length
void dmtflexframegen_assemble(dmtflexframegen _q,
                               const unsigned char *  _header,
                               const unsigned char *  _payload,
                               unsigned int     _payload_len)
{
    // check payload length and reconfigure if necessary
    if (_payload_len != _q->payload_dec_len) {
        _q->payload_dec_len = _payload_len;
        dmtflexframegen_reconfigure(_q);
    }

    // copy user-defined header data
    memcpy(_q->header, _header, DMTFLEXFRAME_H_USER*sizeof(unsigned char));

    // encode full header
    dmtflexframegen_encode_header(_q);

    // encode payload
    // allocate the required buffer for the packetizer
    packetizer_encode(_q->p_payload, _payload, _q->payload_enc);

    // set assembled flag
    _q->frame_assembled = 1;

}

// write symbols of assembled frame
//  _q              :   DMT frame generator object
//  _buffer         :   output buffer [size: N+cp_len x 1]
int dmtflexframegen_writesymbol(dmtflexframegen       _q,
                                 liquid_float_complex * _buffer)
{
    // check if frame is actually assembled
    if ( !_q->frame_assembled ) {
        fprintf(stderr,"warning: dmtflexframegen_writesymbol(), frame not assembled\n");
        return 1;
    }

    switch (_q->state) {
    case DMTFLEXFRAMEGEN_STATE_S0a:
        // write S0 symbol (first)
        dmtflexframegen_write_S0a(_q, _buffer);
        break;

    case DMTFLEXFRAMEGEN_STATE_S0b:
        // write S0 symbol (second)
        dmtflexframegen_write_S0b(_q, _buffer);
        break;

    case DMTFLEXFRAMEGEN_STATE_S1:
        // write S1 symbols
        dmtflexframegen_write_S1(_q, _buffer);
        break;

    case DMTFLEXFRAMEGEN_STATE_HEADER:
        // write header symbols
        dmtflexframegen_write_header(_q, _buffer);
        break;

    case DMTFLEXFRAMEGEN_STATE_PAYLOAD:
        // write payload symbols
        dmtflexframegen_write_payload(_q, _buffer);
        break;

    default:
        fprintf(stderr,"error: dmtflexframegen_writesymbol(), unknown/unsupported internal state\n");
        exit(1);
    }

    if (_q->frame_complete) {
        // reset framing object
#if DEBUG_DMTFLEXFRAMEGEN
        printf(" ...resetting...\n");
#endif
        dmtflexframegen_reset(_q);
        return 1;
    }

    return 0;
}


//
// internal
//

// reconfigure internal buffers, objects, etc.
void dmtflexframegen_reconfigure(dmtflexframegen _q)
{
    // re-create payload packetizer
    _q->p_payload = packetizer_recreate(_q->p_payload,
                                        _q->payload_dec_len,
                                        _q->props.check,
                                        _q->props.fec0,
                                        _q->props.fec1);

    // re-allocate memory for encoded message
    _q->payload_enc_len = packetizer_get_enc_msg_len(_q->p_payload);
    _q->payload_enc = (unsigned char*) realloc(_q->payload_enc,
                                               _q->payload_enc_len*sizeof(unsigned char));
#if DEBUG_DMTFLEXFRAMEGEN
    printf(">>>> payload : %u (%u encoded)\n", _q->props.payload_len, _q->payload_enc_len);
#endif

    // re-create modem
    // TODO : only do this if necessary
    _q->mod_payload = modem_recreate(_q->mod_payload, _q->props.mod_scheme);

}

// encode header
void dmtflexframegen_encode_header(dmtflexframegen _q)
{
    // first 'n' bytes user data
    unsigned int n = DMTFLEXFRAME_H_USER;

    // first byte is for expansion/version validation
    _q->header[n+0] = DMTFLEXFRAME_PROTOCOL;

    // add payload length
    _q->header[n+1] = (_q->payload_dec_len >> 8) & 0xff;
    _q->header[n+2] = (_q->payload_dec_len     ) & 0xff;

    // add modulation scheme/depth (pack into single byte)
    _q->header[n+3]  = _q->props.mod_scheme;

    // add CRC, forward error-correction schemes
    //  CRC     : most-significant 3 bits of [n+4]
    //  fec0    : least-significant 5 bits of [n+4]
    //  fec1    : least-significant 5 bits of [n+5]
    _q->header[n+4]  = (_q->props.check & 0x07) << 5;
    _q->header[n+4] |= (_q->props.fec0) & 0x1f;
    _q->header[n+5]  = (_q->props.fec1) & 0x1f;

#if 0
    // Scramble before encoding otherwise bit errors will cause the
    // descrambler to create more errors
    // scramble header
    scramble_data(_q->header, DMTFLEXFRAME_H_DEC);
#endif

    // run packet encoder
    packetizer_encode(_q->p_header, _q->header, _q->header_enc);

#if 1
    // scramble header
    scramble_data(_q->header_enc, DMTFLEXFRAME_H_ENC);
#endif

#if 0
    // print header (decoded)
    unsigned int i;
    printf("header tx (dec) : ");
    for (i=0; i<DMTFLEXFRAME_H_DEC; i++)
        printf("%.2X ", _q->header[i]);
    printf("\n");

    // print header (encoded)
    printf("header tx (enc) : ");
    for (i=0; i<DMTFLEXFRAME_H_ENC; i++)
        printf("%.2X ", _q->header_enc[i]);
    printf("\n");
#endif
}

// write first S0 symbol
void dmtflexframegen_write_S0a(dmtflexframegen _q,
                                float complex * _buffer)
{
#if DEBUG_DMTFLEXFRAMEGEN
    printf("writing S0[a] symbol\n");
#endif

    // write S0 symbol into front of buffer
    dmtframegen_write_S0a(_q->fg, _buffer);

    // update state
    _q->state = DMTFLEXFRAMEGEN_STATE_S0b;
}

// write second S0 symbol
void dmtflexframegen_write_S0b(dmtflexframegen _q,
                                float complex * _buffer)
{
#if DEBUG_DMTFLEXFRAMEGEN
    printf("writing S0[b] symbol\n");
#endif

    // write S0 symbol into front of buffer
    dmtframegen_write_S0b(_q->fg, _buffer);

    // update state
    _q->state = DMTFLEXFRAMEGEN_STATE_S1;
}

// write S1 symbol
void dmtflexframegen_write_S1(dmtflexframegen _q,
                               float complex * _buffer)
{
#if DEBUG_DMTFLEXFRAMEGEN
    printf("writing S1 symbol\n");
#endif

    // write S1 symbol into end of buffer
    dmtframegen_write_S1(_q->fg, _buffer);

    // update state
    _q->state = DMTFLEXFRAMEGEN_STATE_HEADER;
}

// write header symbol
void dmtflexframegen_write_header(dmtflexframegen _q,
                                   float complex * _buffer)
{
#if DEBUG_DMTFLEXFRAMEGEN
    printf("writing header symbol\n");
#endif

    // load data onto data subcarriers
    unsigned int i;
    int sctype;
    for (i=0; i<_q->M; i++) {
        //
        sctype = _q->p[i];

        // 
        if (sctype == DMTFRAME_SCTYPE_DATA) {
            // load...
            unsigned char sym;

            if (_q->header_bit_index < (DMTFLEXFRAME_H_ENC * 8)) {
                // modulate header symbol onto data subcarrier
                unsigned int bps = modem_get_bps(_q->mod_header);

                liquid_unpack_array(_q->header_enc, DMTFLEXFRAME_H_ENC,
                      _q->header_bit_index, bps, &sym);

                _q->header_bit_index += bps;
            } else {
                //printf("  random header symbol\n");
                // load random symbol
                sym = modem_gen_rand_sym(_q->mod_header);
            }

            // printf("  writing symbol %3u / %3u (x = %8.5f + j%8.5f)\n", _q->header_bit_index, DMTFLEXFRAME_H_ENC * 8, crealf(_q->X[i]), cimagf(_q->X[i]));
            modem_modulate(_q->mod_header, sym, &_q->X[i]);
        } else {
            // ignore subcarrier (dmtframegen handles nulls and pilots)
            _q->X[i] = 0.0f;
        }
    }

    // write symbol
    dmtframegen_writesymbol(_q->fg, _q->X, _buffer);

    // check state
    if (_q->header_bit_index >= (DMTFLEXFRAME_H_ENC * 8) ) {
        _q->state = DMTFLEXFRAMEGEN_STATE_PAYLOAD;
    }
}

// write payload symbol
void dmtflexframegen_write_payload(dmtflexframegen _q,
                                    float complex * _buffer)
{
#if DEBUG_DMTFLEXFRAMEGEN
    printf("writing payload symbol\n");
#endif

    // load data onto data subcarriers
    unsigned int i;
    int sctype;
    for (i=0; i<_q->M; i++) {
        //
        sctype = _q->p[i];

        // 
        if (sctype == DMTFRAME_SCTYPE_DATA) {
            // load...
            unsigned char sym;

            if (_q->payload_bit_index < (_q->payload_enc_len * 8)) {
                // modulate payload symbol onto data subcarrier
                unsigned int bps = modem_get_bps(_q->mod_payload);

                liquid_unpack_array(_q->payload_enc, _q->payload_enc_len,
                      _q->payload_bit_index, bps, &sym);

                _q->payload_bit_index += bps;
            } else {
                // printf("  random payload symbol\n");
                // load random symbol
                sym = modem_gen_rand_sym(_q->mod_payload);
            }

            // printf("  writing symbol %3u / %3u (x = %8.5f + j%8.5f)\n", _q->payload_bit_index, _q->payload_enc_len * 8, crealf(_q->X[i]), cimagf(_q->X[i]));
            modem_modulate(_q->mod_payload, sym, &_q->X[i]);
        } else {
            // ignore subcarrier (dmtframegen handles nulls and pilots)
            _q->X[i] = 0.0f;
        }
    }

    // write symbol
    dmtframegen_writesymbol(_q->fg, _q->X, _buffer);

    // check to see if this is the last symbol in the payload
    if (_q->payload_bit_index >= (_q->payload_enc_len * 8))
        _q->frame_complete = 1;
}

