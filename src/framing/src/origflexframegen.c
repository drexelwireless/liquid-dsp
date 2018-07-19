/*
 * Copyright (c) 2007 - 2014 Joseph Gaeddert
 *
 * This file is part of liquid.
 *
 * liquid is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * liquid is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with liquid.  If not, see <http://www.gnu.org/licenses/>.
 */

//
// flexframegen.c
//
// flexible frame generator
//

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <complex.h>

#include "liquid.internal.h"

#define DEBUG_FLEXFRAMEGEN          0

// reconfigure internal properties
void origflexframegen_reconfigure(origflexframegen _q);

// encode header
void origflexframegen_encode_header(origflexframegen _q);

void origflexframegen_modulate_header(origflexframegen _q);

void origflexframegen_modulate_payload(origflexframegen _q);

//
void origflexframegen_write_preamble(origflexframegen _q, float complex * _buffer);
void origflexframegen_write_header(  origflexframegen _q, float complex * _buffer);
void origflexframegen_write_payload( origflexframegen _q, float complex * _buffer);
void origflexframegen_write_tail(    origflexframegen _q, float complex * _buffer);


// default flexframegen properties
static origflexframegenprops_s origflexframegenprops_default = {
    LIQUID_CRC_16,      // check
    LIQUID_FEC_NONE,    // fec0
    LIQUID_FEC_NONE,    // fec1
    LIQUID_MODEM_BPSK,  // mod_scheme
};

void origflexframegenprops_init_default(origflexframegenprops_s * _props)
{
    memmove(_props, &origflexframegenprops_default, sizeof(origflexframegenprops_s));
}

struct origflexframegen_s {
    // BPSK preamble
    float preamble_pn[64];              // p/n sequence
    // post-p/n sequence symbols?

    // header (BPSK)
    modem mod_header;                   // header BPSK modulator
    packetizer p_header;                // header packetizer
    unsigned char * header;             // header data (uncoded)
    unsigned int    header_user_len;    // header user section length
    unsigned int    header_dec_len;     // header length (decoded)
    unsigned char * header_enc;         // header data (encoded)
    unsigned int    header_enc_len;     // header length (encoded)
    unsigned char * header_mod;         // header symbols (modem input)
    unsigned int    header_mod_len;     // header symbol length

    // payload
    packetizer p_payload;               // payload packetizer
    unsigned int payload_dec_len;       // payload length (num un-encoded bytes)
    modem mod_payload;                  // payload modulator
    unsigned char * payload_enc;        // payload data (encoded bytes)
    unsigned char * payload_mod;        // payload symbols (modem input)
    unsigned int payload_enc_len;       // length of encoded payload
    unsigned int payload_mod_len;       // length of encoded payload

    // interpolator
    unsigned int k;                     // interp samples/symbol (fixed at 2)
    unsigned int m;                     // interp filter delay (symbols)
    float        beta;                  // excess bandwidth factor
    firinterp_crcf interp;              // interpolator object

    // counters/states
    unsigned int symbol_counter;         // output symbol number
    enum {
        STATE_PREAMBLE=0,               // write preamble p/n sequence
        STATE_HEADER,                   // write header symbols
        STATE_PAYLOAD,                  // write payload symbols
        STATE_TAIL,                     // tail symbols
    } state;
    int frame_assembled;                // frame assembled flag
    int frame_complete;                 // frame completed flag

    // properties
    flexframegenprops_s props;
};

origflexframegen origflexframegen_create(origflexframegenprops_s * _fgprops)
{
    origflexframegen q = (origflexframegen) malloc(sizeof(struct origflexframegen_s));

    unsigned int i;

    // ensure frame is not assembled
    q->frame_assembled = 0;

    // generate pn sequence
    msequence ms = msequence_create(6, 0x005b, 1);
    for (i=0; i<64; i++)
        q->preamble_pn[i] = (msequence_advance(ms)) ? 1.0f : -1.0f;
    msequence_destroy(ms);

    // create header objects
    q->mod_header = NULL;
    q->p_header = NULL;
    q->header = NULL;
    q->header_enc = NULL;
    q->header_mod = NULL;
    origflexframegen_set_header_len(q, ORIGFLEXFRAME_H_USER_DEFAULT);

    // initial memory allocation for payload
    q->payload_dec_len = 1;
    q->p_payload = packetizer_create(q->payload_dec_len,
                                     LIQUID_CRC_NONE,
                                     LIQUID_FEC_NONE,
                                     LIQUID_FEC_NONE);
    q->payload_enc_len = packetizer_get_enc_msg_len(q->p_payload);
    q->payload_enc = (unsigned char*) malloc(q->payload_enc_len*sizeof(unsigned char));

    q->payload_mod_len = 1;
    q->payload_mod = (unsigned char*) malloc(1*sizeof(unsigned char));

    // create payload modem (initially QPSK, overridden by properties)
    q->mod_payload = modem_create(LIQUID_MODEM_QPSK);
    
    // create pulse-shaping filter
    q->k    = 2;
    q->m    = 7;
    q->beta = 0.25f;
    q->interp = firinterp_crcf_create_prototype(LIQUID_FIRFILT_ARKAISER,q->k,q->m,q->beta,0);

    // initialize properties
    origflexframegen_setprops(q, _fgprops);

    // reset
    origflexframegen_reset(q);

    // return pointer to main object
    return q;
}

void origflexframegen_destroy(origflexframegen _q)
{
    // destroy internal objects
    packetizer_destroy(_q->p_header);   // header packetizer
    modem_destroy(_q->mod_header);      // header modulator
    packetizer_destroy(_q->p_payload);  // payload packetizer
    modem_destroy(_q->mod_payload);     // payload modulator
    firinterp_crcf_destroy(_q->interp); // pulse-shaping filter

    // free buffers/arrays
    free(_q->header);                   // header data (uncoded)
    free(_q->header_enc);               // header data (encoded)
    free(_q->header_mod);               // header symbols (modem input)
    free(_q->payload_enc);              // encoded payload bytes
    free(_q->payload_mod);              // modulated payload symbols

    // destroy frame generator
    free(_q);
}

// print flexframegen object internals
void origflexframegen_print(origflexframegen _q)
{
    printf("flexframegen:\n");
    printf("    p/n sequence len    :   %u\n",       64);
    printf("    header len          :   %u\n",      _q->header_mod_len);
    printf("    payload len, uncoded:   %u bytes\n", _q->payload_dec_len);
    printf("    payload crc         :   %s\n", crc_scheme_str[_q->props.check][1]);
    printf("    fec (inner)         :   %s\n", fec_scheme_str[_q->props.fec0][1]);
    printf("    fec (outer)         :   %s\n", fec_scheme_str[_q->props.fec1][1]);
    printf("    payload len, coded  :   %u bytes\n", _q->payload_enc_len);
    printf("    modulation scheme   :   %s\n", modulation_types[_q->props.mod_scheme].name);
    printf("    num payload symbols :   %u\n", _q->payload_mod_len);
}

// reset flexframegen object internals
void origflexframegen_reset(origflexframegen _q)
{
    // reset internal counters
    _q->symbol_counter  = 0;
    _q->frame_assembled = 0;
    _q->frame_complete  = 0;

    // reset state
    _q->state = STATE_PREAMBLE;
}

// is frame assembled?
int origflexframegen_is_assembled(origflexframegen _q)
{
    return _q->frame_assembled;
}

// get flexframegen properties
//  _q     :   frame generator object
//  _props  :   frame generator properties structure pointer
void origflexframegen_getprops(origflexframegen          _q,
                               origflexframegenprops_s * _props)
{
    // copy properties structure to output pointer
    memmove(_props, &_q->props, sizeof(flexframegenprops_s));
}

// set flexframegen properties
//  _q      :   frame generator object
//  _props  :   frame generator properties structure pointer
void origflexframegen_setprops(origflexframegen          _q,
                               origflexframegenprops_s * _props)
{
    // if frame is already assembled, give warning
    if (_q->frame_assembled) {
        fprintf(stderr, "warning: origflexframegen_setprops(), frame is already assembled; must reset() first\n");
        return;
    }

    // if properties object is NULL, initialize with defaults
    if (_props == NULL) {
        origflexframegen_setprops(_q, &origflexframegenprops_default);
        return;
    }

    // validate input
    if (_props->check == LIQUID_CRC_UNKNOWN || _props->check >= LIQUID_CRC_NUM_SCHEMES) {
        fprintf(stderr, "error: origflexframegen_setprops(), invalid/unsupported CRC scheme\n");
        exit(1);
    } else if (_props->fec0 == LIQUID_FEC_UNKNOWN || _props->fec1 == LIQUID_FEC_UNKNOWN) {
        fprintf(stderr, "error: origflexframegen_setprops(), invalid/unsupported FEC scheme\n");
        exit(1);
    } else if (_props->mod_scheme == LIQUID_MODEM_UNKNOWN ) {
        fprintf(stderr, "error: origflexframegen_setprops(), invalid/unsupported modulation scheme\n");
        exit(1);
    }

    // TODO : determine if re-configuration is necessary

    // copy properties to internal structure
    memmove(&_q->props, _props, sizeof(flexframegenprops_s));

    // reconfigure payload buffers (reallocate as necessary)
    origflexframegen_reconfigure(_q);
}

void origflexframegen_set_header_len(origflexframegen _q,
                                     unsigned int _len)
{
    // if frame is already assembled, give warning
    if (_q->frame_assembled) {
        fprintf(stderr, "warning: origflexframegen_set_header_len(), frame is already assembled; must reset() first\n");
        return;
    }

    _q->header_user_len = _len;
    _q->header_dec_len = ORIGFLEXFRAME_H_DEC + _q->header_user_len;
    _q->header = (unsigned char *) realloc(_q->header, _q->header_dec_len*sizeof(unsigned char));

    if (_q->mod_header)
        modem_destroy(_q->mod_header);
    
    _q->mod_header = modem_create(ORIGFLEXFRAME_H_MOD);

    if (_q->p_header)
        packetizer_destroy(_q->p_header);
    
    _q->p_header = packetizer_create(_q->header_dec_len,
                                     ORIGFLEXFRAME_H_CRC,
                                     ORIGFLEXFRAME_H_FEC0,
                                     ORIGFLEXFRAME_H_FEC1);

    _q->header_enc_len = packetizer_get_enc_msg_len(_q->p_header);
    _q->header_enc = (unsigned char *) realloc(_q->header_enc, _q->header_enc_len*sizeof(unsigned char));

    unsigned int bps = modem_get_bps(_q->mod_header);
    div_t d = div(8*_q->header_enc_len, bps);
    
    _q->header_mod_len = d.quot + (d.rem ? 1 : 0);
    _q->header_mod = (unsigned char*) realloc(_q->header_mod, _q->header_mod_len*sizeof(unsigned char));
}

// get frame length (number of samples)
unsigned int origflexframegen_getframelen(origflexframegen _q)
{
    if (!_q->frame_assembled) {
        fprintf(stderr,"warning: origflexframegen_getframelen(), frame not assembled!\n");
        return 0;
    }
    unsigned int num_frame_symbols =
            64 +                    // preamble p/n sequence length
            _q->header_mod_len +    // header symbols
            _q->payload_mod_len +   // number of modulation symbols
            2*_q->m;                // number of tail symbols

    return num_frame_symbols*_q->k; // k samples/symbol
}

// exectue frame generator (create the frame)
//  _q              :   frame generator object
//  _header         :   user-defined header
//  _payload        :   variable payload buffer (configured by setprops method)
//  _payload_len    :   length of payload
void origflexframegen_assemble(origflexframegen    _q,
                               unsigned char * _header,
                               unsigned char * _payload,
                               unsigned int    _payload_len)
{
    // check payload length and reconfigure if necessary
    if (_payload_len != _q->payload_dec_len) {
        _q->payload_dec_len = _payload_len;
        origflexframegen_reconfigure(_q);
    }

    // set assembled flag
    _q->frame_assembled = 1;

    // copy user-defined header data
    memmove(_q->header, _header, _q->header_user_len*sizeof(unsigned char));

    // encode full header
    origflexframegen_encode_header(_q);

    // modulate header
    origflexframegen_modulate_header(_q);

    // encode payload
    packetizer_encode(_q->p_payload, _payload, _q->payload_enc);

    // 
    // pack modem symbols
    //

    // clear payload
    memset(_q->payload_mod, 0x00, _q->payload_mod_len);

    // repack 8-bit payload bytes into 'bps'-bit payload symbols
    unsigned int bps = modulation_types[_q->props.mod_scheme].bps;
    unsigned int num_written;
    liquid_repack_bytes(_q->payload_enc,  8,  _q->payload_enc_len,
                        _q->payload_mod, bps, _q->payload_mod_len,
                        &num_written);

#if DEBUG_FLEXFRAMEGEN
    printf("wrote %u symbols (expected %u)\n", num_written, _q->payload_mod_len);
    origflexframegen_print(_q);
#endif
}

// write symbols of assembled frame
//  _q              :   frame generator object
//  _buffer         :   output buffer [size: N+cp_len x 1]
int origflexframegen_write_samples(origflexframegen    _q,
                                   float complex * _buffer)
{
    // check if frame is actually assembled
    if ( !_q->frame_assembled ) {
        fprintf(stderr,"warning: origflexframegen_writesymbol(), frame not assembled\n");
        return 1;
    }

    switch (_q->state) {
    case STATE_PREAMBLE:
        // write preamble
        origflexframegen_write_preamble(_q, _buffer);
        break;
    case STATE_HEADER:
        // write header symbols
        origflexframegen_write_header(_q, _buffer);
        break;
    case STATE_PAYLOAD:
        // write payload symbols
        origflexframegen_write_payload(_q, _buffer);
        break;
    case STATE_TAIL:
        // write tail symbols
        origflexframegen_write_tail(_q, _buffer);
        break;
    default:
        fprintf(stderr,"error: origflexframegen_writesymbol(), unknown/unsupported internal state\n");
        exit(1);
    }

    if (_q->frame_complete) {
        // reset framing object
        origflexframegen_reset(_q);
        return 1;
    }

    return 0;
}

//
// internal
//

// reconfigure internal buffers, objects, etc.
void origflexframegen_reconfigure(origflexframegen _q)
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
#if DEBUG_FLEXFRAMEGEN
    printf(">>>> payload : %u (%u encoded)\n", _q->payload_dec_len, _q->payload_enc_len);
#endif

    // re-create modem
    // TODO : only do this if necessary
    _q->mod_payload = modem_recreate(_q->mod_payload, _q->props.mod_scheme);

    // re-allocate memory for payload modem symbols
    unsigned int bps = modulation_types[_q->props.mod_scheme].bps;
    div_t d = div(8*_q->payload_enc_len, bps);
    _q->payload_mod_len = d.quot + (d.rem ? 1 : 0);
    _q->payload_mod = (unsigned char*)realloc(_q->payload_mod,
                                              _q->payload_mod_len*sizeof(unsigned char));
#if DEBUG_FLEXFRAMEGEN
    printf(">>>> payload mod length : %u\n", _q->payload_mod_len);
#endif
}

// encode header of flexframe
void origflexframegen_encode_header(origflexframegen _q)
{
    // first several bytes of header are user-defined
    unsigned int n = _q->header_user_len;

    // add ORIGFLEXFRAME_VERSION
    _q->header[n+0] = ORIGFLEXFRAME_VERSION;

    // add payload length
    _q->header[n+1] = (_q->payload_dec_len >> 8) & 0xff;
    _q->header[n+2] = (_q->payload_dec_len     ) & 0xff;

    // add modulation scheme/depth (pack into single byte)
    _q->header[n+3]  = (unsigned int)(_q->props.mod_scheme);

    // add CRC, forward error-correction schemes
    //  CRC     : most-significant 3 bits of [n+4]
    //  fec0    : least-significant 5 bits of [n+4]
    //  fec1    : least-significant 5 bits of [n+5]
    _q->header[n+4]  = (_q->props.check & 0x07) << 5;
    _q->header[n+4] |= (_q->props.fec0) & 0x1f;
    _q->header[n+5]  = (_q->props.fec1) & 0x1f;

    // run packet encoder
    packetizer_encode(_q->p_header, _q->header, _q->header_enc);

#if DEBUG_FLEXFRAMEGEN
    // print header (decoded)
    unsigned int i;
    printf("header tx (dec) : ");
    for (i=0; i<19; i++)
        printf("%.2X ", _q->header[i]);
    printf("\n");

    // print header (encoded)
    printf("header tx (enc) : ");
    for (i=0; i<32; i++)
        printf("%.2X ", _q->header_enc[i]);
    printf("\n");
#endif
}

// modulate header into symbols
void origflexframegen_modulate_header(origflexframegen _q)
{
    // repack 8-bit payload bytes into 'bps'-bit payload symbols
    unsigned int bps = modem_get_bps(_q->mod_header);
    unsigned int num_written;
    liquid_repack_bytes(_q->header_enc,  8,  _q->header_enc_len,
                        _q->header_mod, bps, _q->header_mod_len,
                        &num_written);
}

// modulate payload
void origflexframegen_modulate_payload(origflexframegen _q)
{
}

// write preamble
void origflexframegen_write_preamble(origflexframegen    _q,
                                     float complex * _buffer)
{
#if DEBUG_FLEXFRAMEGEN
    //printf("writing preamble symbol %u\n", _q->symbol_counter);
#endif

    // interpolate symbol
    float complex s = _q->preamble_pn[_q->symbol_counter];
    firinterp_crcf_execute(_q->interp, s, _buffer);

    // increment symbol counter
    _q->symbol_counter++;

    // check state
    if (_q->symbol_counter == 64) {
        _q->symbol_counter = 0;
        _q->state = STATE_HEADER;
    }
}

// write header
void origflexframegen_write_header(origflexframegen    _q,
                                   float complex * _buffer)
{
#if DEBUG_FLEXFRAMEGEN
    //printf("writing header symbol %u\n", _q->symbol_counter);
#endif

    float complex s;
    modem_modulate(_q->mod_header, _q->header_mod[_q->symbol_counter], &s);

    // interpolate symbol
    firinterp_crcf_execute(_q->interp, s, _buffer);

    // increment symbol counter
    _q->symbol_counter++;

    // check state
    if (_q->symbol_counter == _q->header_mod_len) {
        _q->symbol_counter = 0;
        _q->state = STATE_PAYLOAD;
    }
}

// write payload
void origflexframegen_write_payload(origflexframegen    _q,
                                    float complex * _buffer)
{
#if DEBUG_FLEXFRAMEGEN
    //printf("writing payload symbol %u\n", _q->symbol_counter);
#endif

    float complex s;
    modem_modulate(_q->mod_payload, _q->payload_mod[_q->symbol_counter], &s);

    // interpolate symbol
    firinterp_crcf_execute(_q->interp, s, _buffer);

    // increment symbol counter
    _q->symbol_counter++;

    // check state
    if (_q->symbol_counter == _q->payload_mod_len) {
        _q->symbol_counter = 0;
        _q->state = STATE_TAIL;
    }
}

// write tail
void origflexframegen_write_tail(origflexframegen    _q,
                                 float complex * _buffer)
{
#if DEBUG_FLEXFRAMEGEN
    //printf("writing tail symbol %u\n", _q->symbol_counter);
#endif

    // interpolate symbol
    firinterp_crcf_execute(_q->interp, 0.0f, _buffer);

    // increment symbol counter
    _q->symbol_counter++;

    // check state
    if (_q->symbol_counter == 2*_q->m)
        _q->frame_complete = 1;
}

