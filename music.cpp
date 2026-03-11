#include "music.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>

/* ═══════════════════════════════════════════════════════════════════════
 * Frequency table
 * Matches Python: freq = round(440 * 2^((midi-69)/12), 1)
 *                 where midi = (octave+1)*12 + semitone
 * This is the actual frequency used for BOTH T3 encoding AND pulse counting.
 * ═══════════════════════════════════════════════════════════════════════ */

static double note_actual_freq(int semitone, int octave)
{
    int midi = (octave + 1) * 12 + semitone;
    double freq = 440.0 * pow(2.0, (midi - 69) / 12.0);
    /* Round to 1 decimal place to match Python's round(..., 1) */
    return round(freq * 10.0) / 10.0;
}

/* ═══════════════════════════════════════════════════════════════════════
 * MIDI → T3 lookup table
 * Covers MIDI notes 60-107 (C4–B7).
 * midi = (octave+1)*12 + semitone, so:
 *   octave 4: midi 60-71
 *   octave 5: midi 72-83  ← verified hard-coded values
 *   octave 6: midi 84-95  ← verified hard-coded values
 *   octave 7: midi 96-107
 * ═══════════════════════════════════════════════════════════════════════ */

#define MIDI_T3_MIN  60
#define MIDI_T3_MAX  107

static uint8_t midi_t3_table[MIDI_T3_MAX - MIDI_T3_MIN + 1];
static int     table_built = 0;


static void limit_bluejay_pitch(int *semitone, int *octave)
{
    /* Bluejay max frequency ≈ 2290 Hz (T3 = 1) */
    const double MAX_FREQ = 2200.0;

    double freq = note_actual_freq(*semitone, *octave);

    if (freq > MAX_FREQ)
    {
        /* Drop one octave */
        (*octave)--;

        /* Safety clamp so we never go below valid range */
        if (*octave < 4)
            *octave = 4;
    }
}



static uint8_t midi_to_t3_precise(int midi)
{
    /* exact frequency from MIDI note */
    double freq = 440.0 * pow(2.0, (midi - 69) / 12.0);

    /* Bluejay timing formula */
    double t3f = 41000.0 / freq - 16.9;

    int t3 = (int)lround(t3f);

    if (t3 < 1)   t3 = 1;
    if (t3 > 255) t3 = 255;

    return (uint8_t)t3;
}


static void build_midi_t3_table(void)
{
    if (table_built) return;

    static const uint8_t oct5[12] = { 61,57,53,49,45,42,39,35,32,30,27,25 };
    static const uint8_t oct6[12] = { 23,20,18,16,15,12,11,10,8,7,5,4 };

    for (int midi = MIDI_T3_MIN; midi <= MIDI_T3_MAX; midi++)
    {
        int semitone = midi % 12;

        if (midi >= 72 && midi <= 83)
            midi_t3_table[midi - MIDI_T3_MIN] = oct5[semitone];

        else if (midi >= 84 && midi <= 95)
            midi_t3_table[midi - MIDI_T3_MIN] = oct6[semitone];

        else
            midi_t3_table[midi - MIDI_T3_MIN] = midi_to_t3_precise(midi);
    }

    table_built = 1;
}

/* semitone 0=C..11=B, octave 4-7 → T3 byte */
static uint8_t note_to_t3(int semitone, int octave)
{
    build_midi_t3_table();
    int midi = (octave + 1) * 12 + semitone;
    if (midi >= MIDI_T3_MIN && midi <= MIDI_T3_MAX)
        return midi_t3_table[midi - MIDI_T3_MIN];
    /* Fallback for out-of-range octaves */
    return midi_to_t3_precise(note_actual_freq(semitone, octave));
}

/* ═══════════════════════════════════════════════════════════════════════
 * Note name parsing
 * ═══════════════════════════════════════════════════════════════════════ */

static const char *NOTE_NAMES[12] = {
    "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
};

static const struct { const char *flat; const char *sharp; } FLAT_MAP[] = {
    {"DB","C#"}, {"EB","D#"}, {"FB","E"}, {"GB","F#"},
    {"AB","G#"}, {"BB","A#"}, {"CB","B"}, {NULL, NULL}
};

/* Advances *p past the note name. Returns semitone 0-11, or -1 on failure. */
static int parse_note_name(const char **p)
{
    char name[4] = {0};
    int  len = 0;

    if (!isalpha((unsigned char)**p)) return -1;
    name[len++] = (char)toupper((unsigned char)**p);
    (*p)++;

    if (**p == '#' || **p == 'b') {
        name[len++] = (char)toupper((unsigned char)**p);
        (*p)++;
    }

    for (int i = 0; FLAT_MAP[i].flat; i++)
        if (strcmp(name, FLAT_MAP[i].flat) == 0)
        { strcpy(name, FLAT_MAP[i].sharp); break; }

    for (int i = 0; i < 12; i++)
        if (strcmp(name, NOTE_NAMES[i]) == 0) return i;

    return -1;
}

/* ═══════════════════════════════════════════════════════════════════════
 * Duration string → divisor integer
 * Accepts "8", "4", "1/8", "1/4" etc.
 * ═══════════════════════════════════════════════════════════════════════ */
static int parse_duration_str(const char *s)
{
    const char *slash = strchr(s, '/');
    if (slash)
        return atoi(slash + 1);   /* "1/8" → 8 */
    return atoi(s);               /* "8"   → 8 */
}

/* ═══════════════════════════════════════════════════════════════════════
 * Internal encoder — operates on const char*
 * ═══════════════════════════════════════════════════════════════════════ */

static int encode_notation(const char *notation, int bpm, uint8_t out[BLUEJAY_ARRAY_SIZE])
{
    memset(out, 0, BLUEJAY_ARRAY_SIZE);

    /* Tokenise — work on a mutable copy */
    char buf[2048];
    strncpy(buf, notation, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    for (char *c = buf; *c; c++) if (*c == ',') *c = ' ';

    char  *raw_toks[512];
    int    ntok = 0;
    char  *tok  = strtok(buf, " \t\r\n");
    while (tok && ntok < (int)(sizeof(raw_toks)/sizeof(raw_toks[0]))) {
        raw_toks[ntok++] = tok;
        tok = strtok(NULL, " \t\r\n");
    }

    /* Parse tokens into events */
    typedef struct { int semitone; int octave; int dur_div; } Event;
    Event events[512];
    int   nevents = 0;

    int dur_counts[33] = {0};
    int oct_counts[9]  = {0};

    int i = 0;
    while (i < ntok) {
        const char *t = raw_toks[i];

        if (t[0] == 'P' || t[0] == 'p') {
            int d = parse_duration_str(t + 1);
            if (d <= 0) return -1;
            events[nevents].semitone = -1;
            events[nevents].octave   = 0;
            events[nevents].dur_div  = d;
            nevents++;
            if (d <= 32) dur_counts[d]++;
            i++;
        } else {
            const char *p = t;
            int semi = parse_note_name(&p);
            if (semi < 0) return -1;

            int octave = 5;
            if (*p >= '4' && *p <= '7') { octave = *p - '0'; p++; }

            if (i + 1 >= ntok) return -1;
            int d = parse_duration_str(raw_toks[i + 1]);
            if (d <= 0) return -1;

            events[nevents].semitone = semi;
            events[nevents].octave   = octave;
            events[nevents].dur_div  = d;
            nevents++;
            if (d <= 32)  dur_counts[d]++;
            if (octave >= 0 && octave <= 8) oct_counts[octave]++;
            i += 2;
        }
    }

    if (nevents == 0) return -1;

    /* Determine default duration and octave (most common) */
    int default_dur = 4, best_dur_cnt = 0;
    for (int d = 1; d <= 32; d++)
        if (dur_counts[d] > best_dur_cnt) { best_dur_cnt = dur_counts[d]; default_dur = d; }

    int default_oct = 5, best_oct_cnt = 0;
    for (int o = 4; o <= 7; o++)
        if (oct_counts[o] > best_oct_cnt) { best_oct_cnt = oct_counts[o]; default_oct = o; }

    /* Build (T4, T3) pairs */
    double ms_per_beat = 60000.0 / bpm;
    uint8_t pairs[BLUEJAY_MAX_PAIRS * 2];
    int     npairs = 0;

    for (int e = 0; e < nevents && npairs < BLUEJAY_MAX_PAIRS; e++) {
        double dur_ms = ms_per_beat * 4.0 / events[e].dur_div;

        if (events[e].semitone < 0) {
            /* Pause: T3=0, T4=ms clamped 1-255 */
            int t4 = (int)round(dur_ms);
            if (t4 < 1)   t4 = 1;
            if (t4 > 255) t4 = 255;
            pairs[npairs * 2]     = (uint8_t)t4;
            pairs[npairs * 2 + 1] = 0;
            npairs++;
        } else {
            int semi = events[e].semitone;
            int oct  = events[e].octave;

            /* Automatically lower notes above C#7 */
            limit_bluejay_pitch(&semi, &oct);

            uint8_t t3 = note_to_t3(semi, oct);

            /*
             * FIX: use the actual note frequency for pulse counting,
             * NOT the inverse of the T3 byte (41000/(t3+16.9)).
             * The T3-inverse introduces rounding error that produces
             * wrong pair counts, especially for higher-octave notes.
             * This matches the Python reference tool exactly.
             */
            double actual_freq = note_actual_freq(semi, oct);
           // double actual_freq = note_actual_freq(events[e].semitone, events[e].octave);
            double pulses_remaining = dur_ms / 1000.0 * actual_freq;

            while (pulses_remaining > 0.5 && npairs < BLUEJAY_MAX_PAIRS) {
                int chunk = (pulses_remaining >= 255.0)
                ? 255
                : (int)round(pulses_remaining);
                if (chunk < 1) chunk = 1;
                pairs[npairs * 2]     = (uint8_t)chunk;
                pairs[npairs * 2 + 1] = t3;
                npairs++;
                pulses_remaining -= chunk;
            }
        }
    }

    /* Write 128-byte header + pairs */
    out[0] = 0;
    out[1] = (uint8_t)(bpm > 255 ? 255 : bpm);
    out[2] = (uint8_t)default_oct;
    out[3] = (uint8_t)default_dur;
    for (int p = 0; p < npairs; p++) {
        out[4 + p * 2]     = pairs[p * 2];
        out[4 + p * 2 + 1] = pairs[p * 2 + 1];
    }

    return npairs;
}


int blheli32_to_bluejay_array(const QString &notation, int bpm, uint8_t out[BLUEJAY_ARRAY_SIZE])
{
    QByteArray utf8 = notation.toUtf8();
    return encode_notation(utf8.constData(), bpm, out);
}

void bluejay_array_to_c_literal(const uint8_t arr[BLUEJAY_ARRAY_SIZE],
                                const char *var_name)
{
    printf("uint8_t %s[%d] = {\n", var_name, BLUEJAY_ARRAY_SIZE);
    printf("    /* [0]  reserved */ 0x%02X,\n",  arr[0]);
    printf("    /* [1]  BPM      */ 0x%02X,  // %d BPM\n", arr[1], arr[1]);
    printf("    /* [2]  def oct  */ 0x%02X,  // octave %d\n", arr[2], arr[2]);
    printf("    /* [3]  def dur  */ 0x%02X,  // 1/%d\n", arr[3], arr[3]);
    printf("\n    /* [4..127] (T4, T3) note pairs */\n");
    for (int i = 4; i < BLUEJAY_ARRAY_SIZE; i += 2) {
        uint8_t t4 = arr[i], t3 = arr[i + 1];
        if (t4 == 0 && t3 == 0)
            printf("    0x00, 0x00,  // (unused)\n");
        else if (t3 == 0)
            printf("    0x%02X, 0x00,  // pause %d ms\n", t4, t4);
        else
            printf("    0x%02X, 0x%02X,  // T3=0x%02X (~%.0f Hz), %d pulses\n",
                   t4, t3, t3, 41000.0 / (t3 + 16.9), t4);
    }
    printf("};\n");
}

