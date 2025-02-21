/*
Overview:
The SDR's core is quite simple:
We convert the entire sampled band into frequency domain 
(the stuff you see in the waterfalls) by simply passing 
the I and Q samples to a library called the FFT3W.

As the FFT turns time samples signals into their frequency bins, the signals are spread on both sides
of the 0 hz line. 

The FFT lines up not just amplitudes of signals at each frequency but also their phase.
A reverse FFT will add up all these sinewaves and reproduce the signal.

To bring a signal to baseband, we just rotate the bins around until the carrier of 
the SSB signal is at 0 bin. Then, we filter out the rest.

The filter is very simple, we just draw the filter shape in frequency domain
and multiply the FFT bin with that.
 
Once, we are left just the baseband signal in the FFT, we do an inverse FFT
and convert the signals back to time domain.

The basic receiver is just that.

HOW THE FILTER WORKS
This is a little tricky, it took me some time to understand. 
To begin with understand that you can convert a series of time samples of a signal
to frequency domain and convert it back to get back the original signal.
Time and Frequency domains are just two ways to represent a signal. We hams are
find it more convenient to handle frequency domain.
We tune to a band, we schedule calls on some, we scan, etc. We all understand 
the waterfall.

So here goes:

1. When you see a single spike on the waterfall, it means there is a cw signal.
When you see the time samples, you will see a continuous sinewave. 

2. Similarly, if there is a blip in the time samples, it spreads across the entire
waterfall.

If you stare at the two above statements, you will realize that what appears as a 
single sample standing out in one domain corresponds to a continuous response
in the other.

So, if we convert a signal to frequency domain, zero all the bins except the
frequency that we intend to hear, convert it back to time domain and
play it through the speaker? It will work, almost. But not really. The reason is
that some signals fall between two bins. These and other signal types will generate
all kinds of artifacts and clicks. You need a filter that is smooth.
There are ways to smoothen it. 

I am following a method that Phil Karn suggested to me. I first produce a 
'perfect filter' in frequency domain by setting frequency bins of the frequency
that I need to '1' and the rest to zeros. Then, I convert this back to time domain using 
inverse FFT. Now, if you think about the point 1 and 2 made above, you will
can guess that the time domain representation of the filter's shape will
have continuous waveform. 
*/

#ifndef SDR_H_
#define SDR_H_

#include <fftw3.h>
#include <complex.h>
#include <math.h>
#include <time.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* Queue structure for buffering data */
struct Queue
{
    int id;
    int head;
    int tail;
    int stall;
    int *data;
    unsigned int underflow;
    unsigned int overflow;
    unsigned int max_q;
};

void q_init(struct Queue *p, int32_t length);
int q_length(struct Queue *p);
int32_t q_read(struct Queue *p);
int q_write(struct Queue *p, int w);
void q_empty(struct Queue *p);

#define SAMPLE_RATE 48000
#define MAX_BINS 2048

/*
All the incoming samples are converted to frequency domain in sound_process(). 
The fft_bins stores these as frequency bins.
These fft_bins are also used to paint the spectrum and waterfall.

You can have any number of radios working with different slices of the spectrum.
At the moment, only SSB (and CW as a single sideband signal) are demodulated.
Each receiver is inserted as a node in a linked list that starts at rx_list.
*/

extern float fft_bins[];
extern int spectrum_plot[];
extern struct filter *ssb;

/* VFO definitions */
#define MAX_PHASE_COUNT (16385)
struct vfo {
    int freq_hz;
    int phase;
    int phase_increment;
};

void vfo_init_phase_table();
void vfo_start(struct vfo *v, int frequency_hz, int start_phase);
int vfo_read(struct vfo *v);

/* Filter definitions */
struct filter {
    complex float *fir_coeff;
    complex float *overlap;
    int N;
    int L;
    int M;
};

struct filter *filter_new(int input_length, int impulse_length);
int filter_tune(struct filter *f, float const low, float const high, float const kaiser_beta);
int make_hann_window(float *window, int max_count);
void filter_print(struct filter *f);
long set_bfo_offset(int offset, long freq);
void resetup_oscillators();
int get_bfo_offset();

/* Inline functions to compute the norm of a complex number */
static inline float const cnrmf(const complex float x) {
    return crealf(x)*crealf(x) + cimagf(x)*cimagf(x);
}
static inline double const cnrm(const complex double x) {
    return creal(x)*creal(x) + cimag(x)*cimag(x);
}

#define power2dB(x) (10*log10f(x))

/* Modulation Mode Constants */
/* Original modes: */
#define MODE_USB       0
#define MODE_LSB       1
#define MODE_CW        2
#define MODE_CWR       3
#define MODE_NBFM      4 
#define MODE_AM        5 
#define MODE_FT8       6  
#define MODE_PSK31     7 
#define MODE_RTTY      8 
#define MODE_DIGITAL   9 
#define MODE_2TONE     10 
#define MODE_CALIBRATE 11 

/* Added FM mode:
   MODE_FM is added to support full (or wideband) Frequency Modulation.
   FM mode will be selectable and handled internally like any other mode,
   with specific DSP routines (to be added in the signal processing functions)
   that implement the frequency deviation based on the modulating signal.
*/
#define MODE_FM        12

/* The struct rx holds the state for each receiver/transmitter instance.
   The mode field selects the modulation/demodulation technique.
   Modes currently include:
     MODE_USB, MODE_LSB, MODE_CW, MODE_CWR, MODE_NBFM, MODE_AM,
     MODE_FT8, MODE_PSK31, MODE_RTTY, MODE_DIGITAL, MODE_2TONE, MODE_CALIBRATE,
     and now MODE_FM.
*/
struct rx {
    long tuned_bin;          // Tuned FFT bin (translates to frequency)
    short mode;              // Modulation mode (e.g., MODE_USB, MODE_LSB, ..., MODE_FM)
    int low_hz;              // Lower frequency cutoff for filtering
    int high_hz;             // Upper frequency cutoff for filtering
    fftw_plan plan_rev;      // Inverse FFT plan to convert frequency domain back to time
    fftw_complex *fft_freq;  // Frequency domain data
    fftw_complex *fft_time;  // Time domain data after inverse FFT

    /* AGC and signal averaging parameters */
    int agc_speed;
    int agc_threshold;
    int agc_loop;
    double signal_strength;
    double agc_gain;
    int agc_decay_rate;
    double signal_avg;
    
    struct filter *filter;   // Convolution filter for baseband processing
    int output;              // Output destination (-1 = none, 0 = audio, others for network)
    struct rx* next;         // Next receiver/transmitter in the linked list
};

extern struct rx *rx_list;
extern int freq_hdr;

/* Function declarations for setting LO, volume, and command handling */
void set_lo(int frequency);
void set_volume(double v);
void sdr_request(char *request, char *response);
void cmd_exec(char *cmd);
void sdr_modulation_update(int32_t *samples, int count, double scale_up);

/* Modem functions from modems.c */
void modem_rx(int mode, int32_t *samples, int count);
void modem_set_pitch(int pitch, int mode);
void modem_init();
int get_tx_data_byte(char *c);
int get_tx_data_length();
void modem_poll(int mode);
float modem_next_sample(int mode);
void modem_abort();

/* Transmit control functions */
int is_in_tx();
#define TX_OFF 0
#define TX_PTT 1
#define TX_SOFT 2 
void tx_on(int trigger);
void tx_off();
long get_freq();
int get_passband_bw();
void hamlib_tx(int tx_on);
int get_default_passband_bw();
int get_pitch();
time_t time_sbitx();

/* CW definitions */
#define CW_IDLE     0
#define CW_DASH     1 
#define CW_DOT      2 
#define CW_DOT_DELAY 4
#define CW_DASH_DELAY 8 
#define CW_WORD_DELAY 16 
#define CW_DOWN     32 

/* CW input method definitions */
#define CW_STRAIGHT 0
#define CW_IAMBIC   1
#define CW_IAMBICB  2  
#define CW_KBD      3

int key_poll();
int key_poll2();
int get_cw_delay();
int get_data_delay();
int get_cw_input_method();
int get_data_delay();
int get_cw_tx_pitch();
int get_modem_pitch();
int get_wpm();
#define FT8_AUTO   2
#define FT8_SEMI   1
#define FT8_MANUAL 0
void ft8_setmode(int config);

void telnet_open(char *server);
int telnet_write(char *text);
void telnet_close();
double agc2(struct rx *r);
FILE *wav_start_writing(const char* path);

#define MULTICAST_ADDR "224.0.0.1"
#define MULTICAST_PORT 5005
#define MULTICAST_MAX_BUFFER_SIZE 1024

/* S-Meter */
int get_rx_gain(void);

/* Loopback play reset */
void sound_reset(int force);

#endif /* SDR_H_ */

