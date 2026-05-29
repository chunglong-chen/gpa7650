#ifndef WAVEFORM_H
#define	WAVEFORM_H

#ifdef	__cplusplus
extern "C" {
#endif

    void triangle(int edges, int ramp, int interval[2], uint16_t* wave, int repeat);
    void staircase(int levels, int tread, int interval[2], uint16_t* wave);
    void cosine_wave(double startv, double endv, int interval[2], uint16_t* wave, int* length);
    void trigger_array(int avgcnt, int linecnt, int trigger_start, int period, uint32_t* result_array, int* result_size);
    void gcc_x_wave(int edges, int ramp, int linecnt, int interval[2], uint16_t* wave);
    void gcc_y_wave(int edges, int ramp, int linecnt, int interval[2], uint16_t* wave);
    void radial_x_wave(int edges, int ramp, int linecnt, int interval[2], uint16_t* wave);
    void radial_y_wave(int edges, int ramp, int linecnt, int interval[2], uint16_t* wave);
#ifdef	__cplusplus
}
#endif

#endif	/* WAVEFORM_H */

