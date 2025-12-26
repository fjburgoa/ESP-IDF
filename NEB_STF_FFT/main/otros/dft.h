#ifndef _DFT_H
#define _DFT_H

#define N 1000 // Número de datos


typedef struct {
    float real;
    float imag;
} Complex;

extern int harmonics[10];               //número de armónico
extern  float weights[10];              //peso del armónico
extern Complex signal[N];               //señal medida y procesada


void fft(Complex *x, int n);
void findTopHarmonics(Complex *x, int harmonics[], int numHarmonics, float weights[]) ;
void Harmonics(Complex *x, int *harmonic_nr, float *magnitude, float *relative_weight);

#endif