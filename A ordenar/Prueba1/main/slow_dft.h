#ifndef _DFT_H
#define _DFT_H

#define N 1000 // Número de datos




void computeFFT(float signal[], float real[], float imag[]);
void findTopHarmonics(float real[], float imag[], int harmonics[], int numHarmonics, float weights[]) ;

#endif