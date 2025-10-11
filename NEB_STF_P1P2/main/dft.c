#include <stdio.h>
#include <math.h>
#include "dft.h"

#define PI 3.141592


int harmonics[10];               //número de armónico
float weights[10];               //peso del armónico
Complex signal[N];               //señal medida y procesada


void fft(Complex *x, int n) 
{
    if (n <= 1) return;

    Complex *even = (Complex *)malloc(n / 2 * sizeof(Complex));
    Complex *odd  = (Complex *)malloc(n / 2 * sizeof(Complex));

    for (int i = 0; i < n / 2; ++i) {
        even[i] = x[i * 2];
        odd[i] = x[i * 2 + 1];
    }

    fft(even, n / 2);                         //recursivo
    fft(odd, n / 2);                          //recursivo  

    for (int i = 0; i < n / 2; ++i) {
        float t = 2 * PI * i / n;
        Complex twiddle = {cos(t), -sin(t)};
        Complex temp = {twiddle.real * odd[i].real - twiddle.imag * odd[i].imag,
                        twiddle.real * odd[i].imag + twiddle.imag * odd[i].real};

        x[i].real = even[i].real + temp.real;
        x[i].imag = even[i].imag + temp.imag;
        x[i + n / 2].real = even[i].real - temp.real;
        x[i + n / 2].imag = even[i].imag - temp.imag;
    }

    free(even);
    free(odd);
}

void Harmonics(Complex *x, int *harmonic_nr, float *magnitude, float *relative_weight)
{
    float totalMagnitude = 0.0;

    for (int i = 0; i < N / 2; i++) 
    {
        harmonic_nr[i] = i;
        magnitude[i] = sqrt(x[i].real * x[i].real + x[i].imag * x[i].imag);
        totalMagnitude += magnitude[i];
    }

    for (int i = 0; i < N / 2 ; i++)
    { 
        relative_weight[i] = magnitude[i] / totalMagnitude * 100.0; // Peso relativo en porcentaje
    }

}



void findTopHarmonics(Complex *x, int harmonics[], int numHarmonics, float weights[]) 
{
    float magnitudes[N / 2];
    float totalMagnitude = 0.0;

    for (int i = 0; i < N / 2; ++i) {
        magnitudes[i] = sqrt(x[i].real * x[i].real + x[i].imag * x[i].imag);
        totalMagnitude += magnitudes[i];
    }

    for (int i = 0; i < numHarmonics; ++i) {
        int maxIndex = 0;
        for (int j = 1; j < N / 2; ++j) {
            if (magnitudes[j] > magnitudes[maxIndex]) {
                maxIndex = j;
            }
        }
        harmonics[i] = maxIndex;
        weights[i] = magnitudes[maxIndex] / totalMagnitude * 100.0; // Peso relativo en porcentaje
        magnitudes[maxIndex] = -1.0;                                 // Remove this max from future consideration
    }
}
