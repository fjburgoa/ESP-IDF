#include <math.h>
#include <dft.h>


#define N 150
#define PI 3.141592


void computeFFT(float signal[], float real[], float imag[]) {
    for (int k = 0; k < N / 2; ++k) { // Solo necesitamos calcular hasta N/2 debido a la simetría
        real[k] = 0.0;
        imag[k] = 0.0;
        for (int n = 0; n < N; ++n) {
            float angle = 2 * PI * k * n / N;
            real[k] += signal[n] * cos(angle);
            imag[k] -= signal[n] * sin(angle);
        }
    }
}
void findTopHarmonics(float real[], float imag[], int harmonics[], int numHarmonics, float weights[]) 
{
    float magnitudes[N / 2];
    float totalMagnitude = 0.0;

    for (int i = 0; i < N / 2; ++i) {
        magnitudes[i] = sqrt(real[i] * real[i] + imag[i] * imag[i]);
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
        magnitudes[maxIndex] = -1.0; // Remove this max from future consideration
    }
}
