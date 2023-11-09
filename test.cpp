#include <DBase.hpp>

_D_USING_BASE

c_AnkSpline cAnkL(0.004), cAnkR(0.004);
FILE *fpt;
int main() {
    cAnkL.fnvUpdateAnkTra(0.4, {0.0, 0.0, 0.05}, {0.02, -0.02, 0.05}, 0.04);
    cAnkR.fnvUpdateAnkTra(0.4, {0.0, 0.0, 0.05}, {-0.02, 0.02, 0.15}, 0.04);
    fpt = fopen("ankDat.dat", "w");
    double *dAnkLX = cAnkL.GetTraX(), *dAnkLY = cAnkL.GetTraY(), *dAnkLZ = cAnkL.GetTraZ(); 
    double *dAnkRX = cAnkR.GetTraX(), *dAnkRY = cAnkR.GetTraY(), *dAnkRZ = cAnkR.GetTraZ(); 
    fprintf(fpt, "dAnkLX\tdAnkLY\tdAnkLZ\tdAnkRX\tdAnkRY\tdAnkRZ\n");
    for(int i = 0; i < 400; i++) {
        fprintf(fpt, "%.6lf\t%.6lf\t%.6lf\t", dAnkLX[i], dAnkLY[i], dAnkLZ[i]);
        fprintf(fpt, "%.6lf\t%.6lf\t%.6lf\n", dAnkRX[i], dAnkRY[i], dAnkRZ[i]);
    }
    fclose(fpt);
    return true;
}