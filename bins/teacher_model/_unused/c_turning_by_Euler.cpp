#include <mex.h>
#include <cmath>

using namespace std;

void mexFunction( int            nlhs,    // Количество выходных аргументов
                  mxArray       *plhs[],  // Выходной массив аргументов
                  int            nrhs,    // Количество входных аргументов
                  const mxArray *prhs[]   // Входной массив аргументов
                 ) 
{
    if( nrhs != 2 ) {
        mexErrMsgTxt("Wrong arguments number!");
        return;
    } else {
        double *v = mxGetPr(prhs[0]);
        double *w = mxGetPr(prhs[1]);
        plhs[0] = mxCreateDoubleMatrix(3, 1, mxREAL);
        double *dv_dt = mxGetPr(plhs[0]);
        dv_dt[0] = w[1]*sin(v[2]) + w[2]*cos(v[2]);
        dv_dt[1] = (w[1]*cos(v[2]) - w[2]*sin(v[2]))/cos(v[0]);
        dv_dt[2] = w[0] - tan(v[0])*(w[1]*cos(v[2]) - w[2]*sin(v[2]));
        return;
    }
}