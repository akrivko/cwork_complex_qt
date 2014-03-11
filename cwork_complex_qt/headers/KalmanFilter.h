#ifndef _KALMAN_FILTER_
#define _KALMAN_FILTER_

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>

namespace ublas = boost::numeric::ublas;


bool InvertMatrix(const ublas::matrix<double>& input, ublas::matrix<double>& inverse) {
    using namespace boost::numeric::ublas;
    //matrix<double> inverse(input.size1(), input.size2());
    typedef permutation_matrix<double> pmatrix;
    // create a working copy of the input
    matrix<double> A(input);
    // create a permutation matrix for the LU-factorization
    pmatrix pm(A.size1());
    // perform LU-factorization
    int res = lu_factorize(A,pm);
    if( res != 0 ) return false;
    // create identity matrix of "inverse"
    inverse.assign(ublas::identity_matrix<double>(A.size1()));
    // backsubstitute to get the inverse
    lu_substitute(A, pm, inverse);
    return true;
};



class KalmanFilter{
public:    
    matrix<double> estP;

    KalmanFilter(matrix<double> initP){
        estP = initP;
    }

    vector<double> estimateDeltaX(vector<double> deltaX, vector<double> deltaY, matrix<double> F, matrix<double> H, matrix<double> D){
      //  estimateP(F,H,D);

        int numSat = H.size1();
        int numComponent = deltaX.size();

        matrix<double> Dinv(numSat, numSat);
        vector<double> res(numComponent);
        matrix<double> PH(numComponent, numSat);
        matrix<double> PHDinv(numSat, numSat);
        vector<double> FdeltaX(numComponent);
        matrix<double> HF(numSat, numComponent);
        vector<double> HFdeltaX(numSat);
        vector<double> deltaYHFdeltaX(numSat);

        InvertMatrix(D, Dinv);

        PH = prod(estP, trans(H));
        PHDinv = prod(PH, Dinv);

        FdeltaX = prod(F, deltaX);
        HF = prod(H,F);
        HFdeltaX = prod(HF, deltaX);
        deltaYHFdeltaX = deltaY - HFdeltaX;

        //res = FdeltaX + prod(PHDinv, deltaYHFdeltaX);

        res = prod(PHDinv, deltaY);

        return res;
    }

    matrix<double> getEstP(){
        return estP;
    }


    void estimateP(matrix<double> F, matrix<double> H, matrix<double> D){
        int numSat = H.size1();
        int numComponent = F.size1();

        for (int i=0; i<numComponent; ++i){
            std::cout<< estP(i,i)<< std::endl;
        }

        matrix<double> P(numComponent,numComponent);
        matrix<double> FP(numComponent,numComponent);
        matrix<double> FPF(numComponent,numComponent);
        matrix<double> FPFinv(numComponent,numComponent);
        matrix<double> Dinv(numSat, numSat);
        matrix<double> HDinv(numComponent, numSat);
        matrix<double> HDinvH(numComponent,numComponent);
        matrix<double> FPFinvHDinvH(numComponent,numComponent);

        FP = prod(F, estP);
        FPF = prod(FP, trans(F));
        InvertMatrix(FPF, FPFinv);
        InvertMatrix(D, Dinv);
        HDinv = prod(trans(H), Dinv);
        HDinvH = prod(HDinv, H);
        FPFinvHDinvH = FPFinv + HDinvH;
        InvertMatrix(FPFinvHDinvH, P);
        estP = P;


    }

};


#endif
