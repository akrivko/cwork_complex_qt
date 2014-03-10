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
    KalmanFilter(vector<double> beginDeltaX){

        matrix<double> P(6,6);
        for (int i = 0; i < 6; ++i){
            for (int j = 0; j < 6; ++j){
                P(i,j) = 0;
            }
        }
        for (int i = 0; i < 3; ++i)
        {
            P(i,i) = 1e8;
            P(i+3,i+3) = 1e5;
        }

        estP = P;
    }



    vector<double> estimateDeltaX(vector<double> deltaX, vector<double> deltaY, matrix<double> F, matrix<double> H, matrix<double> D){
        estimateP(F,H,D);

        int numSat = H.size1();

        matrix<double> Dinv(numSat, numSat);
        vector<double> res(6);
        matrix<double> PH(6, numSat);
        matrix<double> PHDinv(numSat, numSat);
        vector<double> FdeltaX(6);
        matrix<double> HF(numSat, 6);
        vector<double> HFdeltaX(numSat);
        vector<double> deltaYHFdeltaX(numSat);

        FdeltaX = prod(F, deltaX);
        HF = prod(H,F);
        HFdeltaX = prod(HF, deltaX);
        deltaYHFdeltaX = deltaY - HFdeltaX;

        InvertMatrix(D, Dinv);

        PH = prod(estP, trans(H));
        PHDinv = prod(PH, Dinv);

        //res = FdeltaX + prod(PHDinv, deltaYHFdeltaX);
        res = prod(PHDinv, deltaY);
        return res;
    }

    matrix<double> getEstP(){
        return estP;
    }


private:
    vector<double> estDeltaX;
    matrix<double> estP;


    void estimateP(matrix<double> F, matrix<double> H, matrix<double> D){
        int numSat = H.size1();
        for (int i=0; i<6; ++i){
            std::cout<< estP(i,i)<< std::endl;
        }

        matrix<double> P(6,6);
        matrix<double> FP(6,6);
        matrix<double> FPF(6,6);
        matrix<double> FPFinv(6,6);
        matrix<double> Dinv(numSat, numSat);
        matrix<double> HDinv(6, numSat);
        matrix<double> HDinvH(6,6);
        matrix<double> FPFinvHDinvH(6,6);

        FP = prod(F, estP);
        FPF = prod(FP, trans(F));
        InvertMatrix(FPF, FPFinv);
        InvertMatrix(D, Dinv);
        HDinv = prod(trans(H), Dinv);
        HDinvH = prod(HDinv, H);
        FPFinvHDinvH = FPFinv + HDinvH;
        InvertMatrix(FPFinvHDinvH, P);
        estP = P;

        for (int i=0; i<6; ++i){
            estP(i,i) = estP(i,i) * 1.0;
        }

    }

};


#endif
