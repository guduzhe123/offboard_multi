//
// Created by zhouhua on 2021/4/28.
//

#ifndef OFFBOARD_USE_CUDA_CUH
#define OFFBOARD_USE_CUDA_CUH
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>
#include "cuda_runtime.h"
#include "cublas_v2.h"
#include "cusolverDn.h"
#include "Eigen/Dense"
#include "error.cuh"



class UseCuda {
public:
    void onInit();
    void calMatrixInverse(const Eigen::MatrixXd &A, Eigen::MatrixXd &x_sol, float &use_time);

private:

    // define handles
    cusolverDnHandle_t cusolverH = NULL;
    cublasHandle_t cublasH = NULL;
};
#endif //OFFBOARD_USE_CUDA_CUH
