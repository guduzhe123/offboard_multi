//
// Created by zhouhua on 2021/4/28.
//

#include "motion_plan/poly_traj/use_cuda.cuh"

void print_matrix(int R, int C, double* A, const char* name);

template<typename T>
void PrintEMatrix(const T &mat, const char *name) {
    std::cout << name << " =\n";
    std::cout << mat << std::endl;
}
//##############################################################################
template<typename T>
__global__
void Ker_CopyUpperSubmatrix(const T *__restrict d_in,
                            T *__restrict d_ou,
                            const int M, const int N, const int subM) {
    const int i = threadIdx.x + blockIdx.x*blockDim.x;
    const int j = threadIdx.y + blockIdx.y*blockDim.y;
    if (i>=subM || j>=N)
        return;
    d_ou[j*subM+i] = d_in[j*M+i];
}
//##############################################################################

void UseCuda::onInit() {

#define CUSOLVER_ERRCHK(x) \
        assert((x) == CUSOLVER_STATUS_SUCCESS && "cusolver failed");
#define CUBLAS_ERRCHK(x) \
        assert((x) == CUBLAS_STATUS_SUCCESS && "cublas failed");
    CUSOLVER_ERRCHK(cusolverDnCreate(&cusolverH));
    CUBLAS_ERRCHK(cublasCreate(&cublasH));
}

void UseCuda::calMatrixInverse(const Eigen::MatrixXd &A, Eigen::MatrixXd &x_sol, float &use_time) {
    typedef double T; // NOTE: don't change this. blas has different func name
    typedef Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic> MatrixXd;

    // define handles
    cusolverDnHandle_t cusolverH = NULL;
    cublasHandle_t cublasH = NULL;

    const int M = A.rows();
    const int N = A.cols();

    if (M != N) return;
    const int K = M;

    x_sol.resize(N,K);
//    x_ref = A.inverse();
    MatrixXd b = MatrixXd::Identity(N,K);

    float elapsed_time;
    cudaEvent_t start1, stop1;
    CHECK(cudaEventCreate(&start1));
    CHECK(cudaEventCreate(&stop1));
    CHECK(cudaEventRecord(start1));
    cudaEventQuery(start1);
/*    PrintEMatrix(A, "A");
    PrintEMatrix(b, "b");
/*    std::cout << "solution l1 error = " << (x_ref - C).norm()
              << std::endl;*/

#define CUSOLVER_ERRCHK(x) \
        assert((x) == CUSOLVER_STATUS_SUCCESS && "cusolver failed");
#define CUBLAS_ERRCHK(x) \
        assert((x) == CUBLAS_STATUS_SUCCESS && "cublas failed");

    CUSOLVER_ERRCHK(cusolverDnCreate(&cusolverH));
    CUBLAS_ERRCHK(cublasCreate(&cublasH));



    T *d_A, *d_b, *d_work, *d_work2, *d_tau;
    int *d_devInfo, devInfo;
    CHECK(cudaMalloc((void **) &d_A, sizeof(T) * M * N));
    CHECK(cudaMalloc((void **) &d_b, sizeof(T) * M * K));
    CHECK(cudaMalloc((void **) &d_tau, sizeof(T) * M));
    CHECK(cudaMalloc((void **) &d_devInfo, sizeof(int)));
    CHECK(cudaMemcpy(d_A, A.data(), sizeof(T) * M * N, cudaMemcpyHostToDevice));
    CHECK(cudaMemcpy(d_b, b.data(), sizeof(T) * M * K, cudaMemcpyHostToDevice));
    int bufSize, bufSize2;

    // in-place A = QR
    CUSOLVER_ERRCHK(
            cusolverDnDgeqrf_bufferSize(
                    cusolverH,
                    M,
                    N,
                    d_A,
                    M,
                    &bufSize
            )
    );
    CHECK(cudaMalloc((void **) &d_work, sizeof(T) * bufSize));
    CUSOLVER_ERRCHK(
            cusolverDnDgeqrf(
                    cusolverH,
                    M,
                    N,
                    d_A,
                    M,
                    d_tau,
                    d_work,
                    bufSize,
                    d_devInfo
            )
    );
    CHECK(cudaMemcpy(&devInfo, d_devInfo, sizeof(int),
                     cudaMemcpyDeviceToHost));
    assert(0 == devInfo && "QR factorization failed");

    // Q^T*b
    CUSOLVER_ERRCHK(
            cusolverDnDormqr_bufferSize(
                    cusolverH,
                    CUBLAS_SIDE_LEFT,
                    CUBLAS_OP_T,
                    M,
                    K,
                    N,
                    d_A,
                    M,
                    d_tau,
                    d_b,
                    M,
                    &bufSize2
            )
    );
    CHECK(cudaMalloc((void **) &d_work2, sizeof(T) * bufSize2));
    CUSOLVER_ERRCHK(
            cusolverDnDormqr(
                    cusolverH,
                    CUBLAS_SIDE_LEFT,
                    CUBLAS_OP_T,
                    M,
                    K,
                    min(M, N),
                    d_A,
                    M,
                    d_tau,
                    d_b,
                    M,
                    d_work2,
                    bufSize2,
                    d_devInfo
            )
    );
    CHECK(cudaDeviceSynchronize());
    CHECK(cudaMemcpy(&devInfo, d_devInfo, sizeof(int),
                     cudaMemcpyDeviceToHost));
    assert(0 == devInfo && "Q^T b failed");

    // need to explicitly copy submatrix for the triangular solve
    T *d_R, *d_b_;
    CHECK(cudaMalloc((void **) &d_R, sizeof(T) * N * N));
    CHECK(cudaMalloc((void **) &d_b_, sizeof(T) * N * K));
    dim3 thd_size(10, 10);
    dim3 blk_size((N + thd_size.x - 1) / thd_size.x, (N + thd_size.y - 1) / thd_size.y);
    Ker_CopyUpperSubmatrix<T><<<blk_size, thd_size>>>(d_A, d_R, M, N, N);
    blk_size = dim3((N + thd_size.x - 1) / thd_size.x, (K + thd_size.y - 1) / thd_size.y);
    Ker_CopyUpperSubmatrix<T><<<blk_size, thd_size>>>(d_b, d_b_, M, K, N);

    // solve x = R \ (Q^T*B)
    const double one = 1.0;
    CUBLAS_ERRCHK(
            cublasDtrsm(
                    cublasH,
                    CUBLAS_SIDE_LEFT,
                    CUBLAS_FILL_MODE_UPPER,
                    CUBLAS_OP_N,
                    CUBLAS_DIAG_NON_UNIT,
                    N,
                    K,
                    &one,
                    d_R,
                    N,
                    d_b_,
                    N
            )
    );
    CHECK(cudaDeviceSynchronize());

    CHECK(cudaMemcpy(x_sol.data(), d_b_, sizeof(T) * N * K,
                     cudaMemcpyDeviceToHost));

    CHECK(cudaEventRecord(stop1));
    CHECK(cudaEventSynchronize(stop1));
    CHECK(cudaEventElapsedTime(&elapsed_time, start1, stop1));
    use_time = elapsed_time;
    printf("Time2 = %g ms.\n", elapsed_time);

}

void UseCuda::calMatrixDgemm(const Eigen::MatrixXd &matrix_A, const Eigen::MatrixXd &matrix_B,
                             Eigen::MatrixXd &sol, float &time) {
    float elapsed_time;
    cudaEvent_t start1, stop1;
    CHECK(cudaEventCreate(&start1));
    CHECK(cudaEventCreate(&stop1));
    CHECK(cudaEventRecord(start1));
    cudaEventQuery(start1);

    typedef double T;
    int M = matrix_A.rows();
    int K = matrix_A.cols();
    int N = matrix_B.cols();
    int MK = M * K;
    int KN = K * N;
    int MN = M * N;

    double *h_A = (double *) malloc(sizeof(double) * MK);
    double *h_B = (double *) malloc(sizeof(double) * KN);
    double *h_C = (double *) malloc(sizeof(double) * MN);

    for (int i = 0; i < K; i++) {
        for (int j = 0; j < M; j++) {
            h_A[i * M + j] = matrix_A(j, i);
        }
    }

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < K; j++) {
            h_B[i * K + j] = matrix_B(j, i);
        }
    }

    for (int i = 0; i < MN; i++) {
        h_C[i] = 0;
    }

    double *g_A, *g_B, *g_C;
    CHECK(cudaMalloc((void **) &g_A, sizeof(double) * MK));
    CHECK(cudaMalloc((void **) &g_B, sizeof(double) * KN));
    CHECK(cudaMalloc((void **) &g_C, sizeof(double) * MN));

    cublasSetVector(MK, sizeof(double), h_A, 1, g_A, 1);
    cublasSetVector(KN, sizeof(double), h_B, 1, g_B, 1);
    cublasSetVector(MN, sizeof(double), h_C, 1, g_C, 1);

    cublasHandle_t handle;
    cublasCreate(&handle);
    double alpha = 1.0;
    double beta = 0.0;
    cublasDgemm(handle, CUBLAS_OP_N, CUBLAS_OP_N,
                M, N, K, &alpha, g_A, M, g_B, K, &beta, g_C, M);
    cublasDestroy(handle);

    cublasGetVector(MN, sizeof(double), g_C, 1, h_C, 1);

    sol = Eigen::MatrixXd::Zero(M, N);

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < M; j++) {
            sol(j, i) = h_C[i * M + j];
        }
    }
    free(h_A);
    free(h_B);
    free(h_C);
    CHECK(cudaFree(g_A));
    CHECK(cudaFree(g_B));
    CHECK(cudaFree(g_C));

    CHECK(cudaEventRecord(stop1));
    CHECK(cudaEventSynchronize(stop1));
    CHECK(cudaEventElapsedTime(&elapsed_time, start1, stop1));
    time = elapsed_time;
}

void print_matrix(int R, int C, double* A, const char* name)
{
    printf("%s = \n", name);
    for (int r = 0; r < R; ++r)
    {
        for (int c = 0; c < C; ++c)
        {
            printf("%10.6f", A[c * R + r]);
        }
        printf("\n");
    }
}