#include <omp.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "math.h"
#include "task_control/libdtw.h"
#include "task_control/libtrajectory.h"

using namespace std;

double euclidean_distance(int P_size, double * P1, double * P2)
{
    double total = 0.0;
    for (int i = 0; i < P_size; i++)
    {
        total = total + pow((P1[i] - P2[i]), 2);
    }
    return sqrt(total);
}

int omp_test(void)
{
    printf("Testing OpenMP...\n");
    int th_id, nthreads;
    #pragma omp parallel private(th_id)
    {
        th_id = omp_get_thread_num();
        printf("Hello World from thread %d\n", th_id);
        #pragma omp barrier
        if (th_id == 0)
        {
            nthreads = omp_get_num_threads();
            printf("There are %d threads\n",nthreads);
        }
    }
    return nthreads;
}

int dtw_test(void)
{
    struct timespec bst, bet, pst, pet;
    printf("Building test arrays\n");
    double ** test_seq_1 = (double **)malloc(100 * sizeof(double*));
    for (int i = 0; i < 100; i++)
    {
        test_seq_1[i] = (double *)malloc(3 * sizeof(double));
        test_seq_1[i][0] = 0.0;
        test_seq_1[i][1] = 0.0;
        test_seq_1[i][2] = 0.0;
    }
    std::vector< std::vector<double> > test_vec_1;
    for (int i = 0; i < 100; i++)
    {
        std::vector<double> state;
        state.push_back(0.0);
        state.push_back(0.0);
        state.push_back(0.0);
        test_vec_1.push_back(state);
    }
    printf("Building evaluator\n");
    DTW my_eval = DTW(100, 100, 3, euclidean_distance);
    printf("Evaluating\n");
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &bst);
    double cost1 = my_eval.EvaluateCost(test_vec_1, test_vec_1);
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &bet);
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &pst);
    double cost2 = my_eval.EvaluateCost(100, test_seq_1, 100, test_seq_1);
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &pet);
    float bsecs = (float)(bet.tv_sec - bst.tv_sec);
    bsecs = bsecs + (float)(bet.tv_nsec - bst.tv_nsec) / 1000000000.0;
    float psecs = (float)(pet.tv_sec - pst.tv_sec);
    psecs = psecs + (float)(pet.tv_nsec - pst.tv_nsec) / 1000000000.0;
    printf("Evaluation complete with array ptr cost %f and vector cost %f, requiring %f seconds / %f seconds\n", cost2, cost1, psecs, bsecs);
    return 0;
}

int parallel_dtw_test(void)
{
    struct timespec bst, bet, pst, pet;
    printf("Building test arrays\n");
    double ** test_seq_1 = (double **)malloc(100 * sizeof(double*));
    for (int i = 0; i < 100; i++)
    {
        test_seq_1[i] = (double *)malloc(3 * sizeof(double));
        test_seq_1[i][0] = 0.0;
        test_seq_1[i][1] = 0.0;
        test_seq_1[i][2] = 0.0;
    }
    std::vector< std::vector<double> > test_vec_1;
    for (int i = 0; i < 100; i++)
    {
        std::vector<double> state;
        state.push_back(0.0);
        state.push_back(0.0);
        state.push_back(0.0);
        test_vec_1.push_back(state);
    }
    printf("Building evaluator\n");
    ParallelDTW my_eval = ParallelDTW(100, 100, 3, euclidean_distance);
    printf("Evaluating\n");
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &bst);
    double cost1 = my_eval.EvaluateCost(test_vec_1, test_vec_1);
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &bet);
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &pst);
    double cost2 = my_eval.EvaluateCost(100, test_seq_1, 100, test_seq_1);
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &pet);
    float bsecs = (float)(bet.tv_sec - bst.tv_sec);
    bsecs = bsecs + (float)(bet.tv_nsec - bst.tv_nsec) / 1000000000.0;
    float psecs = (float)(pet.tv_sec - pst.tv_sec);
    psecs = psecs + (float)(pet.tv_nsec - pst.tv_nsec) / 1000000000.0;
    printf("Parallel evaluation complete with array ptr cost %f and vector cost %f, requiring %f seconds / %f seconds\n", cost2, cost1, psecs, bsecs);
    return 0;
}

int measure_performance(int iterations)
{
    struct timespec bsta, beta, bstv, betv, psta, peta, pstv, petv;
    printf("Building test arrays\n");
    double ** test_seq_1 = (double **)malloc(100 * sizeof(double*));
    for (int i = 0; i < 100; i++)
    {
        test_seq_1[i] = (double *)malloc(3 * sizeof(double));
        test_seq_1[i][0] = 0.0;
        test_seq_1[i][1] = 0.0;
        test_seq_1[i][2] = 0.0;
    }
    std::vector< std::vector<double> > test_vec_1;
    for (int i = 0; i < 100; i++)
    {
        std::vector<double> state;
        state.push_back(0.0);
        state.push_back(0.0);
        state.push_back(0.0);
        test_vec_1.push_back(state);
    }
    DTW my_eval = DTW(100, 100, 3, euclidean_distance);
    ParallelDTW par_eval = ParallelDTW(100, 100, 3, euclidean_distance);
    printf("Evaluating\n");
    //Run tests
    for (int i = 0; i < iterations; i++)
    {
        //-----Test single-threaded version------------
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &bstv);
        double scost1 = my_eval.EvaluateCost(test_vec_1, test_vec_1);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &betv);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &bsta);
        double scost2 = my_eval.EvaluateCost(100, test_seq_1, 100, test_seq_1);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &beta);
        //---------------------------------------------
        //-----Test multi-threaded version-------------
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &pstv);
        double pcost1 = par_eval.EvaluateCost(test_vec_1, test_vec_1);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &petv);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &psta);
        double pcost2 = par_eval.EvaluateCost(100, test_seq_1, 100, test_seq_1);
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &peta);
        //---------------------------------------------
        //-----Compute runtimes (single-threaded)--------;
        float bsecsv = (float)(betv.tv_sec - bstv.tv_sec);
        bsecsv = bsecsv + (float)(betv.tv_nsec - bstv.tv_nsec) / 1000000000.0;
        float bsecsa = (float)(beta.tv_sec - bsta.tv_sec);
        bsecsa = bsecsa + (float)(beta.tv_nsec - bsta.tv_nsec) / 1000000000.0;
        //-----Compute runtimes (multi-threaded)---------;
        float psecsv = (float)(petv.tv_sec - pstv.tv_sec);
        psecsv = psecsv + (float)(petv.tv_nsec - pstv.tv_nsec) / 1000000000.0;
        float psecsa = (float)(peta.tv_sec - psta.tv_sec);
        psecsa = psecsa + (float)(peta.tv_nsec - psta.tv_nsec) / 1000000000.0;
        printf("SINGLE (array) | SINGLE (vector) | PARALLEL (array) | PARALLEL (vector)\n%f | %f | %f | %f\n", bsecsa, bsecsv, psecsa, psecsv);
    }
    return 0;
}

int compare_performance(int traj_length, int iterations)
{
    struct timespec bsta, beta, bstv, betv, psta, peta, pstv, petv;
    printf("Building test arrays\n");
    double ** test_seq_1 = (double **)malloc(traj_length * sizeof(double*));
    for (int i = 0; i < traj_length; i++)
    {
        test_seq_1[i] = (double *)malloc(3 * sizeof(double));
        test_seq_1[i][0] = 0.0;
        test_seq_1[i][1] = 0.0;
        test_seq_1[i][2] = 0.0;
    }
    std::vector< std::vector<double> > test_vec_1;
    for (int i = 0; i < traj_length; i++)
    {
        std::vector<double> state;
        state.push_back(0.0);
        state.push_back(0.0);
        state.push_back(0.0);
        test_vec_1.push_back(state);
    }
    double *** test_seq_2 = (double ***)malloc(iterations * sizeof(double**));
    for (int i = 0; i < iterations; i++)
    {
        test_seq_2[i] = (double **)malloc(traj_length * sizeof(double*));
        for (int j = 0; j < traj_length; j++)
        {
            test_seq_2[i][j] = (double *)malloc(3 * sizeof(double));
            test_seq_2[i][j][0] = (double)rand();
            test_seq_2[i][j][1] = (double)rand();
            test_seq_2[i][j][2] = (double)rand();
        }
    }
    std::vector< std::vector< std::vector<double> > > test_vec_2;
    for (int i = 0; i < iterations; i++)
    {
        std::vector< std::vector<double> > traj;
        for (int j = 0; j < traj_length; j++)
        {
            std::vector<double> state2;
            state2.push_back((double)rand());
            state2.push_back((double)rand());
            state2.push_back((double)rand());
            traj.push_back(state2);
        }
        test_vec_2.push_back(traj);
    }
    DTW my_eval = DTW(traj_length, traj_length, 3, euclidean_distance);
    ParallelDTW par_eval = ParallelDTW(traj_length, traj_length, 3, euclidean_distance);
    printf("Evaluating\n");
    //Run tests
    printf("-----Test single-threaded version-----\n");
    printf("Testing vector variant\n");
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &bstv);
    for (int i = 0; i < iterations; i++)
    {
        double scost1 = my_eval.EvaluateCost(test_vec_1, test_vec_2[i]);
    }
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &betv);
    printf("Testing array variant\n");
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &bsta);
    for (int i = 0; i < iterations; i++)
    {
        double scost2 = my_eval.EvaluateCost(traj_length, test_seq_1, traj_length, test_seq_2[i]);
    }
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &beta);
    //---------------------------------------------
    printf("-----Test multi-threaded version-----\n");
    printf("Testing vector variant\n");
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &pstv);
    for (int i = 0; i < iterations; i++)
    {
        double pcost1 = par_eval.EvaluateCost(test_vec_1, test_vec_2[i]);
    }
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &petv);
    printf("Testing array variant\n");
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &psta);
    for (int i = 0; i < iterations; i++)
    {
        double pcost2 = par_eval.EvaluateCost(traj_length, test_seq_1, traj_length, test_seq_2[i]);
    }
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &peta);
    //---------------------------------------------
    //-----Compute runtimes (single-threaded)--------;
    float bsecsv = (float)(betv.tv_sec - bstv.tv_sec);
    bsecsv = bsecsv + (float)(betv.tv_nsec - bstv.tv_nsec) / 1000000000.0;
    float bsecsa = (float)(beta.tv_sec - bsta.tv_sec);
    bsecsa = bsecsa + (float)(beta.tv_nsec - bsta.tv_nsec) / 1000000000.0;
    //-----Compute runtimes (multi-threaded)---------;
    float psecsv = (float)(petv.tv_sec - pstv.tv_sec);
    psecsv = psecsv + (float)(petv.tv_nsec - pstv.tv_nsec) / 1000000000.0;
    float psecsa = (float)(peta.tv_sec - psta.tv_sec);
    psecsa = psecsa + (float)(peta.tv_nsec - psta.tv_nsec) / 1000000000.0;
    printf("SINGLE (array) | SINGLE (vector) | PARALLEL (array) | PARALLEL (vector)\n%f | %f | %f | %f\n", bsecsa, bsecsv, psecsa, psecsv);
    return 0;
}

int main()
{
    //omp_test();
    //dtw_test();
    //parallel_dtw_test();
    compare_performance(100, 1000);
    //dtw_test();
    //parallel_dtw_test();
}
