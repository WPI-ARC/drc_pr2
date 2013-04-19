#include <stdio.h>
#include <stdlib.h>
#include <omp.h>
#include "math.h"
#include "assert.h"
#include "task_control/libdtw.h"

using namespace std;

DTW::DTW(unsigned int x_size, unsigned int y_size, unsigned int point_size, double (*distance_eval)(int P_size, double * P1, double * P2))
{
    x_dim = x_size + 1;
    y_dim = y_size + 1;
    point_dim = point_size;
    DistanceFn = distance_eval;
    //Allocate the dtw matrix
    //Allocate the 3D dvoxel grid
    dtw_matrix = (double **) malloc(x_dim * sizeof(double *));
    //Safety check
    if (dtw_matrix == NULL)
    {
        return;
    }
    //Allocate the empty matrices
    for (unsigned int i = 0; i < x_dim; i++)
    {
        dtw_matrix[i] = (double *) malloc(y_dim * sizeof(double));
        //Safety check
        if (dtw_matrix[i] == NULL)
        {
            return;
        }
    }
    //Populate matrix with starting values
    dtw_matrix[0][0] = 0.0;
    for (unsigned int i = 1; i < x_dim; i++)
    {
        dtw_matrix[i][0] = INFINITY;
    }
    for (unsigned int i = 1; i < y_dim; i++)
    {
        dtw_matrix[0][i] = INFINITY;
    }
}

double DTW::EvaluateCost(unsigned int length_1, double ** sequence_1, unsigned int length_2, double ** sequence_2)
{
    //Safety checks
    assert(x_dim > length_1);
    assert(y_dim > length_2);
    //Compute DTW cost for the two sequences
    for (unsigned int i = 1; i <= length_1; i++)
    {
        for (unsigned int j = 1; j <= length_2; j++)
        {
            double index_cost = DistanceFn(point_dim, sequence_1[i - 1], sequence_2[j - 1]);
            double prev_cost;
            if (dtw_matrix[i - 1][j] < dtw_matrix[i - 1][j - 1] && dtw_matrix[i - j][j] < dtw_matrix[i][j - 1])
            {
                prev_cost = dtw_matrix[i - 1][j];
            }
            else if (dtw_matrix[i][j - 1] < dtw_matrix[i - 1][j] && dtw_matrix[i][j - 1] < dtw_matrix[i - 1][j - 1])
            {
                prev_cost = dtw_matrix[i][j - 1];
            }
            else
            {
                prev_cost = dtw_matrix[i - 1][j - 1];
            }
            dtw_matrix[i][j] = index_cost + prev_cost;
        }
    }
    //Return total path cost
    return dtw_matrix[length_1][length_2];
}

double DTW::EvaluateCost(std::vector< std::vector<double> > sequence_1, std::vector< std::vector<double> > sequence_2)
{
    //Safety checks
    assert(x_dim > sequence_1.size());
    assert(y_dim > sequence_2.size());
    assert(sequence_1[0].size() == sequence_2[0].size());
    //Compute DTW cost for the two sequences
    for (unsigned int i = 1; i <= sequence_1.size(); i++)
    {
        for (unsigned int j = 1; j <= sequence_2.size(); j++)
        {
            double index_cost = DistanceFn(point_dim, sequence_1[i - 1].data(), sequence_2[j - 1].data());
            double prev_cost;
            if (dtw_matrix[i - 1][j] < dtw_matrix[i - 1][j - 1] && dtw_matrix[i - j][j] < dtw_matrix[i][j - 1])
            {
                prev_cost = dtw_matrix[i - 1][j];
            }
            else if (dtw_matrix[i][j - 1] < dtw_matrix[i - 1][j] && dtw_matrix[i][j - 1] < dtw_matrix[i - 1][j - 1])
            {
                prev_cost = dtw_matrix[i][j - 1];
            }
            else
            {
                prev_cost = dtw_matrix[i - 1][j - 1];
            }
            dtw_matrix[i][j] = index_cost + prev_cost;
        }
    }
    //Return total path cost
    return dtw_matrix[sequence_1.size()][sequence_2.size()];
}

DTW::~DTW()
{
    //Clean up the dtw matrix
    for (unsigned int i = 0; i < x_dim; i++)
    {
        free(dtw_matrix[i]);
    }
    free(dtw_matrix);
}

ParallelDTW::ParallelDTW(unsigned int x_size, unsigned int y_size, unsigned int point_size, double (*distance_eval)(int P_size, double * P1, double * P2))
{
    x_dim = x_size + 1;
    y_dim = y_size + 1;
    point_dim = point_size;
    DistanceFn = distance_eval;
    //Allocate the dtw matrix
    dtw_matrix = (double **) malloc(x_dim * sizeof(double *));
    intermediate_matrix = (double **) malloc(x_dim * sizeof(double *));
    //Safety check
    if (dtw_matrix == NULL || intermediate_matrix == NULL)
    {
        return;
    }
    //Allocate the empty matrices
    for (unsigned int i = 0; i < x_dim; i++)
    {
        dtw_matrix[i] = (double *) malloc(y_dim * sizeof(double));
        intermediate_matrix[i] = (double *) malloc(y_dim * sizeof(double));
        //Safety check
        if (dtw_matrix[i] == NULL || intermediate_matrix[i] == NULL)
        {
            return;
        }
    }
    //Populate matrix with starting values
    dtw_matrix[0][0] = 0.0;
    intermediate_matrix[0][0] = 0.0;
    for (unsigned int i = 1; i < x_dim; i++)
    {
        dtw_matrix[i][0] = INFINITY;
        intermediate_matrix[i][0] = 0.0;
    }
    for (unsigned int i = 1; i < y_dim; i++)
    {
        dtw_matrix[0][i] = INFINITY;
        intermediate_matrix[0][i] = 0.0;
    }
}

double ParallelDTW::EvaluateCost(unsigned int length_1, double ** sequence_1, unsigned int length_2, double ** sequence_2)
{
    //Safety checks
    assert(x_dim > length_1);
    assert(y_dim > length_2);
    //Compute DTW cost for the two sequences
    #pragma omp parallel for
    for (unsigned int i = 1; i <= length_1; i++)
    {
        for (unsigned int j = 1; j <= length_2; j++)
        {
            intermediate_matrix[i][j] = DistanceFn(point_dim, sequence_1[i - 1], sequence_2[j - 1]);
        }
    }
    for (unsigned int i = 1; i <= length_1; i++)
    {
        for (unsigned int j = 1; j <= length_2; j++)
        {
            double prev_cost;
            if (dtw_matrix[i - 1][j] < dtw_matrix[i - 1][j - 1] && dtw_matrix[i - j][j] < dtw_matrix[i][j - 1])
            {
                prev_cost = dtw_matrix[i - 1][j];
            }
            else if (dtw_matrix[i][j - 1] < dtw_matrix[i - 1][j] && dtw_matrix[i][j - 1] < dtw_matrix[i - 1][j - 1])
            {
                prev_cost = dtw_matrix[i][j - 1];
            }
            else
            {
                prev_cost = dtw_matrix[i - 1][j - 1];
            }
            dtw_matrix[i][j] = intermediate_matrix[i][j] + prev_cost;
        }
    }
    //Return total path cost
    return dtw_matrix[length_1][length_2];
}

double ParallelDTW::EvaluateCost(std::vector< std::vector<double> > sequence_1, std::vector< std::vector<double> > sequence_2)
{
    //Safety checks
    assert(x_dim > sequence_1.size());
    assert(y_dim > sequence_2.size());
    assert(sequence_1[0].size() == sequence_2[0].size());
    //Compute DTW cost for the two sequences
    #pragma omp parallel for
    for (unsigned int i = 1; i <= sequence_1.size(); i++)
    {
        for (unsigned int j = 1; j <= sequence_2.size(); j++)
        {
            intermediate_matrix[i][j] = DistanceFn(point_dim, sequence_1[i - 1].data(), sequence_2[j - 1].data());
        }
    }
    for (unsigned int i = 1; i <= sequence_1.size(); i++)
    {
        for (unsigned int j = 1; j <= sequence_2.size(); j++)
        {
            double prev_cost;
            if (dtw_matrix[i - 1][j] < dtw_matrix[i - 1][j - 1] && dtw_matrix[i - j][j] < dtw_matrix[i][j - 1])
            {
                prev_cost = dtw_matrix[i - 1][j];
            }
            else if (dtw_matrix[i][j - 1] < dtw_matrix[i - 1][j] && dtw_matrix[i][j - 1] < dtw_matrix[i - 1][j - 1])
            {
                prev_cost = dtw_matrix[i][j - 1];
            }
            else
            {
                prev_cost = dtw_matrix[i - 1][j - 1];
            }
            dtw_matrix[i][j] = intermediate_matrix[i][j] + prev_cost;
        }
    }
    //Return total path cost
    return dtw_matrix[sequence_1.size()][sequence_2.size()];
}

ParallelDTW::~ParallelDTW()
{
    //Clean up the dtw matrix
    for (unsigned int i = 0; i < x_dim; i++)
    {
        free(dtw_matrix[i]);
        free(intermediate_matrix[i]);
    }
    free(dtw_matrix);
    free(intermediate_matrix);
}
