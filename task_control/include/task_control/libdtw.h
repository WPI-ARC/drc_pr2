#include <stdlib.h>
#include <stdio.h>
#include <vector>

#ifndef DTW_H
#define DTW_H

using namespace std;

class DTW
{
private:
    double (*DistanceFn)(int P_size, double * P1, double * P2);
    double ** dtw_matrix;
    unsigned int x_dim;
    unsigned int y_dim;
    unsigned int point_dim;
public:
    DTW(unsigned int x_size, unsigned int y_size, unsigned int point_size, double (*distance_eval)(int P_size, double * P1, double * P2));
    ~DTW();
    double EvaluateCost(unsigned int length_1, double ** sequence_1, unsigned int length_2, double **sequence_2);
    double EvaluateCost(std::vector< std::vector<double> > sequence_1, std::vector< std::vector<double> > sequence_2);
};

class ParallelDTW
{
private:
    double (*DistanceFn)(int P_size, double * P1, double * P2);
    double ** dtw_matrix;
    double ** intermediate_matrix;
    unsigned int x_dim;
    unsigned int y_dim;
    unsigned int point_dim;
public:
    ParallelDTW(unsigned int x_size, unsigned int y_size, unsigned int point_size, double (*distance_eval)(int P_size, double * P1, double * P2));
    ~ParallelDTW();
    double EvaluateCost(unsigned int length_1, double ** sequence_1, unsigned int length_2, double **sequence_2);
    double EvaluateCost(std::vector< std::vector<double> > sequence_1, std::vector< std::vector<double> > sequence_2);
};

#endif // DTW_H
