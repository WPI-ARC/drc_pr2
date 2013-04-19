#include "stdlib.h"
#include "stdio.h"
#include "stdint.h"

typedef struct d_voxel {
    float deformity;
    float cost;
    uint8_t R;
    uint8_t G;
    uint8_t B;
    uint8_t A;
} dVoxel;

typedef struct d_voxel_array {
    dVoxel ***dVoxels;
    int xDim;
    int yDim;
    int zDim;
    int xCenter;
    int yCenter;
    int zCenter;
    float defaultCost;
    float defaultDeformity;
} dVoxelArray;

dVoxelArray * AllocateDVoxelArray(int Xdim, int Ydim, int Zdim, int Xcenter, int Ycenter, int Zcenter, float defaultCost, float defaultDeformity);

int DestroyDVoxelArray(dVoxelArray * arrayToDestroy);

int WriteDVoxelArrayToFile(dVoxelArray * arrayToWrite, char* filename);

dVoxelArray * LoadDVoxelArrayFromFile(char* filename);

dVoxelArray * CloneDVoxelArray(dVoxelArray * arrayToClone);

double * RotatePoint(double X, double Y, double Z, double XR, double YR, double ZR, double XC, double YC, double ZC);

dVoxelArray * TransformMergeDVoxelArrays(dVoxelArray * world, dVoxelArray * object, float Xt, float Yt, float Zt, float Xr, float Yr, float Zr);

/** DEPRECATED AND REPLACED - DO NOT USE
dVoxelArray * RotateDVoxelArray(dVoxelArray * sourceArray, float Xr, float Yr, float Zr);

int MergeDVoxelArrays(dVoxelArray * mainArray, dVoxelArray * arrayToMerge, int Xs, int Ys,int Zs);
*/

int DVoxelEqual(dVoxel one, dVoxel two);
