#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "task_control/libtrajectory.h"

#define snap(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

dVoxelArray * AllocateDVoxelArray(int Xdim, int Ydim, int Zdim, int Xcenter, int Ycenter, int Zcenter, float defaultCost, float defaultDeformity)
{
    //Allocates a dVoxelArray with the given parameters
    dVoxelArray *new_dvoxel_array = (dVoxelArray*) malloc(sizeof(dVoxelArray));
    if (new_dvoxel_array == NULL)
    {
        return NULL;
    }
    //printf("Allocation start\n");
    new_dvoxel_array->xDim = Xdim;
    new_dvoxel_array->yDim = Ydim;
    new_dvoxel_array->zDim = Zdim;
    new_dvoxel_array->xCenter = Xcenter;
    new_dvoxel_array->yCenter = Ycenter;
    new_dvoxel_array->zCenter = Zcenter;
    new_dvoxel_array->defaultCost = defaultCost;
    new_dvoxel_array->defaultDeformity = defaultDeformity;
    //printf("Array allocation start\n");
    //Allocate the 3D dvoxel grid
    new_dvoxel_array->dVoxels = (dVoxel***) malloc(Xdim * sizeof(dVoxel**));
    //Safety check
    if (new_dvoxel_array->dVoxels == NULL)
    {
        return NULL;
    }
    //printf("Multidim allocation start\n");
    for (int i = 0; i < Xdim; i++)
    {
        new_dvoxel_array->dVoxels[i] = (dVoxel**) malloc(Ydim * sizeof(dVoxel*));
        //Safety check
        if (new_dvoxel_array->dVoxels[i] == NULL)
        {
            return NULL;
        }
        for (int j = 0; j < Ydim; j++)
        {
            new_dvoxel_array->dVoxels[i][j] = (dVoxel*) malloc(Zdim * sizeof(dVoxel));
            if (new_dvoxel_array->dVoxels[i][j] == NULL)
            {
                return NULL;
            }
            //Populate individual dvoxel
            for (int k = 0; k < Zdim; k++)
            {
                new_dvoxel_array->dVoxels[i][j][k].deformity = defaultDeformity;
                new_dvoxel_array->dVoxels[i][j][k].cost = defaultCost;
                new_dvoxel_array->dVoxels[i][j][k].R = 0x00;
                new_dvoxel_array->dVoxels[i][j][k].G = 0x00;
                new_dvoxel_array->dVoxels[i][j][k].B = 0x00;
                new_dvoxel_array->dVoxels[i][j][k].A = 0x00;
            }
        }
    }
    //printf("Ready to return assembled array\n");
    return new_dvoxel_array;
}

int DestroyDVoxelArray(dVoxelArray * arrayToDestroy)
{
    //Deallocates a dVoxelArray
    int Xdim = arrayToDestroy->xDim;
    int Ydim = arrayToDestroy->yDim;
    for (int i = 0; i < Xdim; i++)
    {
        for (int j = 0; j < Ydim; j++)
        {
            //printf("Free level 2\n");
            free(arrayToDestroy->dVoxels[i][j]);
        }
        //printf("Free level 1\n");
        free(arrayToDestroy->dVoxels[i]);
    }
    //printf("Free level 0\n");
    free(arrayToDestroy->dVoxels);
    //printf("Free remainder\n");
    free(arrayToDestroy);
    return 0;
}

int WriteDVoxelArrayToFile(dVoxelArray * arrayToWrite, char* filename)
{
    FILE *array_file = fopen(filename, "w");
    //Write file header
    fprintf(array_file, "D-Voxel Array Storage File\nSize: %d|%d|%d\nCenter: %d|%d|%d\nDefaults: %f|%f\nDATA", arrayToWrite->xDim, arrayToWrite->yDim, arrayToWrite->zDim, arrayToWrite->xCenter, arrayToWrite->yCenter, arrayToWrite->zCenter, arrayToWrite->defaultCost, arrayToWrite->defaultDeformity);
    //Write file data
    for (int i = 0; i < arrayToWrite->xDim; i++)
    {
        for (int j = 0; j < arrayToWrite->yDim; j++)
        {
            for (int k = 0; k < arrayToWrite->zDim; k++)
            {
                dVoxel temp = arrayToWrite->dVoxels[i][j][k];
                fwrite(&temp, sizeof(dVoxel), 1, array_file);
            }
        }
    }
    //Flush & close file
    return fclose(array_file);
}

dVoxelArray * LoadDVoxelArrayFromFile(char* filename)
{
    FILE *array_file = fopen(filename, "r");
    if (array_file == NULL)
    {
        printf("*** Unable to find DVXL file! ***\n");
        fflush(stdout);
        exit(1);
    }
    //Read file header
    int Xdim, Ydim, Zdim, Xcenter, Ycenter, Zcenter;
    float defCost, defDeform;
    int read = fscanf(array_file, "D-Voxel Array Storage File\nSize: %d|%d|%d\nCenter: %d|%d|%d\nDefaults: %f|%f\nDATA", &Xdim, &Ydim, &Zdim, &Xcenter, &Ycenter, &Zcenter, &defCost, &defDeform);
    if (read != 8)
    {
        printf("*** Broken or corrupted DVXL file! - Check the file manually to make sure the header is correct :: Got %d parameters, expected %d! ***\n", read, 8);
        fflush(stdout);
        exit(1);
    }
    //Make an empty DVOXEL array
    dVoxelArray *new_dvoxel_array = AllocateDVoxelArray(Xdim, Ydim, Zdim, Xcenter, Ycenter, Zcenter, defCost, defDeform);
    //Safety check
    if (new_dvoxel_array == NULL)
    {
        return NULL;
    }
    //Read file data
    char * temp_read = (char *) malloc(sizeof(dVoxel));
    for (int i = 0; i < Xdim; i++)
    {
        for (int j = 0; j < Ydim; j++)
        {
            for (int k = 0; k < Zdim; k++)
            {
                int read = fread(temp_read, sizeof(dVoxel), 1, array_file);
                if (read != sizeof(dVoxel))
                {
                    //printf("*** Broken or corrupted DVXL file! - Check the file manually to make sure the header is correct :: Current index: [%d,%d,%d] :: Read %d bytes [%s], expected %d! ***\n", i, j, k, read, temp_read, sizeof(dVoxel));
                    //fflush(stdout);
                    //exit(1);
                }
                new_dvoxel_array->dVoxels[i][j][k].deformity = *( (float*)(temp_read) );
                new_dvoxel_array->dVoxels[i][j][k].cost = *( (float*)(temp_read + 4) );
                new_dvoxel_array->dVoxels[i][j][k].R = temp_read[8];
                new_dvoxel_array->dVoxels[i][j][k].G = temp_read[9];
                new_dvoxel_array->dVoxels[i][j][k].B = temp_read[10];
                new_dvoxel_array->dVoxels[i][j][k].A = temp_read[11];
            }
        }
    }
    //Flush & close file
    fclose(array_file);
    free(temp_read);
    //Return data
    return new_dvoxel_array;
}

dVoxelArray * CloneDVoxelArray(dVoxelArray * arrayToClone)
{
    //Make an empty DVOXEL array
    dVoxelArray *new_dvoxel_array = AllocateDVoxelArray(arrayToClone->xDim, arrayToClone->yDim, arrayToClone->zDim, arrayToClone->xCenter, arrayToClone->yCenter, arrayToClone->zCenter, arrayToClone->defaultCost, arrayToClone->defaultDeformity);
    //Safety check
    if (new_dvoxel_array == NULL)
    {
        return NULL;
    }
    //Copy data
    for (int i = 0; i < (arrayToClone->xDim); i++)
    {
        for (int j = 0; j < (arrayToClone->yDim); j++)
        {
            for (int k = 0; k < (arrayToClone->zDim); k++)
            {
                new_dvoxel_array->dVoxels[i][j][k].deformity = arrayToClone->dVoxels[i][j][k].deformity;
                new_dvoxel_array->dVoxels[i][j][k].cost = arrayToClone->dVoxels[i][j][k].cost;
                new_dvoxel_array->dVoxels[i][j][k].R = arrayToClone->dVoxels[i][j][k].R;
                new_dvoxel_array->dVoxels[i][j][k].G = arrayToClone->dVoxels[i][j][k].G;
                new_dvoxel_array->dVoxels[i][j][k].B = arrayToClone->dVoxels[i][j][k].B;
                new_dvoxel_array->dVoxels[i][j][k].A = arrayToClone->dVoxels[i][j][k].A;
            }
        }
    }
    //Return data
    return new_dvoxel_array;
}

double * RotatePoint(double X, double Y, double Z, double XR, double YR, double ZR, double XC, double YC, double ZC)
{
    double * rotated = (double *)malloc(sizeof(double) * 3);
    if ((XR == 0.0 || XR == -0.0) && (YR == 0.0 || YR == -0.0) && (ZR == 0.0 || ZR == -0.0))
    {
        rotated[0] = X;
        rotated[1] = Y;
        rotated[2] = Z;
        return rotated;
    }
    //X-axis rotation changes Y and Z coords
    double x1 = X;
    double y1 = YC + (cos(XR) * (Y - YC) - sin(XR) * (Z - ZC));
    double z1 = ZC + (sin(XR) * (Y - YC) + cos(XR) * (Z - ZC));
    //Y-axis rotation changes X and Z coords
    double x2 = XC + (cos(YR) * (x1 - XC) + sin(YR) * (z1 - ZC));
    double y2 = y1;
    double z2 = ZC + (-sin(YR) * (x1 - XC) + cos(YR) * (z1 - ZC));
    //Z-axis rotation changes X and Y coords
    double x3 = XC + (cos(ZR) * (x2 - XC) - sin(ZR) * (y2 - YC));
    double y3 = YC + (sin(ZR) * (x2 - XC) + cos(ZR) * (y2 - YC));
    double z3 = z2;
    rotated[0] = x3;
    rotated[1] = y3;
    rotated[2] = z3;
    return rotated;
}

dVoxelArray * TransformMergeDVoxelArrays(dVoxelArray * world, dVoxelArray * object, float Xt, float Yt, float Zt, float Xr, float Yr, float Zr)
{
    double new_center_x = Xt;
    double new_center_y = Yt;
    double new_center_z = Zt;
    for (int i = 0; i < object->xDim; i++)
    {
        for (int j = 0; j < object->yDim; j++)
        {
            for (int k = 0; k < object->zDim; k++)
            {
                //Find the translated absolute position of the current index
                float Xoffset = (float)(i - object->xCenter);
                float Yoffset = (float)(j - object->yCenter);
                float Zoffset = (float)(k - object->zCenter);
                //Compute the rotated position of the current point
                double * rotated_point = RotatePoint(Xoffset, Yoffset, Zoffset, Xr, Yr, Zr, 0.0, 0.0, 0.0);
                rotated_point[0] = rotated_point[0] + new_center_x;
                rotated_point[1] = rotated_point[1] + new_center_y;
                rotated_point[2] = rotated_point[2] + new_center_z;
                //int new_x = snap(rotated_point[0]);
                //int new_y = snap(rotated_point[1]);
                //int new_z = snap(rotated_point[2]);
                int new_x = snap(new_center_x + Xoffset);
                int new_y = snap(new_center_y + Yoffset);
                int new_z = snap(new_center_z + Zoffset);
                //Safety check
                if ((new_x >= 0 && new_y >= 0 && new_z >= 0) && (new_x < world->xDim && new_y < world->yDim && new_z < world->zDim))
                {
                    //Merge the new point into the world
                    world->dVoxels[new_x][new_y][new_z].deformity = object->dVoxels[i][j][k].deformity;
                    world->dVoxels[new_x][new_y][new_z].cost = object->dVoxels[i][j][k].cost;
                    world->dVoxels[new_x][new_y][new_z].R = object->dVoxels[i][j][k].R;
                    world->dVoxels[new_x][new_y][new_z].G = object->dVoxels[i][j][k].G;
                    world->dVoxels[new_x][new_y][new_z].B = object->dVoxels[i][j][k].B;
                    world->dVoxels[new_x][new_y][new_z].A = object->dVoxels[i][j][k].A;
                }
                else
                {
                    printf("Attempted to transform to an illegal state!\n");
                    printf("Attempted to transform index to %d|%d|%d, but the boundaries are 0|0|0 and %d|%d|%d!\n", new_x, new_y, new_z, world->xDim, world->yDim, world->zDim);
                }
                free(rotated_point);
            }
        }
    }
    return world;
}

int DVoxelEqual(dVoxel one, dVoxel two)
{
    if (one.deformity == two.deformity)
    {
        if (one.cost == two.cost)
        {
            if (one.R == two.R && one.G == two.G && one.B == two.B && one.A == two.A)
            {
                return 1;
            }
        }
    }
    return 0;
}

/** DEPRECATED AND REPLACED - DO NOT USE
int MergeDVoxelArrays(dVoxelArray *mainArray, dVoxelArray *arrayToMerge, int Xs, int Ys, int Zs)
{
    //Copy data
    int count = 0;
    for (int i = 0; i < (arrayToMerge->xDim); i++)
    {
        if ((i + Xs) < mainArray->xDim && (i + Xs) >= 0)
        {
            for (int j = 0; j < (arrayToMerge->yDim); j++)
            {
                if ((j + Ys) < mainArray->yDim && (j + Ys) >= 0)
                {
                    for (int k = 0; k < (arrayToMerge->zDim); k++)
                    {
                        if ((k + Zs) < mainArray->zDim && (k + Zs) >= 0)
                        {
                            int X = i + Xs;
                            int Y = j + Ys;
                            int Z = k + Zs;
                            mainArray->dVoxels[X][Y][Z].deformity = arrayToMerge->dVoxels[i][j][k].deformity;
                            mainArray->dVoxels[X][Y][Z].cost = arrayToMerge->dVoxels[i][j][k].cost;
                            mainArray->dVoxels[X][Y][Z].R = arrayToMerge->dVoxels[i][j][k].R;
                            mainArray->dVoxels[X][Y][Z].G = arrayToMerge->dVoxels[i][j][k].G;
                            mainArray->dVoxels[X][Y][Z].B = arrayToMerge->dVoxels[i][j][k].B;
                            mainArray->dVoxels[X][Y][Z].A = arrayToMerge->dVoxels[i][j][k].A;
                            count++;
                        }
                    }
                }
            }
        }
    }
    //Return data
    return count;
}
*/
