import math
import time

class DTW:

    def __init__(self):
        pass

    def Evaluate(self, array_1, array_2, distancefn, exportMatrix=False):
        #start = time.clock()
        DTW_matrix = [[0 for y in range(len(array_2)+1)] for x in range(len(array_1)+1)]
        #Populate matrix
        for i in range(1, len(array_2)+1):
            DTW_matrix[0][i] = float("inf")
        for i in range(1, len(array_1)+1):
            DTW_matrix[i][0] = float("inf")
        DTW_matrix[0][0] = 0.0
        #Evaluate
        for i in range(1, len(array_1)+1):
            for j in range(1, len(array_2)+1):
                cost = distancefn(array_1[i-1], array_2[j-1])
                DTW_matrix[i][j] = cost + min(DTW_matrix[i-1][j], DTW_matrix[i][j-1], DTW_matrix[i-1][j-1])
        #Return
        #end = time.clock()
        #print "[DEBUG] Individual DTW comparisson took " + str(end - start) + " seconds"
        if (exportMatrix):
            return [DTW_matrix[len(array_1)][len(array_2)], DTW_matrix]
        else:
            return DTW_matrix[len(array_1)][len(array_2)]

class PreAllocDTW:

    def __init__(self, x_dim, y_dim, fixed_array=None, distance_fn=None):
        self.MX = x_dim
        self.MY = y_dim
        self.DTW_matrix = [[0 for y in range(y_dim+1)] for x in range(x_dim+1)]
        #Populate matrix
        for j in range(1, y_dim+1):
            self.DTW_matrix[0][j] = float("inf")
        for i in range(1, x_dim+1):
            self.DTW_matrix[i][0] = float("inf")
        self.DTW_matrix[0][0] = 0.0
        #Setup data
        if (fixed_array != None):
            self.A1 = fixed_array
        else:
            self.A1 = []
        self.A2 = []
        #Set distance function
        if (distance_fn == None):
            self.Distance = self.EuclideanDistance
        else:
            self.Distance = distance_fn

    def Update(self, new_a2_value, new_a1_value=None):
        assert(len(self.A2) < self.MY)
        if (new_a1_value != None):
            assert(len(self.A1) < self.MX)
            self.A1.append(new_a1_value)
        self.A2.append(new_a2_value)

    def ReEvaluate(self):
        assert(len(self.A1) > 0)
        assert(len(self.A2) > 0)
        for i in range(1, len(self.A1)+1):
            for j in range(1, len(self.A2)+1):
                cost = self.Distance(self.A1[i-1], self.A2[j-1])
                self.DTW_matrix[i][j] = cost + min(self.DTW_matrix[i-1][j], self.DTW_matrix[i][j-1], self.DTW_matrix[i-1][j-1])
        #Return
        return self.DTW_matrix[len(self.A1)][len(self.A2)]

    def EuclideanDistance(self, state1, state2):
        assert(len(state1) == len(state2))
        total = 0.0
        for index in range(len(state1)):
            temp = (state1[index] - state2[index]) ** 2
            total = total + temp
        return math.sqrt(total)
