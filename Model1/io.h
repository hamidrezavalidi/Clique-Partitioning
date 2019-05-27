// Much of the IO class is written by Eugene Lykhovyd (student of Sergiy Butenko).
// Modified by Hamidreza Validi 
#ifndef _IO_H
#define _IO_H
#include "gurobi_c++.h"
#include <vector>

using namespace std;

int read_input_data(const char* a_fname, int n, int m, // INPUTS
    vector<vector<int>>& a); // OUTPUTS
#endif
