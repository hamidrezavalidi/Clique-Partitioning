#include "io.h"
#include <vector>
#include <cstdio>
#include <cstring>
#include <cmath>
#include "gurobi_c++.h"

using namespace std;

int read_input_data(const char* a_fname, int n, int m, // INPUTS
    vector<vector<int>>& a) // OUTPUTS
{
    // read matrix a (must be sorted)
    FILE* f = fopen(a_fname, "r");
    if (!f) 
    {
        fprintf(stderr, "Failed to open %s\n", a_fname);
        return 1;
    }
    // file contains the first row as a header row, skip it
    // also skip the first element in each row (node id)
    char buf[50000]; //dummy
    fgets(buf, sizeof(buf), f); // skip first line
    if (strlen(buf) >= sizeof(buf) - 5)
        printf("WARNING: Possible buffer overlow!\n");
    a.resize(n);
    for (int i = 0; i < n; ++i)
    {
        a[i].resize(m);
        int d; fscanf(f, "%d,", &d); // skip first element
        for (int j = 0; j < m; ++j) 
        {
            fscanf(f, "%d,", &d);
            a[i][j] = d;
        }
    }
    fclose(f);
    return 0;
}

