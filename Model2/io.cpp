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

// construct districts from hess variables
void translate_solution(GRBVar** x, vector<int>& sol, int n, const vector<vector<bool>>& F0, const vector<vector<bool>>& F1)
{
    // translate the solution
    sol.resize(n);

    vector<int> heads(n, 0);
    int cur = 1;
    // firstly assign district number for clusterheads
    for (int i = 0; i < n; ++i)
    {
        if ((F1[i][i] && !F0[i][i]) || x[i][i].get(GRB_DoubleAttr_X) > 0.5)
            heads[i] = cur++;
    }
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            if ((F1[i][j] && !F0[i][j]) || x[i][j].get(GRB_DoubleAttr_X) > 0.5)
                sol[i] = heads[j];

}

// prints the solution <node> <district>
void printf_solution(const vector<int>& sol, const char* fname)
{
    // write solution to file
    if (sol.size() > 0)
    {
        FILE* f;
        if (fname)
            f = fopen(fname, "w");
        else
            f = stderr;

        for (int i = 0; i < sol.size(); ++i)
            fprintf(f, "%d %d\n", i, sol[i]);

        if (fname)
            fclose(f);
    }
}

void calculate_UL(const vector<int> population, int k, int* L, int* U)
{
    int total_pop = 0;
    for (int p : population)
        total_pop += p;
    *U = static_cast<int>(floor(static_cast<float>(total_pop) / static_cast<float>(k) * 1.005));
    *L = static_cast<int>(ceil(static_cast<float>(total_pop) / static_cast<float>(k) * 0.995));
}



//read ralg initial point from file [fname] to [x0]
void read_ralg_hot_start(const char* fname, double* x0, int dim)
{
    FILE* f = fopen(fname, "r");
    if (!f)
    {
        fprintf(stderr, "Failed to open %s.\n", fname);
        return;
    }
    for (int i = 0; i < dim; ++i)
    {
        double val = 0.;
        if (EOF == fscanf(f, "%lf ", &val))
        {
            fprintf(stderr, "Failure to complete reading ralg x0 from %s.\n", fname);
            break;
        }
        x0[i] = val;
    }
    fclose(f);
}

// dump result to "ralg_hot_start.txt"
void dump_ralg_hot_start(double* res, int dim)
{
    const char* outname = "ralg_hot_start.txt";
    FILE* f = fopen(outname, "w");
    if (!f)
    {
        fprintf(stderr, "Cannot open %s for dumping ralg result.\n", outname);
        return;
    }
    for (int i = 0; i < dim; ++i)
        fprintf(f, "%lf\n", res[i]);
    fclose(f);
}
