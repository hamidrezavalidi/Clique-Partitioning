#include "gurobi_c++.h"
#include "io.h"
#include <algorithm>
#include <cstdio>
#include <vector>
#include <cmath>
#include <cstring>
#include <chrono>
#include <string>

using namespace std;

int main(int argc, char *argv[])
{
    ios::sync_with_stdio(1);
    // parse command line arguments
    int n = atoi(argv[1]);
    int m = atoi(argv[2]);
    char* a_fname = argv[3];

    // read inputs
    vector<vector<int>> a;
    if (read_input_data(a_fname, n, m, a))
        return 1; // failure

    printf("Model input: n = %d, m = %d\n", n, m);

    auto start = chrono::steady_clock::now();

    // convert a's to w's
    int nTotal = n + m;

    vector<vector<int>> w(nTotal, vector<int>(nTotal, 0));
    for (int i = 0; i < n; ++i)
    {
        for (int j = n; j < nTotal; ++j)
        {
           if(a[i][j-n] == 1)
              w[i][j] = a[i][j-n];
           else
              w[i][j] = -1;
        }
    }

    try {
        // initialize environment and create an empty model
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);
        model.set(GRB_DoubleParam_TimeLimit, 3600.); // 1 hour
        model.set(GRB_IntParam_Threads, 10);
        model.set(GRB_DoubleParam_NodefileStart, 10); // 10 GB
        model.set(GRB_IntParam_Method, 3);
        model.set(GRB_DoubleParam_MIPGap, 0);

        // create variables y
        GRBVar** y = new GRBVar*[nTotal];
        for (int i = 0; i < nTotal; ++i)
        {
            y[i] = new GRBVar[nTotal];
            for (int j = 0; j < nTotal; ++j)
                y[i][j] = model.addVar(0., 1., w[i][j], GRB_BINARY);
        }
        model.update();

        // set objective function
        model.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);

        // add triangular constraints
        for (int i = 0; i < nTotal; ++i)
        {
            for (int j = i + 1; j < nTotal; ++j)
            {
                for (int k = j + 1; k < nTotal; ++k)
                {
                    model.addConstr(y[i][j] + y[j][k] <= 1 + y[i][k]);
                    model.addConstr(y[i][j] + y[i][k] <= 1 + y[j][k]);
                    model.addConstr(y[i][k] + y[j][k] <= 1 + y[i][j]);
                }
            }
        }

        model.update();

        model.write("debug_cliquePartitioning.lp");

        //optimize the model

        model.optimize();
        chrono::duration<double> duration = chrono::steady_clock::now() - start;
        printf("Time elapsed: %lf seconds\n", duration.count()); // TODO use gurobi Runtime model attr

        //translate solution to clique partitions
 
        vector<vector<int>> cluster (nTotal);
        vector<bool> assigned (nTotal, false);

        for (int i = 0; i < nTotal; ++i)
        {
            if (assigned[i]) continue;
            cluster[i].push_back(i);
            assigned[i] = true;
            for (int j = i+1; j < nTotal; ++j)
            {
                if (assigned[j]) continue;
                if (y[i][j].get(GRB_DoubleAttr_X) > 0.5)
                {
                    cluster[i].push_back(j);
                    assigned[j] = true;
                }
            }
        }

        int count = 0;

        for (int i = 0; i < nTotal; ++i)
        {
            if (cluster[i].size() == 0) continue;
            count++;
        }

        cerr << "# of clusters: " << count << endl;

        count = 0;

        for (int i = 0; i < nTotal; ++i)
        {
            if (cluster[i].size() == 0) continue;
            count++;
            vector<int> parts;
            vector<int> machines;
            for (int j = 0; j < cluster[i].size(); ++j)
            {
                int cur = cluster[i][j];
                if (cur < n)
                    parts.push_back(cur + 1);
                else
                    machines.push_back(cur - n + 1);
            }
            cerr << "Cluster " << count << " containts the following parts:" << endl;
            for (int j = 0; j < parts.size(); ++j)
                cerr << parts[j] << endl;
            cerr << "Cluster " << count << " containes the following machines:" << endl;
            for (int j = 0; j < machines.size(); ++j)
                cerr << machines[j] << endl;
        }
    }
    catch (GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
    catch (const char* msg) {
        cout << "Exception with message : " << msg << endl;
    }
    catch (...) {
        cout << "Exception during optimization" << endl;
    }
    return 0;
}
