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
    int n = atoi(argv[1]); // number of parts
    int m = atoi(argv[2]); // number of machines
    int k = atoi(argv[3]); // maximum number of expected clusters
    char* a_fname = argv[4];

    // read inputs
    vector<vector<int>> a;
    if (read_input_data(a_fname, n, m, a))
        return 1; // failure

    printf("Model input: n = %d, m = %d, k = %d\n", n, m, k);

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

        // create variables x
        GRBVar** x = new GRBVar*[nTotal];
        for (int i = 0; i < nTotal; ++i)
        {
            x[i] = new GRBVar[k];
            for (int j = 0; j < k; ++j)
                x[i][j] = model.addVar(0., 1., 0, GRB_BINARY);
        }
        model.update();

        // create variables z
        GRBVar ***z = new GRBVar**[nTotal];
        for (long i = 0; i < nTotal; ++i)
        {
            GRBVar **z_temp = new GRBVar*[nTotal];
            for (long j = 0; j < nTotal; ++j)
                z_temp[j] = model.addVars(k, GRB_BINARY);
            z[i] = z_temp;
        }
        model.update();

        // set objective: maximize sum w_ij*z^k_ij
        GRBLinExpr expr = 0;
        for (int i = 0; i < nTotal; ++i)
        {
            for (int j = i + 1; j < nTotal; ++j)
            {
                GRBLinExpr exprZ = 0;
                for (int u = 0; u < k; ++u)
                    exprZ += z[i][j][u];
                
                expr += w[i][j] * exprZ;
            }
        }
        
        model.setObjective(expr, GRB_MAXIMIZE);

        // add constraints (21b)
        for (int i = 0; i < nTotal; ++i)
        {
            GRBLinExpr expr1 = 0;

            for (int j = 0; j < k; ++j)
                expr1 += x[i][j];
            
            model.addConstr(expr1 == 1);
        }

        // add constraints (21c), (21d), (21e)
        for (int i = 0; i < nTotal; ++i)
        {
            for (int j = i + 1; j < nTotal; ++j)
            {
                for (int u = 0; u < k; ++u)
                {
                    model.addConstr(z[i][j][u] <= x[i][u]);
                    model.addConstr(z[i][j][u] <= x[j][u]);
                    model.addConstr(x[i][u] + x[j][u] <= 1 + z[i][j][u]);
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
 
        vector<vector<int>> cluster (k);
        vector<bool> assigned (nTotal, false);

        

        for (int i = 0; i < nTotal; ++i)
        {
            for (int j = 0; j < k; ++j)
            {
                if (x[i][j].get(GRB_DoubleAttr_X) > 0.5)
                    cluster[j].push_back(i);
            }
        }

        int count = 0;

        for (int i = 0; i < k; ++i)
        {
            if (cluster[i].size() == 0) continue;
            count++;
        }

        cerr << "# of clusters: " << count << endl;

        count = 0;

        for (int i = 0; i < k; ++i)
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
