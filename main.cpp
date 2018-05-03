#include <iostream>
#include <vector>
#include <math.h>
#include "data_structure.hpp"
#include "dummy.hpp"
#include<fstream>
#include<string>
#include <boost/heap/fibonacci_heap.hpp>
#include <cfloat>
#include <chrono>
#include <unordered_map>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#define inf DBL_MAX
#define pi (22.0/7.0)
#define ex  boost::math::constants::e<double>()

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
using namespace std ;


typedef bg::model::point<double, 3, bg::cs::cartesian> point;

typedef unsigned long int lint;

int ** Map,sizex,sizey;
double st_x,st_y,st_theta,goal_x,goal_y,goal_theta;

unordered_map<lint , vertex * > V_table;

unordered_map<lint , vertex * > V_old_table;

bgi::rtree< point, bgi::quadratic<16> > V_rtree;

bgi::rtree< point, bgi::quadratic<16> > X_samples_rtree;


void read_data(string filename);

void bitstar();

double h_distance (vertex * v1, vertex * v2);

void prune (double c_best);

lint get_id (vertex *);

void sample (vector<vertex *> & X_samples,int m, double c_best,vertex * , vertex* );

void Expand_Vertex(vertex * current);


int main(int argc, char *argv[])
{
    auto start = std::chrono::high_resolution_clock::now();
    double st_x,st_y,st_theta,goal_x,goal_y,goal_theta;
    int sizex,sizey;

    srand (static_cast <unsigned> (time(0))); //time(0)  random seed

    read_data(argv[1]);

    bitstar();

    auto finish = std::chrono::high_resolution_clock::now();

    cout << std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count()/1000 << "ms\n";

    return 0;
}

void read_data(string filename)
{
    ifstream data;
    data.open(filename);

    char dummy;
    int obs [100][2];
    int obsno=0;
    int tmp=0;

    data>>sizex;  //4
    data>>sizey;  //3

    int m=sizex;
    int n=sizey;

    Map =new int * [m];

    for (int k = 0; k < m; ++k)
        Map[k]=new int[n];

    for (int i = 0; i <sizey; ++i)
    {
        for (int j = 0; j < sizex; ++j)
        {
            data>>dummy;

            if(dummy=='#')
            {
                obs[obsno][0]=j;
                obs[obsno][1]=sizey - 1 - i;
                obsno++;
            }
        }
    }

    data>>st_x>>st_y>>st_theta>>goal_x>>goal_y>>goal_theta;

    for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            if(i==obs[tmp][0] && j== obs[tmp][1] )
            {
                tmp++;
                Map[i][j] = 1;
            }
            else
                Map[i][j]=0;
        }
    }
    data.close();

    /*for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            if(Map[i][j]==1)
                cout<<i<<" "<<j<<endl;
        }
    }printf("%f %f %f %f %f %f \n",st_x,st_y,st_theta,goal_x,goal_y,goal_theta);
    */
}


void bitstar()
{
    boost::heap::fibonacci_heap<vertex * , boost::heap::compare<cmp_V>> QV ;

    boost::heap::fibonacci_heap<edge * , boost::heap::compare<cmp_E>> QE ;

    vector<vertex *> X_samples;
    int knn=0;

    vertex * x_start ,*  x_goal;

    bool done =0;
    double c_best= inf , rewireFactor=1.1;




    ///********************************* start initialization ********************************************************\\\

    x_start = new vertex;

    x_goal = new vertex;

    x_start->x=st_x;
    x_start->y=st_y;
    x_start->theta=st_theta;
    x_start->ghat=0;
    x_start->gt=0;
    x_start->h=h_distance(x_start,x_goal);

    x_goal->x=goal_x;
    x_goal->y=goal_y;
    x_goal->theta=goal_theta;
    x_start->ghat=h_distance(x_goal,x_start);
    x_start->gt=inf;
    x_start->h=0;


    V_rtree.insert(point(x_start->x,x_start->y,x_start->theta)); // intialize V kdtree

    V_table.insert({get_id(x_start),x_start});  // intialize V table

    tree T(x_start);   // intialize V

    T.E.clear();      //  intialize E

    X_samples.push_back(x_goal);  //  intialize x_samples
    X_samples_rtree.insert(point(x_goal->x,x_goal->y,x_goal->theta));   //  intialize x_samples kdtree

    knn=int(inf);

    QV.clear();
    QE.clear();


///********************************* done initialization ********************************************************\\\
///

    while (! done)
    {
        if (QV.empty() && QE.empty() )
        {
            prune(c_best);
            sample (X_samples,100,c_best,x_goal,x_start);

            for (int i = 0; i < T.V.size(); ++i)
            {
                V_old_table.insert({get_id(T.V[i]),T.V[i]});
                QV.push(T.V[i]);
            }

            knn=rewireFactor * log(T.V.size()+X_samples.size()) * (ex + ex/3) ;
        }

        auto QV_it =QV.begin();

        auto QE_it =QE.begin();

        vertex * current;

        do
        {
            current = (* QV_it);

            Expand_Vertex(current);
        }
        while ( (*QV_it)->gt+(*QV_it)->h <= (*QE_it)->st->gt + (*QE_it)->chat + (*QE_it)->end->h );




        done=1;
    }






}


double h_distance (vertex * v1, vertex * v2)
{
    return sqrt( (v1->x - v2->x) * (v1->x - v2->x) + (v1->y - v2->y) * (v1->y - v2->y)   );
}

void prune (double c_best)
{

}

lint get_id (vertex * v)
{
    lint key,y,x;

    x= hash<double >{}(v->x+v->y*sizex);

    y= hash<double >{}(v->theta);

    key =(x+y)*(x+y+1)/2+x;

    return key;

}

void sample (vector<vertex *> & X_samples,int m, double c_best ,vertex * x_goal , vertex * x_start)
{
    int count=0;
    double ran_x,ran_y,ran_theta;
    vertex * t;

    while (count< m)
    {
        ran_x = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (sizex-0.0001)));
        ran_y = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (sizey-0.0001)));
        ran_theta = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (2*pi-0.0001)));

        if(Map[(int)ran_x][(int)ran_y]==1)
            continue;

        count++;

        t= new vertex;

        t->x=ran_x;
        t->y=ran_y;
        t->theta=ran_theta;
        t->h=h_distance(t,x_goal);
        t->ghat=h_distance(t,x_start);
        t->gt = inf;

        X_samples.push_back(t);
        X_samples_rtree.insert(point(t->x,t->y,t->theta));
    }

}


void Expand_Vertex(vertex * current)
{

}













