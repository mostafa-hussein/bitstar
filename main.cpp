#include <iostream>
#include <vector>
#include <math.h>
#include "data_structure.hpp"
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
#define inf DBL_MAX

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
using namespace std ;


typedef bg::model::point<double, 3, bg::cs::cartesian> point;

typedef unsigned long int lint;

int ** Map;

unordered_map<lint , vertex * > V_table;

unordered_map<lint , vertex * > V_old_table;

bgi::rtree< point, bgi::quadratic<16> > V_rtree;

bgi::rtree< point, bgi::quadratic<16> > X_samples_rtree;


void read_data(double & st_x,double & st_y,double & st_theta, int & sizex,int & sizey,double & goal_x,double & goal_y,double & goal_theta,string filename);

void bitstar(double & st_x,double & st_y,double & st_theta, int & sizex,int & sizey,double & goal_x,double & goal_y,double & goal_theta);

double h_distance (vertex * v1, vertex * v2);

void prune (double c_best);

lint get_id (vertex *,int sizex);


int main(int argc, char *argv[])
{
    auto start = std::chrono::high_resolution_clock::now();
    double st_x,st_y,st_theta,goal_x,goal_y,goal_theta;
    int sizex,sizey;

    read_data(st_x,st_y,st_theta,sizex,sizey,goal_x,goal_y,goal_theta,argv[1]);

    bitstar(st_x,st_y,st_theta,sizex,sizey,goal_x,goal_y,goal_theta);

    auto finish = std::chrono::high_resolution_clock::now();

    cout << std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count()/1000 << "ms\n";

    return 0;
}

void read_data(double & st_x,double & st_y,double & st_theta, int & sizex,int & sizey,double & goal_x,double & goal_y,double & goal_theta,string filename)
{
    ifstream data;
    data.open(filename);

    char dummy;
    int obs [100][2];
    int obsno=0;
    int tmp=0;

    data>>sizex;
    data>>sizey;

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


void bitstar(double & st_x,double & st_y,double & st_theta, int & sizex,int & sizey,double & goal_x,double & goal_y,double & goal_theta)
{
    boost::heap::fibonacci_heap<vertex * , boost::heap::compare<cmp_V>> QV ;

    boost::heap::fibonacci_heap<edge * , boost::heap::compare<cmp_E>> QE ;

    vector<vertex *> X_samples;
    int knn=0;

    vertex * x_start ,*  x_goal;

    bool done =0;
    double c_best= inf;






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

    V_table.insert({get_id(x_start,sizex),x_start});  // intialize V table

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
        }
        done=1;
    }










\
}


double h_distance (vertex * v1, vertex * v2)
{
    return sqrt( (v1->x - v2->x) * (v1->x - v2->x) + (v1->y - v2->y) * (v1->y - v2->y)   );
}

void prune (double c_best)
{

}


lint get_id (vertex * v,int sizex)
{
    lint key,y,x;

    x= hash<double >{}(v->x+v->y*sizex);

    y= hash<double >{}(v->theta);

    key =(x+y)*(x+y+1)/2+x;

    return key;

}


















