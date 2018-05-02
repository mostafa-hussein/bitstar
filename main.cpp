#include <iostream>
#include <vector>
#include <queue>
#include "data_structure.hpp"
#include<fstream>
#include<string>


using namespace std ;

int ** map;

void read_data(double & st_x,double & st_y,double & st_theta, int & sizex,int & sizey,double & goal_x,double & goal_y,double & goal_theta,string filename);

void bitstar(double & st_x,double & st_y,double & st_theta, int & sizex,int & sizey,double & goal_x,double & goal_y,double & goal_theta);


int main(int argc, char *argv[])
{
    double st_x,st_y,st_theta,goal_x,goal_y,goal_theta;
    int sizex,sizey;

    read_data(st_x,st_y,st_theta,sizex,sizey,goal_x,goal_y,goal_theta,argv[1]);

    bitstar(st_x,st_y,st_theta,sizex,sizey,goal_x,goal_y,goal_theta);

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

    map =new int * [m];

    for (int k = 0; k < m; ++k)
        map[k]=new int[n];

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
                map[i][j] = 1;
            }
            else
                map[i][j]=0;
        }
    }
    data.close();

    /*for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            if(map[i][j]==1)
                cout<<i<<" "<<j<<endl;
        }
    }printf("%f %f %f %f %f %f \n",st_x,st_y,st_theta,goal_x,goal_y,goal_theta);
    */
}


void bitstar(double & st_x,double & st_y,double & st_theta, int & sizex,int & sizey,double & goal_x,double & goal_y,double & goal_theta)
{
}