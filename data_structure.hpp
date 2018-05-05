//
// Created by mostafa on 5/2/18.
//

#ifndef BITSTAR_DATA_STRUCTURE_HPP
#define BITSTAR_DATA_STRUCTURE_HPP

#endif //BITSTAR_DATA_STRUCTURE_HPP

#include <utility>
#include <iostream>
#include <vector>
typedef unsigned long int lint;

using namespace std;


struct vertex
{
    double x;
    double y;
    double theta;
    double ghat;
    double gt;
    double h;
    vertex * parent;
    lint id;
};

struct edge
{
    vertex * st;
    vertex * end;
    double c=-1;
    double chat;
    lint id;
};


class tree
{
public:

    vector <vertex *> V;
    vector <edge *> E;

    tree(vertex * v)
    {
        V.push_back(v);
    }
};
