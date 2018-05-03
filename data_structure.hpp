//
// Created by mostafa on 5/2/18.
//

#ifndef BITSTAR_DATA_STRUCTURE_HPP
#define BITSTAR_DATA_STRUCTURE_HPP

#endif //BITSTAR_DATA_STRUCTURE_HPP

#include <utility>
#include <iostream>
#include <vector>

using namespace std;


struct vertex
{
    double x;
    double y;
    double theta;
    double ghat;
    double gt;
    double h;
};

struct edge
{
    vertex * st;
    vertex * end;
    double c;
    double chat;
};


struct cmp_V
{
    bool operator()(const vertex * v1,const  vertex * v2) const
    {
        return v1->gt > v2->gt;
    }
};

struct cmp_E
{
    bool operator()(const edge * e1, const edge * e2) const
    {
        return e1->chat > e2->chat;
    }
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
