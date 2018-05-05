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

bool cmp_V( vertex * v1,  vertex * v2)
{
    if( v1->gt+v1->h >= v2->gt+v2->h)
        return v1->gt+v1->h >= v2->gt+v2->h;

    else if(v1->gt+v1->h == v2->gt+v2->h)
        return v1->gt >= v2->gt;
}

bool cmp_E( edge * e1,  edge * e2)
{
    if((e1->st->gt + e1->chat + e1->end->h) >=  (e2->st->gt + e2->chat + e2->end->h))
        return e1->st->gt + e1->chat + e1->end->h >=  e2->st->gt + e2->chat + e2->end->h;

    else if (e1->st->gt + e1->chat + e1->end->h ==  e2->st->gt + e2->chat + e2->end->h)
        return e1->st->gt + e1->chat  >=  e2->st->gt + e2->chat ;

    else if (e1->st->gt + e1->chat  ==  e2->st->gt + e2->chat)
        return  e1->st->gt   >=  e2->st->gt ;
}


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
