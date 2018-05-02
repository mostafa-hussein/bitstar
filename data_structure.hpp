//
// Created by mostafa on 5/2/18.
//

#ifndef BITSTAR_DATA_STRUCTURE_HPP
#define BITSTAR_DATA_STRUCTURE_HPP

#endif //BITSTAR_DATA_STRUCTURE_HPP

#include <utility>
#include <iostream>


struct vertex
{
    double x;
    double y;
    double theta;
    double g;
    double gt;
    double h;
};

struct edge
{
    vertex st;
    vertex end;
    double c;
    double chat;
};

