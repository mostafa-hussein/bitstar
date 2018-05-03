//
// Created by mostafa on 5/2/18.
//

#ifndef BITSTAR_DUMMY_HPP
#define BITSTAR_DUMMY_HPP

#endif //BITSTAR_DUMMY_HPP

//
// Created by mostafa on 5/2/18.
//

#include <iostream>
#include <vector>
#include <math.h>
#include "data_structure.hpp"
#include<fstream>
#include<string>
#include <boost/heap/fibonacci_heap.hpp>
#include <cfloat>
using namespace std ;

boost::heap::fibonacci_heap<int> fh;
fh.push(5);
fh.push(5);
fh.push(15);
fh.push(20);
boost::heap::fibonacci_heap<int>::handle_type h2= fh.s_handle_from_iterator(fh.begin());



boost::heap::fibonacci_heap<int>::handle_type h1 = fh.push(3);



for( int  v : fh)
{
std::cout << v << "\n";
}


cout<< "\n\n\n hello \n\n\n\n";


fh.erase(h1);
fh.erase(h2);

for( int  v : fh)
{
std::cout << v << "\n";
}


auto it=QV.begin(), end=QV.end();
for (int i=0;i<2;i++)
{
//vertex * tmp;
auto tmp = * it;
cout<<tmp->gt;
cout<<"\n";
it++;
}




std::vector<value> result_n;
rtree.query(bgi::nearest(point(0, 0), 5), std::back_inserter(result_n));

At the end we'll print results.

// display results
std::cout << "spatial query box:" << std::endl;
std::cout << bg::wkt<box>(query_box) << std::endl;
std::cout << "spatial query result:" << std::endl;
BOOST_FOREACH(value const& v, result_s)
std::cout << bg::wkt<box>(v.first) << " - " << v.second << std::endl;

std::cout << "knn query point:" << std::endl;
std::cout << bg::wkt<point>(point(0, 0)) << std::endl;
std::cout << "knn query result:" << std::endl;
BOOST_FOREACH(value const& v, result_n)
std::cout << bg::wkt<box>(v.first) << " - " << v.second << std::endl;
