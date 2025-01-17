#include <iostream>
#include <vector>
#include <cmath>
#include "data_structure.hpp"
#include<fstream>
#include <boost/heap/fibonacci_heap.hpp>
#include <cfloat>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_map>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include "dubins.h"
#include "dubins.cpp"
#include <algorithm>
#include <fcntl.h>

#define debug


#define inf  (DBL_MAX)
#define pi (22.0/7.0)
#define ex  boost::math::constants::e<double>()

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
using namespace std ;


typedef bg::model::point<double, 3, bg::cs::cartesian> point;

//typedef boost::heap::fibonacci_heap<edge * , boost::heap::compare<cmp_E>> edge_queue;
//typedef  boost::heap::fibonacci_heap<vertex * , boost::heap::compare<cmp_V>> vertex_queue;

typedef vector<edge *> edge_queue;
typedef vector<vertex *> vertex_queue;

typedef unsigned long int lint;

int ** Map,sizex,sizey;
double st_x,st_y,st_theta,goal_x,goal_y,goal_theta;
vertex * x_start ,*  x_goal;
vector<double*> res;
int n=1;
double min_r=0.1;


unordered_map<lint , edge * > E_table;

unordered_map<lint , vertex * > V_table;

unordered_map<lint , vertex * > V_old_table;

bgi::rtree< point, bgi::quadratic<16> > V_rtree;

bgi::rtree< point, bgi::quadratic<16>> X_samples_rtree;

std::ofstream out;



void read_data(string filename);

void bitstar();

double h_distance (vertex * v1, vertex * v2);

void prune (double c_best,vector<vertex *> & X_samples ,tree & T );

lint get_id (vertex *);

lint get_id (vertex *,vertex *);

void sample (vector<vertex *> & X_samples,int m, double c_best);

void Expand_Vertex(vertex * current, vertex_queue & QV , edge_queue & QE ,int knn,double c_best,tree &  T );

bool collision_check_dubin (vertex * v , vertex * x, double & c );

int printConfiguration(double q[3], double x, void* user_data);

double Best_queue_value (vertex_queue & QV);

double Best_queue_value (edge_queue & QE);

void get_path (vector<vertex *> v);

void print_sol(tree T);

void disp (vertex *);

void disp (edge *);



int main(int argc, char *argv[])
{

    string filename="map.sw";

    if(argc >1)
    {
        filename = argv[1];
        n=stoi(argv[1]);
    }

    if(argc >2)
    {
        min_r=stod(argv[2]);
    }

    read_data(filename);

    //cout<<"done reading \n";

    bitstar();



    return 0;
}

void read_data(string filename)
{
    ifstream data;
    data.open(filename);

    char dummy;
    int obsno=0;

#ifdef debug
    cin>>sizex;  //4
    cin>>sizey;  //3
#else
    data>>sizex;  //4
        data>>sizey;  //3
#endif

    int m=sizex;
    int n=sizey;

    Map =new int * [m];

    for (int k = 0; k < m; ++k)
        Map[k]=new int[n];

    for (int i = 0; i < m; ++i)
        for (int j = 0; j < n; ++j)
            Map[i][j]=0;

    for (int i = 0; i <sizey; ++i)
    {
        for (int j = 0; j < sizex; ++j)
        {
#ifdef debug
            cin>>dummy;
#else
            data>>dummy;
#endif

            if(dummy=='#')
            {
                Map[j][sizey - 1 - i]=1;
                obsno++;
            }
            /*else if(dummy=='S')
            {
                st_x=j;
                st_y=sizey - 1 - i;
            }
            else if(dummy=='F')
            {
                goal_x=j;
                goal_y=sizey - 1 - i;
            }*/
        }
    }
#ifdef debug
    cin>>st_x>>st_y;
    cin>>goal_x>>goal_y;
    goal_theta=0;
    st_theta=0;
#else
    data>>st_x>>st_y;st_theta=0;data>>goal_x>>goal_y; goal_theta=0;
#endif

    data.close();

    //cout<<obsno<<endl;
    //cout<<sizex  <<"  "  <<sizey <<endl;
    /*for (int i = 0; i < m; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            if(Map[i][j]==1)
                cout<<i<<" "<<j<<endl;
        }
    }*/
    //printf("%f %f %f %f %f %f \n",st_x,st_y,st_theta,goal_x,goal_y,goal_theta);

}

void bitstar()
{
    auto start = std::chrono::high_resolution_clock::now();

    vertex_queue QV ;
    edge_queue QE ;

    vector<vertex *> X_samples;

    int knn;

    x_start = new vertex;
    x_goal = new vertex;

    int done =0;
    double c_best= (inf) , rewireFactor=1.1;

    srand (static_cast <unsigned> (time(0)));
    //cout<<time(0)<<endl;//  random seed

    ///********************************* start initialization ********************************************************\\\

    x_start->x=st_x;
    x_start->y=st_y;
    x_start->theta=st_theta;
    x_start->ghat=0;
    x_start->gt=0;
    x_start->h=h_distance(x_start,x_goal);
    x_start->id = get_id(x_start);

    x_goal->x=goal_x;
    x_goal->y=goal_y;
    x_goal->theta=goal_theta;
    x_goal->ghat=h_distance(x_goal,x_start);
    x_goal->gt=(inf);
    x_goal->h=0;
    x_goal->id=get_id(x_goal);


    V_rtree.insert(point(x_start->x,x_start->y,x_start->theta)); // intialize V kdtree

    V_table.insert({x_start->id,x_start});  // intialize V table

    tree T(x_start);   // intialize V

    T.E.clear();      //  intialize E

    X_samples.push_back(x_goal);  //  intialize x_samples

    X_samples_rtree.insert(point(x_goal->x,x_goal->y,x_goal->theta));   //  intialize x_samples kdtree

    knn=int((inf));

    QV.clear();
    QE.clear();

    disp(x_start);
    disp(x_goal);

///********************************* done initialization ********************************************************\\\
///

    int co=0,sample_no=0;

    while (done <n &&  sample_no <100000)
    {
        co++;
        if ( QV.empty() && QE.empty() )
        {
            sample_no++;

            //cout<<"Vertex size =  "<<T.V.size()<<endl;
            //cout<<"edge size =  "<<T.E.size()<<endl;
            //cout<<"sample size =  "<<X_samples.size()<<endl;

            prune(c_best,X_samples,T);

            //cout<<"Vertex size =  "<<T.V.size()<<endl;
            //cout<<"edge size =  "<<T.E.size()<<endl;
            //cout<<"sample size =  "<<X_samples.size()<<endl;

            sample (X_samples,100,c_best);


            for (int i = 0; i < T.V.size(); ++i)
            {
                V_old_table.insert({T.V[i]->id,T.V[i]});

                QV.push_back(T.V[i]);
            }

            knn=(int)ceil(rewireFactor * log(T.V.size()+X_samples.size()) * (ex + ex/3.0));

            stable_sort(QV.begin(),QV.end(),cmp_V);
        }
        //cout<<"start calculating \n";

        vertex * current;

        while (Best_queue_value(QV) <= Best_queue_value(QE) )
        {
            //printf("%e <= %e \n",Best_queue_value(QV) , Best_queue_value(QE));

            current = QV[0];

            auto old = QE.size();

            Expand_Vertex(current ,QV ,QE,knn,c_best,T);

            if(QE.size() > old)
            {
                stable_sort(QE.begin(), QE.end(),cmp_E);
            }
        }

        //cout<<"done looping \n";

        edge *e ;

        vertex * v, *x ;

        e= QE[0];

        QE.erase(QE.begin());

        v=e->st;
        x=e->end;

        //cout<<"first test "<< v->gt + e->chat + x->h <<"< "<<c_best<<endl;

        if( (v->gt + e->chat + x->h) < c_best )
        {
            //cout<<"second test "<<v->ghat + e->c + x->h <<" < "<<c_best<<endl;

            if((v->gt + e->chat) < x->gt )
            {
                if(! collision_check_dubin(v,x,e->c))
                    continue;

                //cout<<"third test"<<(v->gt + e->c +x->h )<<"<  " <<c_best <<endl;

                if(v->gt + e->c +x->h  <c_best)
                {

                    if (( v->gt + e->c ) < x->gt )
                    {
                        unordered_map<lint , vertex * >::iterator it;

                        it= V_table.find(x->id);

                        if(it != V_table.end())
                        {

                            for (int i = 0; i < T.E.size(); ++i)
                            {
                                if(x->id == T.E[i]->end->id)
                                {
                                    T.E.erase(T.E.begin()+i);
                                }
                            }
                        }
                        else
                        {
                            //cout<<"push V \n";

                            T.V.push_back(x);

                            V_table.insert({x->id,x});

                            V_rtree.insert(point(x->x,x->y,x->theta));

                            QV.push_back(x);

                            stable_sort(QV.begin(),QV.end(),cmp_V);

                            X_samples_rtree.remove(point(x->x,x->y,x->theta));

                            //cout<<X_samples_rtree.size()<<endl;

                            for (int i = 0; i < X_samples.size(); ++i)
                            {
                                if(X_samples[i]->id == x->id)
                                {
                                    //cout<<"sample erased\n";
                                    X_samples.erase(X_samples.begin() +i);
                                    break;
                                }
                            }
                        }

                        x->gt=e->c + v->gt ;

                        T.E.push_back(e);

                        E_table.insert({e->id ,e});

                        if(x->id == x_goal->id)
                        {
                            c_best=x->gt;
                            x_goal->gt=c_best;
                            cout<<"I am done with best solution cost = "<<c_best<<endl;
                            auto finish = std::chrono::high_resolution_clock::now();
                            cout <<"Time = "<< std::chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count()/1000000 << "ms\n";
                            done++;
                            string file="sol.txt";

                            out.open(file+to_string(done));

                            print_sol(T);

                            out.close();

                            //if(c_best==2)
                            //done++;
                        }

                        vector <edge_queue::iterator> it1;

                        for (int j = 0; j <QE.size() ; ++j)
                        {
                            if(QE[j]->end->id == x->id)
                            {
                                if((QE[j]->st->gt + QE[j]->chat ) >= x->gt)
                                {
                                    it1.push_back(QE.begin()+j);
                                }
                            }
                        }
                        for (int k = it1.size()-1; k >=0; --k)
                        {
                            QE.erase(it1[k]);
                        }
                        stable_sort(QE.begin(), QE.end(),cmp_E);
                    }
                }
            }
        }
        else
        {
            QE.clear();
            QV.clear();
        }
        auto finish = std::chrono::high_resolution_clock::now();
        if (chrono::duration_cast <chrono::nanoseconds>(finish-start).count()/1000000000> 60 )
        {
            cout<<"time is done \n";
            return;
        }
    }
    //print_sol(T);
    cout<<"number of solutions = "<<done<<endl;
}

void print_sol(tree T)
{
    /*printf("no_sample_times = %d \n",sample_no);

   printf("count number %d \n",co);

   printf("Vertex tree count %d \n",(int)T.V.size());

   printf("Edge tree count %d \n",(int)T.E.size());

   /*printf("%d \n",(int)T.V.size());
   for (int i = 0; i < T.V.size(); ++i)
   {
       printf("%f %f\n",T.V[i]->x , T.V[i]->y);
   }*/

    vector<edge *>sol;
    vector<vertex *>final;
    edge * current;

    for (auto j = T.E.size()-1; j >=0 ; --j)
    {
        if(T.E[j]->end->id == x_goal->id)
        {
            current=T.E[j];
            break;
        }
    }
    sol.push_back(current);

    /*printf("%d \n",(int)T.E.size());
    for (int i = 0; i < T.E.size(); ++i)
    {
        printf("%f %f",T.E[i]->st->x , T.E[i]->st->y);
        printf(" %f %f\n",T.E[i]->end->x , T.E[i]->end->y);
    }*/

    bool finish =0;
    while (! finish)
    {
        for (int i = (int)T.E.size() - 1; i >= 0; --i)
        {
            if (current->st->id == T.E[i]->end->id)
            {
                sol.push_back(T.E[i]);
                current = T.E[i];
            }
            if (get_id(current->st) == x_start->id)
            {
                finish=1;
                break;
            }
        }
    }

    printf("%d \n",(int)sol.size()+1);

    printf("%f %f %f\n",sol[sol.size() -1]->st->x , sol[sol.size() -1]->st->y, sol[sol.size() -1]->st->theta);

    final.push_back(sol[sol.size() -1]->st);

    for (int i = sol.size() -2 ; i >= 0; --i)
    {
        printf("%f %f %f\n",sol[i]->st->x , sol[i]->st->y, sol[i]->st->theta);
        final.push_back(sol[i]->st);
    }
    printf("%f %f %f\n",sol[0]->end->x , sol[0]->end->y,sol[0]->end->theta);
    final.push_back(sol[0]->end);


    //printf("%d \n",0);//(int)T.E.size()
    /*for (int i = 0; i < T.E.size(); ++i)
    {
        printf("%f %f",T.E[i]->st->x , T.E[i]->st->y);
        printf(" %f %f\n",T.E[i]->end->x , T.E[i]->end->y);
    }*/

    get_path(final);
    //printf("%d \n",0);
}


double h_distance (vertex * v1, vertex * v2)
{
    return sqrt( (v1->x - v2->x) * (v1->x - v2->x) + (v1->y - v2->y) * (v1->y - v2->y) + (v1->theta - v2->theta) * (v1->theta - v2->theta)   );
}


void prune (double c_best,vector<vertex *> & X_samples ,tree & T )
{
    vector<vector<vertex *>::iterator> itv;
    vector<vector<edge *>::iterator> ite, ite1;
    vector<vertex *>del;


    for (int i = 0; i < X_samples.size(); ++i)
    {
        if( h_distance(X_samples[i],x_start) + h_distance(X_samples[i],x_goal) >= c_best )
        {
            //cout<<"erasing sample \n";

            X_samples_rtree.remove(point(X_samples[i]->x, X_samples[i]->y, X_samples[i]->theta));

            itv.push_back(X_samples.begin() + i);
        }
    }
    for (int j = itv.size()-1; j >=0; --j)
    {
        X_samples.erase(itv[j]);
    }

    itv.clear();

    for (int i = 0; i < T.E.size(); ++i)
    {
        if ( T.E[i]->st->ghat + T.E[i]->st->h > c_best )
        {
            ite.push_back(T.E.begin() + i);

            //if(T.E[i]->end->ghat + T.E[i]->end->h <= c_best)
            {
                X_samples_rtree.insert(point(T.E[i]->end->x, T.E[i]->end->y, T.E[i]->end->theta));
                X_samples.push_back(T.E[i]->end);
                del.push_back(T.E[i]->end);
            }
        }

        else if ( T.E[i]->end->ghat + T.E[i]->end->h > c_best )
        {
            ite.push_back(T.E.begin() + i);
        }
    }

    for (int j = ite.size()-1; j >=0; --j)
    {
        if(j>0)
            if((*ite[j]) == (*ite[j-1]))
                continue;

        E_table.erase((*ite[j])->id);
        T.E.erase(ite[j]);
    }

    for (int i = 0; i < T.V.size(); ++i)
    {
        if ((T.V[i]->ghat + T.V[i]->h) > c_best )
        {
            //cout<<"somthing need to be erased from vertex \n";

            V_rtree.remove(point(T.V[i]->x, T.V[i]->y, T.V[i]->theta));

            V_table.erase(T.V[i]->id);

            itv.push_back(T.V.begin()+i);
        }
    }

    for (int k = 0; k < del.size(); ++k)
    {
        for (int i = 0; i < T.V.size(); ++i)
        {
            if(T.V[i]->id == del[k]->id)
            {
                //cout<<"match\n";
                V_rtree.remove(point(T.V[i]->x, T.V[i]->y, T.V[i]->theta));

                V_table.erase(T.V[i]->id);

                itv.push_back(T.V.begin()+i);
            }
        }
    }
    sort(itv.begin(),itv.end());

    for (int j = itv.size()-1; j >=0; --j)
    {
        if(j>0)
            if((*itv[j]) == (*itv[j-1]))
                continue;

        T.V.erase(itv[j]);
    }

}


lint get_id (vertex * v)
{
    lint key,y,x;

    x= hash<double >{}(v->x+v->y*sizex);

    y= hash<double >{}(v->theta);

    key =(x+y)*(x+y+1)/2+x;

    return key;

}


lint get_id (vertex * st  , vertex * end)
{
    lint key,key2,key1,y,x;

    x= hash<double >{}(st->x+st->y*sizex);

    y= hash<double >{}(st->theta);

    key1 =(x+y)*(x+y+1)/2+x;

    x= hash<double >{}(end->x+end->y*sizex);

    y= hash<double >{}(end->theta);

    key2 =(x+y)*(x+y+1)/2+x;

    x=key1;
    y=key2;
    key= (x+y)*(x+y+1)/2+x;

    return  key;

}


void sample (vector<vertex *> & X_samples,int m, double c_best )
{
    //cout<<"start sampling \n";
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

        t= new vertex;

        t->x=ran_x;
        t->y=ran_y;
        t->theta=ran_theta;

        if(c_best < inf)
        {
            if ((h_distance(t,x_start)+ h_distance(t,x_goal)) >= c_best)
                continue;
        }

        count++;

        t->id=get_id(t);

        X_samples.push_back(t);
        X_samples_rtree.insert(point(t->x,t->y,t->theta));

        //printf("%f %f %f %lu\n",t->x,t->y,t->theta,t->id);
    }
    //cout<<"done sampling \n";
}


void Expand_Vertex(vertex * v , vertex_queue & QV , edge_queue & QE , int knn ,double c_best,tree & T)
{
    QV.erase(QV.begin());

    edge * e;
    vertex * x , *w;

    unordered_map<lint, vertex *>::iterator it_v_old;

    unordered_map<lint, edge *>::iterator it_e;


    vector<point> X_near ,V_near;

    X_samples_rtree.query(bgi::nearest(point(v->x, v->y,v->theta), knn), std::back_inserter(X_near));

    //printf("X_near size = %d \n",X_near.size());

    for (int i = 0; i < X_near.size(); ++i)
    {
        x=new vertex;

        x->x=X_near[i].get<0>();
        x->y=X_near[i].get<1>();
        x->theta=X_near[i].get<2>();
        x->h =h_distance(x,x_goal);
        x->gt =(inf);
        x->ghat=h_distance(x,x_start);
        x->id=get_id(x);

        if(v->id == x->id)
        {
            //cout<<"repeted \n";
            continue;
        }

        if ( (v->ghat + h_distance(v,x) + x->h) < c_best )
        {
            //cout<<"pushing new edge";
            e= new edge;
            e->st=v;
            e->end=x;
            e->chat = h_distance(v,x);
            e->id=get_id(v,x);

            QE.push_back (e);
        }
    }

    it_v_old=V_old_table.find(v->id);

    if(it_v_old == V_old_table.end())  // not in the v_old_table
    {

        V_rtree.query(bgi::nearest(point(v->x,v->y,v->theta), knn), std::back_inserter(V_near));

        //printf("V_near size = %d \n",V_near.size());

        for (int i = 0; i < V_near.size(); ++i)
        {
            w=new vertex;

            w->x=V_near[i].get<0>();
            w->y=V_near[i].get<1>();
            w->theta=V_near[i].get<2>();
            w->ghat=h_distance(w,x_start);
            w->h=h_distance(w,x_goal);
            w->id=get_id(w);

            if(v->id == w->id)
            {
                //cout<<"repeted \n";
                continue;
            }

            for (int j = 0; j < T.V.size(); ++j)
            {
                //cout<<"1";
                if(w->id == T.V[j]->id)  //w->x == T.V[j]->x && w->y == T.V[j]->y && w->theta == T.V[j]->theta
                {
                    //cout<<"2"<<endl;
                    w->gt = T.V[j]->gt;
                    break;
                }
            }

            it_e = E_table.find(get_id(v,w));

            if (it_e == E_table.end() && ((v->ghat + h_distance(v,w) + w->h) < c_best ) && ( v->gt + h_distance(v,w) ) < w->gt  )
            {
                //cout<<"pushing new edge down";
                e= new edge;
                e->st=v;
                e->end=w;
                e->chat = h_distance(v,w);
                e->id =get_id(v,w);

                QE.push_back (e);

            }
        }

    }
}


int printConfiguration(double q[3], double x, void* user_data)
{
    double * tmp;

    tmp =new double[4];
    tmp[0]=q[0];
    tmp[1]=q[1];
    tmp[2]=q[2];
    tmp[3]=x;
    res.push_back(tmp);

    return 0;
}


bool collision_check_dubin (vertex * v , vertex * x, double & c )
{

    res.clear();
    DubinsPath path;

    double q0[3],q1[3],m,n;

    q0[0]=v->x;
    q0[1]=v->y;
    q0[2]=v->theta;

    q1[0]=x->x;
    q1[1]=x->y;
    q1[2]=x->theta;

    dubins_shortest_path(&path, q0, q1, min_r);

    dubins_path_sample_many(&path,0.1, printConfiguration, NULL);

    for (int i = 0; i < res.size(); ++i)
    {

        m=res[i][0];
        n=res[i][1];

        if ((int) m > sizex -.0001 || m < 0 || (int) n > sizey -.0001 || n < 0)
        {
            c=(inf);
            //cout<<"\n \\\\\\\\\\\\\\\\\\\\\\\\\\t  first \\\\\\\\\\\\\\\\\\\\\\\\\\\\ \n";
            return false;
        }

        if (Map[int(m)][int(n)] == 1)
        {
            c=(inf);
            //cout<<"\n \\\\\\\\\\\\\\\\\\\\\\\\\\t  second \\\\\\\\\\\\\\\\\\\\\\\\\\\\ \n";
            return false;
        }
    }
    c=dubins_path_length (&path);
    return true;
}

double Best_queue_value (vertex_queue & QV)
{
    if(QV.empty())
        return inf* inf ;

    auto top = QV[0];
    return top->gt + top->h;

}


double Best_queue_value (edge_queue & QE)
{
    if(QE.empty())
        return inf;

    auto top = QE[0];

    return top->st->gt +top->chat + top->end->h;
}


void get_path (vector<vertex *> v)
{
    DubinsPath path;
    int count=0;



    double q0[3], q1[3];
    for (int i = 0; i < v.size()-1; ++i)
    {
        res.clear();

        q0[0] = v[i]->x;
        q0[1] = v[i]->y;
        q0[2] = v[i]->theta;

        q1[0] = v[i+1]->x;
        q1[1] = v[i+1]->y;
        q1[2] = v[i+1]->theta;

        dubins_shortest_path(&path, q0, q1, min_r);
        dubins_path_sample_many(&path, 0.1, printConfiguration, NULL);
        count+= res.size();
    }
    out<<count<<endl;
    for (int i = 0; i < v.size()-1; ++i)
    {
        res.clear();

        q0[0] = v[i]->x;
        q0[1] = v[i]->y;
        q0[2] = v[i]->theta;

        q1[0] = v[i+1]->x;
        q1[1] = v[i+1]->y;
        q1[2] = v[i+1]->theta;

        dubins_shortest_path(&path, q0, q1, min_r);

        dubins_path_sample_many(&path, 0.1, printConfiguration, NULL);

        for (int j = 0; j < res.size(); ++j)
        {
            out << to_string(res[j][0])<<" "<<to_string(res[j][1])<<endl;
            //printf("%f %f \n", res[j][0], res[j][1]);
        }
    }
    out<<"0\n";
    //data.close();
}


void disp(vertex* v )
{
    printf("x= %f , y= %f , theta = %f , id = %lu\n",v->x,v->y,v->theta,v->id);
}


void disp(edge* e )
{
    disp(e->st);
    disp(e->end);
    printf("id= %lu\n",e->id);
}