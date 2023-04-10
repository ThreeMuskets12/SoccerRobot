#pragma once
#include "Node.h"

class Goal:public Node{
    public:
    vector<double> point={};
    double radius=0.03;
    Goal(vector<double> point);
    Goal();
};