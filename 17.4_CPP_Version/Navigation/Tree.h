#pragma once
#include "Link.h"
#include "Goal.h"
#include <list>
#include <cmath>

class Tree{
    public:
    Node start;
    Goal goal;
    list<Link> links;
    Tree(vector<double> startPoint, vector<double> goalPoint);
    Tree();
    void addLink(Link newLink);
    bool goalReached();
};