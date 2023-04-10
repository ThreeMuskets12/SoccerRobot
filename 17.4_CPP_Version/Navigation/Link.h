#pragma once
#include "Node.h"

class Link{
    public:
    Node parent;
    Node child;
    Link(Node p,Node c);
    // // helper function adds node to the this node's list of children
    // void addNode(Node n);
};