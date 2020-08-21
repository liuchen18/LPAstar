#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <math.h>
#include <memory>
#include "Map.h"

using std::pair;

class LPAStar{
public:
    /**
     * @brief costructor
     * @param startx x position of the start
     * @param starty y position of the start
     * @param endx x position of the end
     * @param endy y position of the end
     * @param m the given map
     */
    LPAStar(int startx, int starty, int endx, int endy, Map m):roadMap(m),startx(startx),starty(starty),endx(endx),endy(endy){
        start = roadMap.setStart(startx,starty);
        end = roadMap.setEnd(endx, endy);
    }

    /**
     * @brief set the rhs of the start to 0 and push it to the open list
     */
    void inti();

    /**
     * @brief the plan function of the LPA STAR
     */
    void planner();

    /**
     * @brief get the path and mark the path  in map
     * @return trajectory
     */
    vector<pair<int,int>> constructPath();

    /**
     * @brief set the obstacles to be the obstacle in the map and update the neighbors of the obstacles
     * @param obstacles
     */
    void costChange(vector<vector<int>> &obstacles);

    /**
     *
     */
    void getMapInfo(){roadMap.printMap();}

    /**
     *
     */
    void clearPath() {roadMap.clearPath();};

    /**
     *
     * @return
     */
    bool checkPath() const {return ableFindPath;}
private:
    Map roadMap;
    int startx;
    int starty;
    int endx;
    int endy;
    bool ableFindPath = true;
    shared_ptr<Node> start;
    shared_ptr<Node> end;
    vector<shared_ptr<Node>> openList;

    /**
     * @brief compute the rhs of the given node. min(neighbor.g+cost(neighbor,current))
     * @param curr
     */
    void computeRHS(shared_ptr<Node> curr);

    /**
     * @brief the heuristic function
     * @param curr
     * @return
     */
    int computeH(shared_ptr<Node> curr);

    /**
     * @brief remove the node from the openlist
     * @param node the node to be removed
     */
    void remove(shared_ptr<Node> node);

    /**
     * @brief the key of the given node
     * @param node
     * @return return the key,[min(g,rhs) + h, min(g,rhs)]
     */
    pair<int,int> calculateKey(shared_ptr<Node> node);

    /**
     * @brief update the given node.
     * @param curr
     */
    void updateVertex(shared_ptr<Node> curr);

    /**
     * @brief insert the node to the openlist and sort the openlist
     * @param node
     */
    void insert(shared_ptr<Node> node);

    /**
     * @brief compare the two node
     * @param n1
     * @param n2
     * @return true if n1 < n2.
     */
    bool compare(shared_ptr<Node> n1, shared_ptr<Node> n2);
    
};


#endif