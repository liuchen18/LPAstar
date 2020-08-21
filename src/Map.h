#ifndef MAP_H
#define MAP_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <math.h>
#include <memory>

using std::vector;
using std::cout;
using std::endl;
using std::shared_ptr;

const double infi = std::numeric_limits<int>::max();

enum class Status{
    path,obstacle,road,start,end
};

struct Node{
    shared_ptr<Node> parent;
    int x;
    int y;
    int g;
    int rhs;
    Status status;
    /**
     * @brief constructor of Node
     * @param x the x position of the new node
     * @param y the y position of the new node
     */
    Node(int x, int y): x(x), y(y), status(Status::road),g(infi),rhs(infi),parent(nullptr){}

    /**
     * @brief compute the cost from the current node to another node
     * @param other another node
     * @return x+y
     */
    double cost(shared_ptr<Node> other){
        if(this->status==Status::obstacle||other->status==Status::obstacle){
            return infi;
        }
        return abs(this->x-other->x)+abs(this->y-other->y);
    }
};

/*----------------------------------start of Map------------------------------------------*/

class Map{
public:
    /**
     * @brief constructor
     * @param row row of the map
     * @param col colum of the map
     */
    Map(int row, int col):row(row),col(col){
        myMap = Init();
    }

    /**
     * @brief get row of the map
     * @return
     */
    int getRow() const {return row;}

    /**
     * @bried get the colum of the map
     * @return
     */
    int getCol() const { return col;}

    /**
     * @brief print the map
     */
    void printMap();

    /**
     * @brief set the points to be the obstacle
     * @param points the vector that contains the points to be set to obstacle
     */
    void setObstacle(vector<vector<int>> &points);

    /**
     * @brief set the given position to path
     * @param x
     * @param y
     */
    void setPath(int x, int y);

    /**
     * @bried set the given position to the start
     * @param x
     * @param y
     * @return the pointer to the start int the map
     */
    shared_ptr<Node> setStart(int x, int y);

    /**
     * @brief set the given position to the end
     * @param x
     * @param y
     * @return the pointer to the end int the map
     */
    shared_ptr<Node> setEnd(int x, int y);

    /**
     *
     * @param x
     * @param y
     * @return
     */
    shared_ptr<Node> getNode(int x, int y) const { return myMap[x][y];}

    /**
     * @brief get the neighbor of the current node
     * @param curr
     * @return
     */
    vector<shared_ptr<Node>> getNeighbor(shared_ptr<Node> curr);

    /**
     *@brief set the path to road
     */
    void clearPath();
    void checkMapTransistioncost();
    
private:
    int row; //the row of the map
    int col; //the colum of the map
    vector<vector<shared_ptr<Node>>> myMap; //the map, contained by the nodes

    /**
     * @bried init the map, ask for memory
     * @return map
     */
    vector<vector<shared_ptr<Node>>> Init();
    const vector<vector<int>> dirs = {{-1,0},{1,0},{0,-1},{0,1}}; //the moving direction
    
};

#endif