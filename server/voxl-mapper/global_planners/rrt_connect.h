#ifndef RRT_CONNECT_H
#define RRT_CONNECT_H

#include <config_file.h>
#include "voxl_mapper.h"
#include <modal_pipe.h>
#include <modal_pipe_interfaces.h>
#include "octree.hpp"
#include "timer.h"
#include "global_planner.h"

#include <Eigen/Core>

class RRTConnect : public GlobalPlanner
{

public:
    RRTConnect(voxblox::TsdfServer *map, int vis_channel);
    ~RRTConnect();

    bool createPlan(const Point3f &start_pos, const Point3f &end_pos, Point3fVector &waypoints);

    void tearDown();

private:
    typedef struct Node
    {
        Point3f position;
        Node *parent;
        std::vector<Node *> children;
        int id;
        float cost;
    } Node;

    /**
     * @brief Runs the RRT algorithm to find a feasible path from start to end
     * 
     * @param start_pos starting position for RRT
     * @param end_pos ending position for RRT
     * @param waypoints output path if RRT runs succesfully
     * @return true if RRT succesfully finds a path
     * @return false if RRT was unable to find a path
     */
    bool runRRT(const Point3f &start_pos, const Point3f &end_pos, Point3fVector &waypoints);

    bool runInformedRRT(const Point3f &start_pos, const Point3f &end_pos, Point3fVector &waypoints);

    /**
     * @brief Determines if the RRT planner can even run by checking that the start
     * and end position are not in occupied space
     * 
     * @param start_pos start position for planning
     * @param end_pos end position for planning
     * @return true if both start and end are in unoccupied space
     * @return false if either start or end is in occupied space
     */
    bool isPlanningFeasible(const Point3f &start_pos, const Point3f &end_pos);

    /**
     * @brief Determines if the straight line from start to end is unoccupied
     * 
     * @param start_pos start position for planning
     * @param end_pos end position for planning
     * @return true if the straight line from start to end is unoccupied
     * @return false if the straight line from start to end is in collision
     */
    bool checkImmediatePath(const Point3f &start_pos, const Point3f &end_pos);

    /**
     * @brief Computes the map bounds and initializes the map variables
     *
     * @return true the ESDF map exists and map variables were able to be computed
     * @return false ESDF map does not exist
     */
    bool computeMapBounds();

    /**
     * @brief Helper function to create a new node
     *
     * @param position 3D point of node
     * @param parent parent of node
     * @return Node* newly created node
     */
    Node *createNewNode(const Point3f &position, Node *parent);

    /**
     * @brief Creates a new node at a random location inside map bounds
     *
     * @return Node* new node with a random position
     */
    Node *createRandomNode();

    Node *createRandomNodeInEllipsoid(const Eigen::Matrix3f &C, const Eigen::Matrix3f &L, const Point3f &center);

    /**
     * @brief Returns nearest node to point
     *
     * @param point position to find the nearest node to
     * @return std::pair<Node *, float> the nearest node and distance to node
     */
    std::pair<Node *, float> findNearest(const Point3f &point);

    void findNearestInRadius(const Point3f &point, float radius, std::vector<Node *> &nearest_nodes, std::vector<float> &distances);

    /**
     * @brief Add a node to the RRT tree
     *
     * @param q_nearest the parent node
     * @param q_new the new node to add
     */
    void add(Node *q_nearest, Node *q_new);

    /**
     * @brief Cleanup nodes by recursively traversing nodes and deleting
     *
     * @param root the starting point of the RRT tree
     */
    void deleteNodes(Node *&root);

    /**
     * @brief Cleanup the RRT tree and all associated search structures
     *
     */
    void cleanupTree();

    /**
     * @brief Prunes the found RRT path in two levels.
     *
     * Level 1: Pick two random edges and two random points along those edges.
     *          Attempt to connect these two points. If collision free then
     *          remove all nodes along the path between these two points to
     *          create a shorter path. If not collision free, pick again.
     *          Run this for as many times as desired.
     *
     * Level 2: Iterate over the path from start to end. For each node check
     *          all other nodes from end to start to see if the path is collision
     *          free. If collision free then remove any nodes that this would skip.
     *
     *
     * Level 1 helps to create an overall shorter path. Level 2 helps to "thin out"
     * the path created by level 1's randomness. Level 2 also helps to further
     * shorten the path.
     */
    void pruneRRTPath(std::vector<Node *> &rrt_path);

    /**
     * @brief Convert from the internal struct used by RRT to the expected
     * output used by GlobalPlanner
     * 
     * @param rrt_path the internal struct used to hold the path
     * @param waypoints the resulting path expected by the GlobalPlanner interface
     */
    void convertPathToOutput(const std::vector<Node *> &rrt_path, Point3fVector &waypoints);

    /**
     * @brief Deletes intermediate nodes created by Level 1 pruning
     *
     */
    void cleanupPruning();

    /**
     * @brief Sends RRT path points to voxl-portal
     *
     */
    void visualizePath(const Point3fVector &waypoints);

    /**
     * @brief Sends tree to voxl-portal
     *
     */
    void visualizeTree();

    voxblox::TsdfServer* map_;
    Point3f lower_bound_;
    Point3f upper_bound_;

    std::vector<Node *> pruning_nodes_;
    Node *root_;
    int node_counter_;

    int vis_channel_;

    Timer timer;

    // Need to keep points for octree alive with octree
    // (otherwise we would need to set copyPoints to true in octree params)
    unibn::Octree<Point3f> octree_;
    std::vector<Node *> nodes_;
    std::vector<Point3f> points_;
    int last_create_idx_;

    bool debug = false;
};

#endif