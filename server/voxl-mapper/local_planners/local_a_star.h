#ifndef LOCAL_A_STAR_H_
#define LOCAL_A_STAR_H_

#include <mav_path_smoothing/loco_smoother.h>
#include "voxl_mapper.h"
#include <stdio.h>
#include <thread>

#include "local_planner.h"

class LocalAStar : public LocalPlanner
{
public:
    LocalAStar(voxblox::TsdfServer *map, int plan_ch, int render_ch);
    ~LocalAStar();

    void setPlan(const Point3fVector &waypoints);
    void setup();
    void start();
    void stop();

private:

    typedef struct Node
    {
        float total_cost;
        float travel_cost;
        float heuristic_cost;
        voxblox::GlobalIndex esdf_idx;
        Node *parent;
        bool closed;
        int steps;
        int idx_in_parent = -1;

        bool operator<(const Node &b)
        {
            return total_cost > b.total_cost;
        }

        Node(float total_cost, float travel_cost, float heuristic_cost, const voxblox::GlobalIndex &esdf_idx, Node *parent)
            : total_cost(total_cost),
              travel_cost(travel_cost),
              heuristic_cost(heuristic_cost),
              esdf_idx(esdf_idx),
              parent(parent),
              closed(false)
        {
            if(parent)
                steps = parent->steps + 1;
            else
                steps = 0;
        }
    } Node;

    struct Compare
    {
        bool operator()(Node *lhs, Node *rhs)
        {
            return ((lhs->total_cost) >= (rhs->total_cost));
        }
    };

    enum class PlannerReturnValue {
        Success,
        SmootherFailed,
        MaxIterationsReached
    };

    /**
     * @brief Thread that runs the local planning logic
     * 
     */
    void plannerThread();

    /**
     * @brief The first plan has slightly different characteristics -
     * i.e. need to plan from start point, has zero vel/acc, has a turn in
     * place step so that sensors can see in direction we want to move
     * 
     * @return true if a plan was found
     * @return false if no plan was found
     */
    bool getInitialPlan();

    /**
     * @brief Calculates angle needed to rotate to in order
     * to face along direction vector and sends message to vvpx4
     * 
     * @param cur_pos current position of robot
     * @param direction direction vector for robot to face
     */
    void rotateToFace(Point3f &cur_pos, Point3f &direction);

    /**
     * @brief Runs A* planning to find a valid path from start to goal.
     * Also contains a pruning step and smoothing step and visualization
     * of the A* path found
     * 
     * @param start_pos start position of plan
     * @param goal_pos goal position of plan
     * @param trajectory the output trajectory
     * @return PlannerReturnValue describes the outcome of the planner
     */
    PlannerReturnValue runPlanner(const Point3f &start_pos, const Point3f &goal_pos, mav_trajectory_generation::Trajectory *trajectory);

    /**
     * @brief Runs the loco smoother on a set of waypoints
     * 
     * @param target_points set of waypoints to find a smooth path through
     * @param trajectory the output trajectory
     * @return true if smoother ran succesfully
     * @return false if smoother was unable to find a suitable trajectory
     */
    bool runSmoother(const Point3fVector &target_points,
                     mav_trajectory_generation::Trajectory *trajectory);

    /**
     * @brief Converts trajectory to a format that vvpx4 will accept. This function
     * also maintains the planners view of the current trajectory (current_traj_)
     * 
     * @param trajectory the trajectory to be sent
     * @param split_id segment id for where to insert the trajectory
     * @param split_time segment time for where to insert the trajectory
     * @param first_plan whether trajectory is the first plan or not
     * @return true if succesfully converted and sent trajectory
     * @return false otherwise
     */
    bool convertAndSendTrajectory(
        const mav_trajectory_generation::Trajectory &trajectory, int split_id, double split_time, bool first_plan = false);

    /**
     * @brief Sends visualization of the trajectory to voxl-portal
     * 
     */
    void visualizeTrajectory();

    /**
     * @brief Sends visualization of the target points to voxl-portal
     * 
     * @param target_points points to visualize
     */
    void visualizeAStarPath(const Point3fVector &target_points);

    /**
     * @brief Computes where the split point occurs based on PLAN_AHEAD_TIME
     * 
     * @param start_pose starting point of split
     * @param split_id id of segment to split at
     * @param split_time segment time to split at
     * @return true if split point succesfully calculated
     * @return false if we could not find the segment id that vvpx4 is evaluating
     * withint current_traj
     */
    bool computeSplitPointDetails(Point3f *start_pose, int *split_id, double *split_time);

    /**
     * @brief Computes the heuristic for A*. Currently uses the Octile heuristic which is
     * an exact heuristic for 3D volumetric spaces. Basically, do not change this heuristic
     * you are guaranteed to make the A* search worse if you do.
     * 
     * @param cur_idx global index of current voxel
     * @param goal_idx global index of goal voxel
     * @return float octile distance between current and goal
     */
    float computeHeuristic(voxblox::GlobalIndex cur_idx, voxblox::GlobalIndex goal_idx);

    /**
     * @brief Checks whether the projected points c lies on the line between
     * point a and b
     * 
     * @param a start of line segment
     * @param b end of line segment
     * @param c point to check
     * @return true if c lies between on line segment ab
     * @return false if c lies outside the line segment ab
     */
    bool isBetween(Point3f a, Point3f b, Point3f c);

    /**
     * @brief Prunes the resulting A* path. Makes it easier to be processed
     * by the smoother
     * 
     * @param path the A* path to prune
     */
    void pruneAStarPath(std::vector<Node *> *path);

    /**
     * @brief Computes a goal position from the global waypoints. Finds the closest
     * point on the global waypoint path, then moves along it by
     * PLANNING_HORIZON distance. If the goal point is in collision then it steps
     * back along the path to find one that is not in collision.
     * 
     * @param start_pos start point to search for the goal position
     * @param goal_pos output goal position
     * @return true if a suitable goal position was found
     * @return false if a suitable goal position could not be found
     */
    bool computeGoalPosition(const Point3f& start_pos, Point3f *goal_pos);

    Point3fVector waypoints_;
    mav_planning::LocoSmoother loco_smoother_;

    voxblox::TsdfServer* map_;

    std::thread planning_thread_;
    std::atomic<bool> running_;

    trajectory_t current_traj_;

    Point3f start_vel;
    Point3f start_acc;

    int plan_ch_;
    int render_ch_;

    bool initial_plan_;

    bool en_debug_ = false;
};

#endif