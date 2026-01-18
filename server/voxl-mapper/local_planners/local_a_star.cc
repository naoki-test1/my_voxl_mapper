#include "local_a_star.h"

#include "voxl_mapper.h"
#include "rc_transform_ringbuf.h"
#include "timing_utils.h"
#include "voxl_trajectory/voxl_trajectory.h"
#include "mav_planning_msgs/mav_planning_msgs.h"
#include "mav_local_planner/conversions.h"
#include "mav_trajectory_generation/trajectory_sampling.h"
#include "mav_trajectory_generation/trajectory_sampling.h"
#include "path_vis.h"

#define LOCAL_PLANNER_RATE 2
#define REACHED_DISTANCE 0.05f
#define PLAN_AHEAD_TIME 0.5
#define PLANNING_HORIZON 1.5f
#define MAX_ITERATIONS 10000

#define PLAN_NAME "plan_msgs"
#define PLAN_LOCATION (MODAL_PIPE_DEFAULT_BASE_DIR PLAN_NAME "/")

LocalAStar::LocalAStar(voxblox::TsdfServer *map, int plan_ch, int render_ch)
    : map_(map),
      running_(false),
      current_traj_({}),
      plan_ch_(plan_ch),
      render_ch_(render_ch),
      initial_plan_(true)
{
}

bool LocalAStar::convertAndSendTrajectory(const mav_trajectory_generation::Trajectory &trajectory, int split_id, double split_time, bool first_plan)
{
    // Segment id is 0 on first plan otherwise add one to the last segment id in the current trajectory
    int segment_id = 0;

    if (!first_plan)
    {
        segment_id = current_traj_.segments[current_traj_.n_segments - 1].id + 1;
    }

    trajectory_t out;

    if (!convertMavTrajectoryToVoxlTrajectory(trajectory, out))
        return false;

    out.magic_number = TRAJECTORY_MAGIC_NUMBER;

    // Set segment id for each segment whilst incrementing id
    for (int i = 0; i < out.n_segments; i++)
        out.segments[i].id = segment_id++;

    if (first_plan)
    {
        out.traj_command = TRAJ_CMD_LOAD_AND_START;
        current_traj_ = out;
    }
    else
    {
        if (split_id < 0 || split_time < 0)
        {
            fprintf(stderr, "Invalid split id or split time\n");
            fprintf(stderr, "split id = %d | split time = %f\n", split_id, split_time);
            return false;
        }

        out.traj_command = TRAJ_CMD_INSERT;
        out.segment_split_id = split_id;
        out.segment_split_time = split_time;

        // Trim to make sure we eliminate unneccessary segments
        trim_trajectory(&current_traj_, cur_segment_id_);

        // Attempt to insert, only send if successful
        if (!insert_trajectory(&current_traj_, &out))
            return false;
    }

    printf("Sending trajectory to plan channel\n");
    pipe_server_write(plan_ch_, &out, sizeof(trajectory_t));

    visualizeTrajectory();

    return true;
}

void LocalAStar::visualizeAStarPath(const Point3fVector &target_points)
{
    std::vector<path_vis_t> waypoints;
    for (const Point3f &p : target_points)
    {
        path_vis_t pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        pt.r = 0;
        pt.g = 255;
        pt.b = 0;
        waypoints.push_back(pt);
    }

    generateAndSendPath(render_ch_, waypoints, PATH_VIS_LINE, "LocalAStar Waypoints");
}

void LocalAStar::visualizeTrajectory()
{
    std::vector<path_vis_t> traj;
    double sampling_dt = 0.05;

    for (int i = 0; i < current_traj_.n_segments; i++)
    {
        poly_segment_t *s = &current_traj_.segments[i];
        for (double t = 0.0; t < s->duration_s; t += sampling_dt)
        {
            path_vis_t pt;
            pt.x = eval_poly_at_t(s->n_coef, s->cx, t);
            pt.y = eval_poly_at_t(s->n_coef, s->cy, t);
            pt.z = eval_poly_at_t(s->n_coef, s->cz, t);
            pt.r = 0;
            pt.g = 0;
            pt.b = 255;
            traj.push_back(pt);
        }
    }

    generateAndSendPath(render_ch_, traj, PATH_VIS_TRAJECTORY, "LocalAStar Smoothed");
}

bool LocalAStar::runSmoother(const Point3fVector &target_points, mav_trajectory_generation::Trajectory *trajectory)
{
    mav_msgs::EigenTrajectoryPointVector waypoints_for_smoother;
    convertPointsToSmootherFormat(target_points, waypoints_for_smoother);

    // Set the velocity and acceleration
    waypoints_for_smoother.front().velocity_W = start_vel.cast<double>();
    waypoints_for_smoother.front().acceleration_W = start_acc.cast<double>();

    int num_segments = static_cast<size_t>(loco_num_segments) < target_points.size() ? loco_num_segments : target_points.size();
    loco_smoother_.setNumSegments(num_segments);

    return loco_smoother_.getTrajectoryBetweenWaypoints(waypoints_for_smoother, trajectory, false);
}

void LocalAStar::setup()
{
    setupSmootherFromConfig(loco_smoother_, map_->getEsdfMapPtr().get());
}

void LocalAStar::start()
{
    // Ignore request to start if already running, otherwise check if previous thread needs cleanup
    if (running_)
        return;
    else if (planning_thread_.joinable())
        planning_thread_.join();

    // Reset the trajectory ids
    pthread_mutex_lock(&segment_mutex_);
    cur_segment_id_ = 0;
    cur_segment_t_ = 0;
    pthread_mutex_unlock(&segment_mutex_);

    running_ = true;
    initial_plan_ = true;
    planning_thread_ = std::thread(&LocalAStar::plannerThread, this);
}

void LocalAStar::stop()
{
    if (!running_)
        return;

    running_ = false;

    if (planning_thread_.joinable())
        planning_thread_.join();
}

bool LocalAStar::computeSplitPointDetails(Point3f *start_pos, int *split_id, double *split_time)
{
    // Lock and copy the segment variables
    pthread_mutex_lock(&segment_mutex_);
    double cur_segment_t = cur_segment_t_;
    int cur_segment_id = cur_segment_id_;
    pthread_mutex_unlock(&segment_mutex_);

    int segment_idx = get_segment_idx(&current_traj_, cur_segment_id);

    if (segment_idx < 0)
    {
        fprintf(stderr, "ERROR: Segment id %d does not exist in current trajectory. Planning from end of trajectory\n", cur_segment_id);
        return false;
    }

    // Now we need to calculate the time point to start at. We do this by moving PLAN_AHEAD_TIME seconds
    // along the polynomial, starting at the most recently evaluate time point
    double t = current_traj_.segments[segment_idx].duration_s - cur_segment_t;

    // If the split happens on this segment then find the start point and split time
    if (t >= PLAN_AHEAD_TIME)
    {
        *split_time = cur_segment_t + PLAN_AHEAD_TIME;
        *split_id = cur_segment_id;

        poly_segment_t *s = &current_traj_.segments[segment_idx];
        float x = eval_poly_at_t(s->n_coef, s->cx, *split_time);
        float y = eval_poly_at_t(s->n_coef, s->cy, *split_time);
        float z = eval_poly_at_t(s->n_coef, s->cz, *split_time);

        float vx = eval_vel_at_t(s->n_coef, s->cx, *split_time);
        float vy = eval_vel_at_t(s->n_coef, s->cy, *split_time);
        float vz = eval_vel_at_t(s->n_coef, s->cz, *split_time);

        float ax = eval_accel_at_t(s->n_coef, s->cx, *split_time);
        float ay = eval_accel_at_t(s->n_coef, s->cy, *split_time);
        float az = eval_accel_at_t(s->n_coef, s->cz, *split_time);

        *start_pos << x, y, z;
        start_vel << vx, vy, vz;
        start_acc << ax, ay, az;
        return true;
    }

    // Otherwise step through the following segments to find where the split point occurs
    for (int i = segment_idx + 1; i < current_traj_.n_segments; i++)
    {
        t += current_traj_.segments[i].duration_s;

        if (t > PLAN_AHEAD_TIME)
        {
            t -= current_traj_.segments[i].duration_s;

            *split_id = current_traj_.segments[i].id;
            *split_time = PLAN_AHEAD_TIME - t;

            poly_segment_t *s = &current_traj_.segments[i];
            float x = eval_poly_at_t(s->n_coef, s->cx, *split_time);
            float y = eval_poly_at_t(s->n_coef, s->cy, *split_time);
            float z = eval_poly_at_t(s->n_coef, s->cz, *split_time);

            float vx = eval_vel_at_t(s->n_coef, s->cx, *split_time);
            float vy = eval_vel_at_t(s->n_coef, s->cy, *split_time);
            float vz = eval_vel_at_t(s->n_coef, s->cz, *split_time);

            float ax = eval_accel_at_t(s->n_coef, s->cx, *split_time);
            float ay = eval_accel_at_t(s->n_coef, s->cy, *split_time);
            float az = eval_accel_at_t(s->n_coef, s->cz, *split_time);

            *start_pos << x, y, z;
            start_vel << vx, vy, vz;
            start_acc << ax, ay, az;
            return true;
        }
    }

    // We will only reach here if PLAN_AHEAD_TIME occurs past end of the current trajectory
    // so the return the last segment end point
    poly_segment_t *last_segment = &current_traj_.segments[current_traj_.n_segments - 1];
    float x = eval_poly_at_t(last_segment->n_coef, last_segment->cx, last_segment->duration_s);
    float y = eval_poly_at_t(last_segment->n_coef, last_segment->cy, last_segment->duration_s);
    float z = eval_poly_at_t(last_segment->n_coef, last_segment->cz, last_segment->duration_s);

    float vx = eval_vel_at_t(last_segment->n_coef, last_segment->cx, last_segment->duration_s);
    float vy = eval_vel_at_t(last_segment->n_coef, last_segment->cy, last_segment->duration_s);
    float vz = eval_vel_at_t(last_segment->n_coef, last_segment->cz, last_segment->duration_s);

    float ax = eval_accel_at_t(last_segment->n_coef, last_segment->cx, last_segment->duration_s);
    float ay = eval_accel_at_t(last_segment->n_coef, last_segment->cy, last_segment->duration_s);
    float az = eval_accel_at_t(last_segment->n_coef, last_segment->cz, last_segment->duration_s);

    *start_pos << x, y, z;
    start_vel << vx, vy, vz;
    start_acc << ax, ay, az;
    *split_id = last_segment->id;
    *split_time = last_segment->duration_s;

    return true;
}

bool LocalAStar::getInitialPlan()
{
    // Zero out start velocity and acceleration
    start_vel.setZero();
    start_acc.setZero();

    // Get start and end positions - since first plan use current as start
    rc_tf_t tf_body_wrt_fixed = RC_TF_INITIALIZER;
    if (!map_->getRobotPose(tf_body_wrt_fixed, rc_nanos_monotonic_time()))
        return false;

    Eigen::Matrix3f rot;
    rot << tf_body_wrt_fixed.d[0][0], tf_body_wrt_fixed.d[0][1], tf_body_wrt_fixed.d[0][2],
        tf_body_wrt_fixed.d[1][0], tf_body_wrt_fixed.d[1][1], tf_body_wrt_fixed.d[1][2],
        tf_body_wrt_fixed.d[2][0], tf_body_wrt_fixed.d[2][1], tf_body_wrt_fixed.d[2][2];

    Point3f cur_pos(tf_body_wrt_fixed.d[0][3], tf_body_wrt_fixed.d[1][3], tf_body_wrt_fixed.d[2][3]);
    Point3f goal_pos;

    if (!computeGoalPosition(cur_pos, &goal_pos))
    {
        fprintf(stderr, "Failed to find valid goal point for initial plan");
        return false;
    }

    mav_trajectory_generation::Trajectory trajectory;

    // Only care if theres a success
    if (runPlanner(cur_pos, goal_pos, &trajectory) != PlannerReturnValue::Success)
        return false;

    // Check if vector between point 0 -> 1 is same as current direction, otherwise
    // insert an additional in place rotation
    Point3f robot_dir = rot * Point3f(1, 0, 0);
    robot_dir[2] = 0; // Remove z component
    robot_dir.normalize();

    const mav_trajectory_generation::Segment::Vector &segments = trajectory.segments();
    Point3f plan_dir = segments[0].evaluate(0.1, 1).cast<float>();
    plan_dir[2] = 0; // Remove z component
    plan_dir.normalize();

    float angle = acos(robot_dir.dot(plan_dir));

    // If the angle between the direction we are facing and direction we need to move is
    // large enough just rotate in place and let the planner run again. Otherwise send plan.
    if (angle >= static_cast<float>(M_PI / 6))
    {
        rotateToFace(cur_pos, plan_dir);
        nanosleep_for(2e9); // Sleep for a second
        initial_plan_ = true;
        return true;
    }

    if (!convertAndSendTrajectory(trajectory, -1, -1, true))
    {
        fprintf(stderr, "Local Planner failed to send initial trajectory\n");
        return false;
    }

    initial_plan_ = false;
    return true;
}

void LocalAStar::rotateToFace(Point3f &cur_pos, Point3f &direction)
{
    float rotation_angle = atan2(direction[1], direction[0]);
    setpoint_position_t msg;
    msg.magic_number = SETPOINT_POSITION_MAGIC_NUMBER;
    msg.coordinate_frame = MAV_FRAME_LOCAL_NED;
    msg.type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE |
                    POSITION_TARGET_TYPEMASK_VY_IGNORE |
                    POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_AX_IGNORE |
                    POSITION_TARGET_TYPEMASK_AY_IGNORE |
                    POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    msg.position[0] = cur_pos[0];
    msg.position[1] = cur_pos[1];
    msg.position[2] = cur_pos[2];
    msg.yaw = rotation_angle;

    // Allocate the memory
    fprintf(stderr, "Yawing by %f to face %f, %f, %f\n", (double)rotation_angle, (double)direction[0], (double)direction[1], (double)direction[2]);
    pipe_server_write(plan_ch_, &msg, sizeof(setpoint_position_t));
}

bool LocalAStar::computeGoalPosition(const Point3f &start_pos, Point3f *goal_pos)
{
    int min_idx = 0;
    float min_dist = std::numeric_limits<float>::max();
    Point3f closest_point = waypoints_.back();

    // Find the closest point on the global path
    for (size_t i = 0; i < waypoints_.size() - 1; i++)
    {
        Point3f p1 = waypoints_[i];
        Point3f p2 = waypoints_[i + 1];

        Point3f line = p2 - p1;
        Point3f unit_line = line.normalized();
        float scalar_projection = (start_pos - p1).dot(unit_line);
        float t = std::fmin(std::fmax(scalar_projection / line.norm(), 0.0f), 1.0f);

        Point3f projected_point = p1 + t * line;
        float d = (projected_point - start_pos).squaredNorm();

        if (d < min_dist)
        {
            min_dist = d;
            min_idx = i;
            closest_point = projected_point;
        }
    }

    // Once we have the closest point on the global path,
    // compute a position that is PLANNING_HORIZON ahead on it
    Point3f cur_wp = closest_point;
    Point3f target_wp = waypoints_[min_idx + 1];

    float total_dist = 0;
    int idx_of_target = min_idx + 1;

    for (size_t i = min_idx; i < waypoints_.size() - 1; i++)
    {
        target_wp = waypoints_[i + 1];
        total_dist += (cur_wp - target_wp).norm();
        idx_of_target = i + 1;

        if (total_dist >= PLANNING_HORIZON)
        {
            break;
        }

        cur_wp = target_wp;
    }

    if (total_dist >= PLANNING_HORIZON)
        target_wp += (cur_wp - target_wp).normalized() * (total_dist - PLANNING_HORIZON);

    // If no collision at target then return the point
    if (!isInCollision(map_->getEsdfMapPtr().get(), target_wp))
    {
        *goal_pos = target_wp;
        return true;
    }
    else
    {
        // Step back along the waypoints until we find something free
        for (int i = idx_of_target - 1; i >= min_idx; i--)
        {
            Point3f vec = (waypoints_[i] - target_wp);
            Point3f dir_vec = vec.normalized() * voxel_size;

            int num_of_steps = floor(vec.norm() / voxel_size);

            // Shift along the edge between two waypoints searching for free space
            for (int j = 0; j < num_of_steps; j++)
            {
                target_wp += dir_vec;

                // Check to make sure we dont go backwards from the closest point
                if (i == min_idx)
                {
                    // target_wp is the end of the edge and waypoints[i] is the start of the edge
                    Point3f to_target = (target_wp - closest_point).normalized();
                    Point3f to_waypoint = (waypoints_[i] - closest_point).normalized();

                    // If to_target and to_waypoint are in the same direction then
                    // our current target is moving backwards
                    if (to_target.dot(to_waypoint) > 0)
                        return false;
                }

                if (!isInCollision(map_->getEsdfMapPtr().get(), target_wp))
                {
                    *goal_pos = target_wp;
                    return true;
                }
            }

            target_wp = waypoints_[i];
        }
    }

    return false;
}

LocalAStar::PlannerReturnValue LocalAStar::runPlanner(const Point3f &start_pos, const Point3f &goal_pos, mav_trajectory_generation::Trajectory *trajectory)
{
    pthread_mutex_lock(&map_->esdf_mutex);
    const voxblox::Layer<voxblox::EsdfVoxel> *layer = map_->getEsdfMapPtr()->getEsdfLayerConstPtr();

    // Calculate start and goal idx
    voxblox::GlobalIndex start_idx = voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>(start_pos, 1.0f / voxel_size);
    voxblox::GlobalIndex goal_idx = voxblox::getGridIndexFromPoint<voxblox::GlobalIndex>(goal_pos, 1.0f / voxel_size);

    std::priority_queue<Node *, std::vector<Node *>, Compare> open;
    voxblox::LongIndexHashMapType<Node *>::type node_lookup;
    Node *start_node = new Node(0, 0, 0, start_idx, nullptr);
    Node *cur_node;
    Node *nbr_node;

    // Calculate what the idx_in_parent value should be based on the velocity
    // this helps guide start of A* search to point in direction of motion
    if (start_vel != Point3f::Zero())
    {
        float min_d = std::numeric_limits<float>::max();
        int min_idx = 0;
        for (int i = 0; i < 26; i++)
        {
            Point3f vec = voxblox::Neighborhood<>::kOffsets.col(i).cast<float>();
            float d = (vec - start_vel).squaredNorm();
            if (d < min_d)
            {
                min_d = d;
                min_idx = i;
            }
        }

        start_node->idx_in_parent = min_idx;
    }

    // Insert node into our lookup map
    node_lookup.insert(std::make_pair(start_node->esdf_idx, start_node));

    int64_t s = rc_nanos_monotonic_time();
    int iteration_count = 0;

    open.push(start_node);

    while (!open.empty())
    {
        if (iteration_count >= MAX_ITERATIONS)
        {
            pthread_mutex_unlock(&map_->esdf_mutex);

            // Cleanup nodes
            for (auto it = node_lookup.begin(); it != node_lookup.end(); it++)
                delete it->second;

            fprintf(stderr, "Failed to reach goal in %d iterations after %fms\n", MAX_ITERATIONS, (double)(rc_nanos_monotonic_time() - s) / 1e6);
            return PlannerReturnValue::MaxIterationsReached;
        }

        iteration_count++;

        cur_node = open.top();
        open.pop();

        // "Insert" node into closed set (less overhead using a variable stored in node)
        cur_node->closed = true;

        // Check if we have reached end
        if (cur_node->esdf_idx == goal_idx)
        {
            break;
        }

        // Get the global indices of neighbors.
        voxblox::Neighborhood<voxblox::Connectivity::kTwentySix>::IndexMatrix neighbor_indices;
        voxblox::Neighborhood<voxblox::Connectivity::kTwentySix>::getFromGlobalIndex(cur_node->esdf_idx, &neighbor_indices);

        // Go through the neighbors and add them to the open set
        for (int idx = 0; idx < neighbor_indices.cols(); ++idx)
        {
            const voxblox::GlobalIndex &nbr_global_idx = neighbor_indices.col(idx);
            const voxblox::EsdfVoxel *esdf_voxel = layer->getVoxelPtrByGlobalIndex(nbr_global_idx);

            if (!treat_unknown_as_occupied)
            {
                // If no voxel then create a temporary new one
                if (esdf_voxel == nullptr)
                {
                    voxblox::EsdfVoxel tmp;
                    tmp.distance = esdf_default_distance;
                    esdf_voxel = &tmp;
                }
                else if ((esdf_voxel->observed || esdf_voxel->hallucinated) && std::fabs(esdf_voxel->distance) < robot_radius)
                {
                    continue;
                }
            }
            else
            {
                // Skip if no voxel, or voxel has not been seen or voxel has been seen and is in collision
                if (esdf_voxel == nullptr || !esdf_voxel->observed || (esdf_voxel->observed && esdf_voxel->distance < robot_radius))
                    continue;
            }

            // We are trying to find the node that optimizes the following:
            //  - Shortest distance from start
            //  - Avoids unneccessary turns
            //  - Avoids obstacles
            float dist_cost = voxblox::Neighborhood<voxblox::Connectivity::kTwentySix>::kDistances[idx];
            float change_dir_cost = cur_node->idx_in_parent == idx ? 0 : 2;

            // Use absolute value since esdf distance behind obstacles is negative (but we still want to be pushed away from it)
            // also need to check against epsilon due to floating point precision
            float obs_cost = std::fabs(esdf_voxel->distance) <= std::numeric_limits<float>::epsilon() ? esdf_default_distance : std::fabs(esdf_voxel->distance);
            obs_cost = 1 / obs_cost;

            float travel_cost = cur_node->travel_cost +
                                dist_cost +
                                change_dir_cost +
                                obs_cost * 3;

            // If nbr is in node_lookup fetch the pointer, otherwise create a new node
            if (node_lookup.count(nbr_global_idx) > 0)
            {
                nbr_node = node_lookup[nbr_global_idx];

                // Skip if node is in closed set
                if (nbr_node->closed)
                {
                    continue;
                }

                // Check if the path through this node is shorter
                if (travel_cost < nbr_node->travel_cost)
                {
                    nbr_node->travel_cost = travel_cost;
                    nbr_node->total_cost = travel_cost + nbr_node->heuristic_cost;
                    nbr_node->parent = cur_node;
                    nbr_node->idx_in_parent = idx;
                    nbr_node->steps = cur_node->steps + 1;

                    // Reorder priority queue since we have changed the value
                    std::make_heap(const_cast<Node **>(&open.top()),
                                   const_cast<Node **>(&open.top()) + open.size(),
                                   Compare());
                }
            }
            else
            {
                float heuristic_cost = 3 * computeHeuristic(nbr_global_idx, goal_idx);
                nbr_node = new Node(travel_cost + heuristic_cost, travel_cost, heuristic_cost, nbr_global_idx, cur_node);
                nbr_node->idx_in_parent = idx;

                // Insert node into our lookup map
                node_lookup.insert(std::make_pair(nbr_node->esdf_idx, nbr_node));

                // Add to queue
                open.push(nbr_node);
            }
        }
    }

    printf("A* took = %6.2fms for %d iterations\n", (double)(rc_nanos_monotonic_time() - s) / 1e6, iteration_count);

    std::vector<Node *> path;

    // Get the path out and reverse it
    while (cur_node != nullptr)
    {
        path.push_back(cur_node);
        cur_node = cur_node->parent;
    }
    std::reverse(path.begin(), path.end());

    pruneAStarPath(&path);

    Point3fVector target_points;

    // Convert path to correct format for smoother
    for (size_t i = 0; i < path.size(); i++)
    {
        Point3f pos = voxblox::getCenterPointFromGridIndex(path[i]->esdf_idx, voxel_size);
        target_points.push_back(pos);
    }

    // Don't want centre of voxels for our start and end points
    target_points.front() = start_pos;
    target_points.back() = goal_pos;

    visualizeAStarPath(target_points);

    // Cleanup nodes
    for (auto it = node_lookup.begin(); it != node_lookup.end(); it++)
        delete it->second;

    s = rc_nanos_monotonic_time();
    if (!runSmoother(target_points, trajectory))
    {
        fprintf(stderr, "Smoother failed in local planner. Skipping..\n");
        pthread_mutex_unlock(&map_->esdf_mutex);
        return PlannerReturnValue::SmootherFailed;
    }
    printf("Smoother took = %6.2fms\n", (double)(rc_nanos_monotonic_time() - s) / 1e6);
    pthread_mutex_unlock(&map_->esdf_mutex);

    return PlannerReturnValue::Success;
}

float LocalAStar::computeHeuristic(voxblox::GlobalIndex cur_idx, voxblox::GlobalIndex goal_idx)
{
    int64_t dx = abs(cur_idx(0) - goal_idx(0));
    int64_t dy = abs(cur_idx(1) - goal_idx(1));
    int64_t dz = abs(cur_idx(2) - goal_idx(2));

    int64_t sorted[3] = {dx, dy, dz};
    int n = sizeof(sorted) / sizeof(int64_t);

    std::sort(sorted, sorted + n);

    // Use octile distance
    return sqrt(3) * sorted[0] + sqrt(2) * (sorted[1] - sorted[0]) + (sorted[2] - sorted[1]);
}

bool LocalAStar::isBetween(Point3f a, Point3f b, Point3f c)
{
    Point3f ab = b - a;
    Point3f ac = c - a;

    float dot_ac = ab.dot(ac);
    float dot_ab = ab.dot(ab);

    if (dot_ac < 0 || dot_ac > dot_ab)
        return false;
    else
        return true;
}

void LocalAStar::pruneAStarPath(std::vector<Node *> *path)
{
    // Nothing to prune
    if (path->size() <= 2)
    {
        return;
    }

    std::vector<Node *> new_path;
    new_path.reserve(path->size());
    new_path.push_back(path->front());

    size_t p = 0;
    size_t p_next = 1;

    while (p != path->size() - 1)
    {
        Point3f p1 = voxblox::getCenterPointFromGridIndex(path->at(p)->esdf_idx, voxel_size);

        while (p_next + 1 < path->size() - 1)
        {
            Point3f p2 = voxblox::getCenterPointFromGridIndex(path->at(p_next + 1)->esdf_idx, voxel_size);
            if (isEdgeInCollision(map_->getEsdfMapPtr().get(), p1, p2))
            {
                break;
            }

            p_next++;
        }

        new_path.push_back(path->at(p_next));
        p = p_next;
        p_next++;
    }

    path->swap(new_path);
}

void LocalAStar::plannerThread()
{
    int64_t next_time = 0;
    map_->updateEsdf(true);

    if (!getInitialPlan())
    {
        fprintf(stderr, "Failed to find initial plan. Exiting..\n");
        running_ = false;
        return;
    }

    while (running_)
    {
        loop_sleep(LOCAL_PLANNER_RATE, &next_time);
        uint64_t s = rc_nanos_monotonic_time();

        // Get current pose to check if we have reached goal
        rc_tf_t tf_body_wrt_fixed = RC_TF_INITIALIZER;
        if (!map_->getRobotPose(tf_body_wrt_fixed, rc_nanos_monotonic_time()))
            continue;

        Point3f cur_pos(tf_body_wrt_fixed.d[0][3], tf_body_wrt_fixed.d[1][3], tf_body_wrt_fixed.d[2][3]);

        if ((cur_pos - waypoints_.back()).norm() <= REACHED_DISTANCE)
        {
            printf("Reached goal. Stopping local planner..");
            break;
        }

        map_->updateEsdf(true);

        Point3f start_pos = cur_pos;
        int split_id = -1;
        double split_time = -1.0;

        if (!initial_plan_)
        {
            // Find the position on the current trajectory to start the new plan from
            // along with the segment id and time along segment where the position occurs

            if (!computeSplitPointDetails(&start_pos, &split_id, &split_time))
            {
                fprintf(stderr, "Failed to compute splitting point. Skippinng\n");
                continue;
            }
        }

        // Get goal point
        Point3f goal_pos;

        // If we cannot find a goal position, retrigger global planning
        if (!computeGoalPosition(start_pos, &goal_pos))
        {
            fprintf(stderr, "Failed to find suitable goal along path. Rerunning global planer\n");
            running_ = false;
            replan_function_(cur_pos, waypoints_.back());
            return;
        }

        mav_trajectory_generation::Trajectory trajectory;
        PlannerReturnValue ret = runPlanner(start_pos, goal_pos, &trajectory);

        switch (ret)
        {
        case PlannerReturnValue::Success:
            // Do nothing
            break;
        case PlannerReturnValue::SmootherFailed:
            // Skip to next plan cycle
            continue;
        case PlannerReturnValue::MaxIterationsReached:
            // Replan globally
            running_ = false;
            fprintf(stderr, "Local planner reached max iterations without solution. Rerunning global planner\n");
            replan_function_(cur_pos, waypoints_.back());
            return;
        }

        if (!convertAndSendTrajectory(trajectory, split_id, split_time, initial_plan_))
        {
            fprintf(stderr, "Local Planner failed to send trajectory\n");
            continue;
        }

        initial_plan_ = false;
        printf("Local planner took %6.2f\n", (double)(rc_nanos_monotonic_time() - s) / 1e6);
    }

    running_ = false;
    return;
}

void LocalAStar::setPlan(const Point3fVector &waypoints)
{
    // Dont allow setting of waypoints if the local planner is already running
    if (running_)
    {
        fprintf(stderr, "ERROR: Attempted to set new waypoints whilst local planner was running\n");
        return;
    }

    // Copy the waypoints
    waypoints_ = Point3fVector(waypoints);
}

LocalAStar::~LocalAStar()
{
    stop();
    map_ = nullptr;
}
