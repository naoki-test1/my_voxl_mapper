#include "rrt_connect.h"
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <random>
#include <float.h>
#include <unistd.h>
#include "path_vis.h"

#define INFORMED_RRT_RUNTIME 2e8

static uint64_t rc_nanos_monotonic_time()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000000) + ts.tv_nsec;
}

RRTConnect::RRTConnect(voxblox::TsdfServer *map, int vis_channel)
    : map_(map),
      root_(nullptr),
      node_counter_(-2), // Set to -2 so that -2 is start and -1 is goal
      vis_channel_(vis_channel),
      last_create_idx_(0)
{
    // Set the random seed
    srand(time(nullptr));

    lower_bound_ = Point3f::Zero();
    upper_bound_ = Point3f::Zero();
}

bool RRTConnect::computeMapBounds()
{
    if (map_ == nullptr)
    {
        printf("ERROR: esdf map is nullptr, cannot compute map bounds.\n");
        return false;
    }

    voxblox::utils::computeObservedMapBoundsFromLayer<voxblox::EsdfVoxel>(*map_->getEsdfMapPtr()->getEsdfLayerPtr(), &lower_bound_, &upper_bound_);

    if (debug)
    {
        printf("Map Bounds:\n");
        printf("Lower-> x:%6.2f, y:%6.2f, z:%6.2f\n", (double)lower_bound_.x(), (double)lower_bound_.y(), (double)lower_bound_.z());
        printf("Upper-> x:%6.2f, y:%6.2f, z:%6.2f\n", (double)upper_bound_.x(), (double)upper_bound_.y(), (double)upper_bound_.z());
    }

    return true;
}

RRTConnect::Node *RRTConnect::createNewNode(const Point3f &position, Node *parent)
{
    Node *new_node = new Node{.position = position, .parent = parent, .children = std::vector<Node *>(), .id = node_counter_};
    node_counter_++;

    if (parent != nullptr)
        parent->children.push_back(new_node);

    return new_node;
}

RRTConnect::Node *RRTConnect::createRandomNode()
{
    float x = ((upper_bound_.x() - lower_bound_.x()) * (float)rand() / (float)RAND_MAX) + lower_bound_.x();
    float y = ((upper_bound_.y() - lower_bound_.y()) * (float)rand() / (float)RAND_MAX) + lower_bound_.y();
    float z = ((upper_bound_.z() - lower_bound_.z()) * (float)rand() / (float)RAND_MAX) + lower_bound_.z();

    Point3f rand_pt(roundf(x * 100) / 100, roundf(y * 100) / 100, roundf(z * 100) / 100);

    return createNewNode(rand_pt, nullptr);
}

RRTConnect::Node *RRTConnect::createRandomNodeInEllipsoid(const Eigen::Matrix3f &C, const Eigen::Matrix3f &L, const Point3f &center)
{
    // Samples in range [-1:1]
    Point3f sampled_point = Point3f::Random();

    while (sampled_point.norm() > 1 || sampled_point.norm() < -1)
        sampled_point = Point3f::Random();

    sampled_point = C * L * sampled_point + center;

    return createNewNode(sampled_point, nullptr);
}

std::pair<RRTConnect::Node *, float> RRTConnect::findNearest(const Point3f &point)
{
    Node *closest = nullptr;
    float distance = FLT_MAX;

    // 3000 was chosen because it was the point at which the number of attempts stopped increasing
    // TODO: A smarter way would be to make this adaptive i.e. start at some lowish number and then
    // check when the time to search becomes greater than the time to create the octree
    if (points_.size() - last_create_idx_ > 3000)
    {
        octree_.initialize(points_);

        last_create_idx_ = points_.size() - 1;

        int32_t idx = octree_.findNeighbor<unibn::L2Distance<Point3f>>(point);

        closest = nodes_[idx];
        distance = (point - nodes_[idx]->position).norm();
    }
    else
    {
        int32_t idx = octree_.findNeighbor<unibn::L2Distance<Point3f>>(point);

        if (idx >= 0)
        {
            closest = nodes_[idx];
            distance = (point - nodes_[idx]->position).squaredNorm();
        }

        for (size_t i = last_create_idx_; i < points_.size(); i++)
        {
            float d = (points_[i] - point).squaredNorm();

            if (d < distance)
            {
                distance = d;
                closest = nodes_[i];
            }
        }

        distance = std::sqrt(distance);
    }

    return std::make_pair(closest, distance);
}

void RRTConnect::findNearestInRadius(const Point3f &point, float radius, std::vector<Node *> &nearest_nodes, std::vector<float> &distances)
{
    std::vector<uint32_t> results;
    std::vector<float> distances_sq;
    const float radius_sq = pow(radius, 2);

    // 3000 was chosen because it was the point at which the number of attempts stopped increasing
    // TODO: A smarter way would be to make this adaptive i.e. start at some lowish number and then
    // check when the time to search becomes greater than the time to create the octree
    if (points_.size() - last_create_idx_ > 3000)
    {
        octree_.initialize(points_);

        last_create_idx_ = points_.size() - 1;

        octree_.radiusNeighbors<unibn::L2Distance<Point3f>>(point, radius, results, distances_sq);

        for (size_t i = 0; i < results.size(); i++)
        {
            nearest_nodes.push_back(nodes_[results[i]]);
            distances.push_back(std::sqrt(distances_sq[i]));
        }
    }
    else
    {
        octree_.radiusNeighbors<unibn::L2Distance<Point3f>>(point, radius, results, distances_sq);

        for (size_t i = 0; i < results.size(); i++)
        {
            nearest_nodes.push_back(nodes_[results[i]]);
            distances.push_back(std::sqrt(distances_sq[i]));
        }

        for (size_t i = last_create_idx_; i < points_.size(); i++)
        {
            float d = (points_[i] - point).squaredNorm();

            if (d <= radius_sq)
            {
                nearest_nodes.push_back(nodes_[i]);
                distances.push_back(std::sqrt(d));
            }
        }
    }
}

void RRTConnect::add(Node *q_nearest, Node *q_new)
{
    q_new->parent = q_nearest;
    q_nearest->children.push_back(q_new);

    nodes_.push_back(q_new);

    points_.push_back(q_new->position);
}

void RRTConnect::deleteNodes(Node *&root)
{
    for (size_t i = 0; i < root->children.size(); i++)
    {
        deleteNodes(root->children[i]);
    }
    delete root;
    root = nullptr;
}

void RRTConnect::cleanupTree()
{
    if (root_)
        deleteNodes(root_);

    nodes_.clear();
    points_.clear();
    octree_.clear();
    node_counter_ = -2;
}

void RRTConnect::cleanupPruning()
{
    for (Node *node : pruning_nodes_)
    {
        delete node;
    }

    pruning_nodes_.clear();
}

void RRTConnect::pruneRRTPath(std::vector<Node *> &rrt_path)
{
    int prune_count_l1 = 0;
    int prune_count_l2 = 0;

    // Level 1 pruning
    int index_a;
    int index_b;

    float shift_a;
    float shift_b;

    Point3f candidate_a;
    Point3f candidate_b;

    // Generate between 0.2 and 0.8 otherwise we will have new nodes too close to the start nodes
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.2, 0.8);

    if (debug)
        fprintf(stderr, "%ld waypoints before pruning\n", rrt_path.size());

    for (int i = 0; i < rrt_prune_iterations; i++)
    {
        // Pick two nodes at random (ensure index b is always greater than index a)
        index_a = rand() % (rrt_path.size() - 2);
        index_b = rand() % (rrt_path.size() - 2 - index_a) + index_a + 1;

        // Generate a random shift along each nodes next edge
        shift_a = distribution(generator);
        shift_b = distribution(generator);

        // Calculate new points along the edge using the shift
        candidate_a = (1 - shift_a) * rrt_path[index_a]->position + shift_a * rrt_path[index_a + 1]->position;
        candidate_b = (1 - shift_b) * rrt_path[index_b]->position + shift_b * rrt_path[index_b + 1]->position;

        // The path between the two new positions is a shortcut. If its collision free,
        // add it into the path and remove the nodes that it skips
        if (!isEdgeInCollision(map_->getEsdfMapPtr().get(), candidate_a, candidate_b))
        {
            // Remove all intermediate nodes in rrt_path between A and B
            while (index_b > index_a)
            {
                rrt_path.erase(rrt_path.begin() + index_b);
                index_b--;
            }

            // Temp nodes only for inserting into the path
            Node *new_a = createNewNode(candidate_a, nullptr);
            Node *new_b = createNewNode(candidate_b, nullptr);

            // Save the nodes so we can clean them up after
            pruning_nodes_.push_back(new_a);
            pruning_nodes_.push_back(new_b);

            rrt_path.insert(rrt_path.begin() + index_a + 1, new_b);
            rrt_path.insert(rrt_path.begin() + index_a + 1, new_a);

            prune_count_l1++;
        }
    }

    // Level 2 pruning
    size_t p = 0;
    size_t p_next = 1;
    std::vector<Node *> new_path;
    new_path.reserve(rrt_path.size());
    new_path.push_back(rrt_path[0]);

    while (p != rrt_path.size() - 1)
    {
        Node *node_p = rrt_path[p];

        while (p_next + 1 < rrt_path.size() - 1)
        {
            Node *node_p_next = rrt_path[p_next + 1];
            if (isEdgeInCollision(map_->getEsdfMapPtr().get(), node_p->position, node_p_next->position))
            {
                break;
            }

            p_next++;
        }

        new_path.push_back(rrt_path[p_next]);
        p = p_next;
        p_next++;
    }

    prune_count_l2 = rrt_path.size() - new_path.size();
    rrt_path.swap(new_path);

    if (debug)
    {
        fprintf(stderr, "Level 1: Pruned %d times\n", prune_count_l1);
        fprintf(stderr, "Level 2: Pruned %d times\n", prune_count_l2);
        fprintf(stderr, "%ld waypoints after pruning\n", rrt_path.size());
    }
}

void RRTConnect::convertPathToOutput(const std::vector<Node *> &rrt_path, Point3fVector &waypoints)
{
    waypoints.reserve(rrt_path.size());

    for (const Node *node : rrt_path)
    {
        waypoints.push_back(node->position);
    }
}

void RRTConnect::visualizePath(const Point3fVector &waypoints)
{
    std::vector<path_vis_t> rrt_path;
    for (const Point3f &p : waypoints)
    {
        path_vis_t pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        pt.r = 0;
        pt.g = 0;
        pt.b = 0;
        rrt_path.push_back(pt);
    }

    generateAndSendPath(vis_channel_, rrt_path, PATH_VIS_LINE, "RRT Global Waypoints");
}

void RRTConnect::visualizeTree()
{
    // Note: The tree cannot be deleted for this to work
    // (i.e. cleanupTree() should not be called before this line runs)
    if (rrt_send_tree && root_ != nullptr)
    {
        std::vector<path_vis_t> rrt_tree;
        std::vector<Node *> stack;
        stack.push_back(root_);

        while (!stack.empty())
        {
            Node *root = stack.back();
            stack.pop_back();

            path_vis_t root_pt;
            root_pt.x = root->position.x();
            root_pt.y = root->position.y();
            root_pt.z = root->position.z();
            root_pt.r = 255;
            root_pt.g = 255;
            root_pt.b = 0;

            for (size_t i = 0; i < root->children.size(); i++)
            {
                stack.push_back(root->children[i]);
                Node *child = root->children[i];

                path_vis_t pt;
                pt.x = child->position.x();
                pt.y = child->position.y();
                pt.z = child->position.z();
                pt.r = 255;
                pt.g = 255;
                pt.b = 0;
                rrt_tree.push_back(root_pt);
                rrt_tree.push_back(pt);
            }
        }

        generateAndSendPath(vis_channel_, rrt_tree, PATH_VIS_TREE, "RRT Tree");
    }
}

bool RRTConnect::runRRT(const Point3f &start_pos, const Point3f &end_pos, Point3fVector &waypoints)
{
    computeMapBounds();

    // Setup start and end nodes
    Node *q_start = createNewNode(start_pos, nullptr);
    Node *q_goal = createNewNode(end_pos, nullptr);

    root_ = q_start;
    std::vector<Node *> rrt_path;

    // Insert start into search structure
    nodes_.push_back(q_start);
    points_.push_back(start_pos);

    uint64_t start_time = rc_nanos_monotonic_time();
    int attempts = 0;
    Node *q_rand = nullptr;
    Node *q_connect = nullptr;
    Node *q_near = nullptr;
    float dist = 0;
    bool collision_found;

    while (rrt_max_runtime_nanoseconds == -1 || start_time + rrt_max_runtime_nanoseconds > rc_nanos_monotonic_time())
    {
        // Get random node or use goal
        if (attempts % 50 == 0)
            q_rand = q_goal;
        else
            q_rand = createRandomNode();

        // Try again if we couldnt get a random node
        if (!q_rand)
            continue;

        collision_found = false;

        // Find nearest node in tree
        std::tie(q_near, dist) = findNearest(q_rand->position);

        Point3f dir_vec = ((q_rand->position - q_near->position) / dist);

        // Continually step towards q_rand by rrt_min_distance and add a node if its collision free
        // Optimization: Use squared distances to avoid having to do the square root which is more expensive
        while ((q_rand->position - q_near->position).squaredNorm() > rrt_min_distance * rrt_min_distance)
        {
            q_connect = createNewNode(q_near->position + rrt_min_distance * dir_vec, nullptr);

            if (!isEdgeInCollision(map_->getEsdfMapPtr().get(), q_near->position, q_connect->position, true))
            {
                add(q_near, q_connect);
                q_near = q_connect;
            }
            else
            {
                // The current node is in collision so it is not added to the tree and needs to be deleted
                delete q_connect;
                q_connect = nullptr;
                collision_found = true;
                break;
            }
        }

        // Add q_rand only if the while loop succesfully finished and its collision free
        if (!collision_found && !isEdgeInCollision(map_->getEsdfMapPtr().get(), q_near->position, q_rand->position))
        {
            add(q_near, q_rand);

            // If we ran goal biasing then the end point would be the goal, so exit
            if (q_rand == q_goal)
                break;
        }
        else if (q_rand != q_goal)
        {
            // Delete q_rand if we couldnt add to graph and it wasnt the goal node
            delete q_rand;
            q_rand = nullptr;
        }

        // Check if we can reach goal
        std::tie(q_near, dist) = findNearest(q_goal->position);

        if (dist <= rrt_goal_threshold)
        {
            add(q_near, q_goal);
            break;
        }

        attempts++;
    }

    if (q_goal->parent != nullptr)
    {
        printf("RRT solution found in %6.2fms and took %d attempts with %ld in tree\n", (rc_nanos_monotonic_time() - start_time) / 1000000.0, attempts, points_.size());
    }
    else
    {
        printf("RRT solution not found. Planning exceeded %6.2fms and took %d attempts\n", rrt_max_runtime_nanoseconds / 1000000.0, attempts);
        return false;
    }

    // Get the path from q_goal to q_start
    Node *cur = q_goal;
    while (cur != nullptr)
    {
        rrt_path.push_back(cur);
        cur = cur->parent;
    }

    // Reverse path since we traveresed tree from leaf to root but we want root to leaf
    std::reverse(rrt_path.begin(), rrt_path.end());

    pruneRRTPath(rrt_path);

    convertPathToOutput(rrt_path, waypoints);

    // Cleanup extra nodes created due to pruning
    cleanupPruning();

    return true;
}

bool RRTConnect::runInformedRRT(const Point3f &start_pos, const Point3f &end_pos, Point3fVector &waypoints)
{
    // Based on https://arxiv.org/pdf/1404.2334.pdf
    fprintf(stderr, "Starting Informed RRT*\n");

    // Compute ellipsoid bounds
    float c_best = 0;
    Point3f prev_wp = waypoints[0];
    Point3f cur_wp = waypoints[1];

    for (size_t i = 1; i < waypoints.size(); i++)
    {
        cur_wp = waypoints[i];
        c_best += (cur_wp - prev_wp).norm();
        prev_wp = cur_wp;
    }
    float direct_dist = (start_pos - end_pos).norm();
    Point3f center = (start_pos + end_pos) / 2;

    // Setup necessary matrices for transforming the unit ball
    Eigen::Matrix3f L = Eigen::Matrix3f::Zero();
    L.diagonal() << c_best / 2, sqrt(pow(c_best, 2) - pow(direct_dist, 2)) / 2, sqrt(pow(c_best, 2) - pow(direct_dist, 2)) / 2;

    Point3f unit(1, 0, 0);
    Eigen::Matrix3f M = ((end_pos - start_pos) / (end_pos - start_pos).norm()) * unit.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix3f tmp = Eigen::Matrix3f::Zero();
    tmp.diagonal() << 1, svd.matrixU().determinant(), svd.matrixV().determinant();
    Eigen::Matrix3f C = svd.matrixU() * tmp * svd.matrixV().transpose();

    // Setup start and end nodes
    Node *x_start = createNewNode(start_pos, nullptr);
    Node *x_goal = createNewNode(end_pos, nullptr);

    x_start->cost = 0;
    x_goal->cost = 0;

    root_ = x_start;

    // Insert start into search structure
    nodes_.push_back(x_start);
    points_.push_back(start_pos);

    uint64_t start_time = rc_nanos_monotonic_time();
    int attempts = 0;
    Node *x_new = nullptr;
    Node *x_min = nullptr;
    float dist = 0;

    // TODO: CHANGE TERMINATION CRITERIA
    while (start_time + INFORMED_RRT_RUNTIME > rc_nanos_monotonic_time())
    {
        // Get random node or use goal
        if (attempts % 50 == 0)
            x_new = createNewNode(end_pos, nullptr);
        else
            x_new = createRandomNodeInEllipsoid(C, L, center);

        // Try again if we couldnt get a random node
        if (!x_new)
            continue;

        // Find nearest node in tree
        std::tie(x_min, dist) = findNearest(x_new->position);

        if (!x_min)
            std::cerr << L << std::endl;

        // Alter x_new position to only be rrt_min_distance away from the nearest point but in same dir
        Point3f dir_vec = ((x_new->position - x_min->position) / dist);
        x_new->position = x_min->position + dir_vec * rrt_min_distance;
        x_new->cost = x_min->cost + rrt_min_distance;

        if (!isEdgeInCollision(map_->getEsdfMapPtr().get(), x_min->position, x_new->position))
        {
            float c_min = x_new->cost;

            // Find all nodes within rewire radius
            std::vector<Node *> nearest_nodes;
            std::vector<float> distances;
            findNearestInRadius(x_new->position, 3 * rrt_min_distance, nearest_nodes, distances);

            // Find minumum cost path that adds x_new
            for (size_t i = 0; i < nearest_nodes.size(); i++)
            {
                Node *x_near = nearest_nodes[i];
                float c_new = x_near->cost + distances[i];

                if (c_new < c_min && !isEdgeInCollision(map_->getEsdfMapPtr().get(), x_near->position, x_new->position))
                {
                    x_min = x_near;
                    c_min = c_new;
                }
            }
            x_new->cost = c_min;
            add(x_min, x_new);

            // Rewire nodes that become shorter when passing through x_new instead of its current parent
            for (size_t i = 0; i < nearest_nodes.size(); i++)
            {
                Node *x_near = nearest_nodes[i];
                float c_near = x_near->cost;
                float c_new = x_new->cost + distances[i];

                if (c_new < c_near && !isEdgeInCollision(map_->getEsdfMapPtr().get(), x_new->position, x_near->position))
                {
                    Node *x_parent = x_near->parent;

                    // Erase-remove idiom to remove x_near from x_parents children
                    x_parent->children.erase(std::remove(x_parent->children.begin(), x_parent->children.end(), x_near), x_parent->children.end());
                    x_near->parent = x_new;
                    x_new->children.push_back(x_near);
                    x_near->cost = c_new;
                }
            }

            float dist_to_goal = (x_new->position - x_goal->position).norm();
            if (dist_to_goal < 2 * rrt_goal_threshold)
            {
                if (x_new->cost + dist_to_goal < c_best)
                {
                    x_goal->parent = x_new;
                    c_best = x_new->cost + dist_to_goal;
                    L.diagonal() << c_best / 2, sqrt(pow(c_best, 2) - pow(direct_dist, 2)) / 2, sqrt(pow(c_best, 2) - pow(direct_dist, 2)) / 2;
                }
            }
        }

        attempts++;
    }

    if (x_goal->parent != nullptr)
    {
        printf("Improved RRT solution found in %6.2fms and took %d attempts with %ld in tree\n", (rc_nanos_monotonic_time() - start_time) / 1000000.0, attempts, points_.size());
    }
    else
    {
        printf("Informed RRT solution not found. Planning exceeded %6.2fms and took %d attempts\n", INFORMED_RRT_RUNTIME / 1000000.0, attempts);
        return false;
    }

    // Get the path from q_goal to q_start
    std::vector<Node *> rrt_path;
    Node *cur = x_goal;
    while (cur != nullptr)
    {
        rrt_path.push_back(cur);
        cur = cur->parent;
    }

    // Reverse path since we traveresed tree from leaf to root but we want root to leaf
    std::reverse(rrt_path.begin(), rrt_path.end());

    waypoints.clear();
    pruneRRTPath(rrt_path);
    convertPathToOutput(rrt_path, waypoints);

    // Cleanup extra nodes created due to pruning
    cleanupPruning();

    return true;
}

bool RRTConnect::isPlanningFeasible(const Point3f &start_pos, const Point3f &end_pos)
{
    if (isInCollision(map_->getEsdfMapPtr().get(), start_pos))
    {
        printf("ERROR: Start point is in collision\n");
        return false;
    }
    else if (isInCollision(map_->getEsdfMapPtr().get(), end_pos))
    {
        printf("ERROR: End point is in collision\n");
        return false;
    }

    return true;
}

bool RRTConnect::checkImmediatePath(const Point3f &start_pos, const Point3f &end_pos)
{
    if (!isEdgeInCollision(map_->getEsdfMapPtr().get(), start_pos, end_pos))
    {
        printf("Immediate collision free path found!\n");
        return true;
    }

    return false;
}

bool RRTConnect::createPlan(const Point3f &start_pos, const Point3f &end_pos, Point3fVector &waypoints)
{
    bool success = false;
    printf("Starting RRTConnect planner\n");

    // Check preconditions for planning
    if (isPlanningFeasible(start_pos, end_pos))
    {
        if (checkImmediatePath(start_pos, end_pos))
        {
            waypoints.push_back(start_pos);
            waypoints.push_back(end_pos);

            success = true;
        }
        else
        {
            int64_t start_time = rc_nanos_monotonic_time();
            success = runRRT(start_pos, end_pos, waypoints);
            int64_t duration = rc_nanos_monotonic_time() - start_time;

            visualizeTree();
            cleanupTree();

            if (success && duration <= 5e8)
                runInformedRRT(start_pos, end_pos, waypoints);
        }

        visualizePath(waypoints);
    }

    cleanupTree();

    return success;
}

void RRTConnect::tearDown()
{
    cleanupTree();
    cleanupPruning();
}

RRTConnect::~RRTConnect()
{
    tearDown();
    map_ = nullptr;
}