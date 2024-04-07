#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                             mbot_lcm_msgs::pose2D_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    bool found_path = false;
    
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    Node* startNode = new Node(startCell.x, startCell.y);
    Node* goalNode = new Node(goalCell.x, goalCell.y);
    Node* endNode;
    bool ifcutOff=false;

    PriorityQueue OpenSet;
    std::vector<Node*> CloseSet;
    std::vector<Node*> SearchedSet;
    startNode->h_cost = h_cost(startNode, goalNode, distances);
    startNode->g_cost = 0;
    OpenSet.push(startNode);
    CloseSet.push_back(startNode);
    SearchedSet.push_back(startNode);
    while (!OpenSet.empty())
    {
        Node* current = OpenSet.pop();
        if(current->cell.x == goalNode->cell.x && current->cell.y == goalNode->cell.y)
        {
            found_path = true;
            break;
        }
        CloseSet.push_back(current);

        
        for (int j = -1; j <= 1; ++j) 
        {
            for (int i = -1; i <= 1; ++i) 
            {
                if (i == 0 && j == 0) continue; 
                int newX = current->cell.x + i;
                int newY = current->cell.y + j;
                if (distances.isCellInGrid(newX,newY) && (distances.operator()(newX, newY)>0.1)) 
                {
                    Node* neighbor = new Node(newX, newY);
                    if (is_in_list(neighbor, CloseSet)) {
                        continue;
                    }
                    if(is_in_list(neighbor,SearchedSet))
                    {
                        neighbor = get_from_list(neighbor,SearchedSet);
                    }

                    double tentativeG;
                    if (fabs(i)==1 && fabs(j)==1)
                    {
                        tentativeG = current->g_cost + distances.metersPerCell()*2.2 + 1/distances.operator()(newX, newY);
                    }
                    else
                    {
                        tentativeG = current->g_cost + distances.metersPerCell() + 1/distances.operator()(newX, newY);
                    }
                    bool isMember = false;

                    if(!is_in_list(neighbor,SearchedSet))
                    {
                        neighbor->g_cost = tentativeG;
                        neighbor->h_cost = h_cost(neighbor, goalNode, distances);
                        neighbor->parent = current;
                        OpenSet.push(neighbor);
                        SearchedSet.push_back(neighbor);
                    }
                    else if(neighbor->g_cost > tentativeG)
                    {
                        neighbor->g_cost = tentativeG;
                        neighbor->parent = current;
                        OpenSet.push(neighbor);
                    }

                
                }
            }
        }
        if(CloseSet.size()>15000)
        {
            ifcutOff = true;
            found_path = true;
            double MinH=9999999,MaxG=0;
            Node* t1;
            Node* t2;
            for(int i=0;i<CloseSet.size();i++)
            {
                if(CloseSet.at(i)->h_cost<MinH)
                {
                    MinH = CloseSet.at(i)->h_cost;
                    t1 = CloseSet.at(i);
                }
                if(CloseSet.at(i)->g_cost>MaxG)
                {
                    MaxG = CloseSet.at(i)->g_cost;
                    t2 = CloseSet.at(i);
                }
            }
            if(abs(t1->cell.x-startNode->cell.x)+abs(t1->cell.x-startNode->cell.x))
            endNode = t2;
            else endNode = t1;
        }
    }
    
    
    

    mbot_lcm_msgs::path2D_t path;
    path.utime = start.utime;
    if (found_path)
    {
        if(ifcutOff)
        {
            goalNode = endNode;
            printf("Too long have no path\n");
        }
        else
        {
            goalNode = get_from_list(goalNode,SearchedSet);
        }
        auto nodePath = extract_node_path(goalNode, startNode);
        nodePath = prune_node_path(nodePath,params.exploration);
        path.path = extract_pose_path(nodePath, distances);
        // Remove last pose, and add the goal pose
        path.path.pop_back();
        path.path.push_back(goal);
        path.path_length = path.path.size();
        // printf("[A*] find a path with length%d\n", path.path_length);
        return path;
    }

    else 
    {
        printf("[A*] Didn't find a path%ld\n",path.path.size());
        path.path_length = path.path.size();
        return path;
    }
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    double h_cost = 0.0;
    ////////////////// TODO: Implement your heuristic //////////////////////////
    int dx = fabs(goal->cell.x - from->cell.x);
    int dy = fabs(goal->cell.y - from->cell.y);
    h_cost = (dx + dy) * distances.metersPerCell();
    return h_cost;
}
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    double g_cost = 0.0;
    ////////////////// TODO: Implement your goal cost, use obstacle distances //////////////////////////
    
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    std::vector<Node*> children;
    ////////////////// TODO: Implement your expand node algorithm //////////////////////////
    
    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    std::vector<Node*> path;
    ////////////////// TODO: Implement your extract node function //////////////////////////
    // Traverse nodes and add parent nodes to the vector
    Node* tmp = goal_node;
    while (tmp != nullptr)
    {
        // printf("stx:%d;sty:%d-->",tmp->cell.x,tmp->cell.y);
        path.push_back(tmp);
        tmp = tmp->parent;
    }
    // std::vector<Node*> processedPath = prune_node_path(path);
    // Reverse path
    // std::reverse(processedPath.begin(), processedPath.end());
    std::reverse(path.begin(), path.end());
    // return processedPath;
    return path;
}
// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    std::vector<mbot_lcm_msgs::pose2D_t> path;
    ////////////////// TODO: Implement your extract_pose_path function //////////////////////////
    // This should turn the node path into a vector of poses (with heading) in the global frame
    // You should prune the path to get a waypoint path suitable for sending to motion controller
    
    for(int i=0;i<nodes.size();i++)
    {
        double x = nodes.at(i)->cell.x * distances.metersPerCell() + distances.originInGlobalFrame().x;
        double y = nodes.at(i)->cell.y * distances.metersPerCell() + distances.originInGlobalFrame().y;
        mbot_lcm_msgs::pose2D_t tmp;
        tmp.x = x;
        tmp.y = y;
        tmp.theta = 0;
        path.push_back(tmp);
    }
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (node->cell.x == item->cell.x && node->cell.y == item->cell.y) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (node->cell.x == n->cell.x && node->cell.y == n->cell.y) return n;
    }
    return NULL;

}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath, double param)
{
    std::vector<Node*> new_node_path;
    int step;
    if(param !=0)step =4;
    else step = 3;
    if(nodePath.size()<=step*2)
        return nodePath;
    for(int i=0;i<nodePath.size();i+=step)
    {
        new_node_path.push_back(nodePath.at(i));
    }
    ////////////////// TODO: Optionally implement a prune_node_path function //////////////////////////
    // This should remove points in the path along the same line
    
    
    return new_node_path;

}
