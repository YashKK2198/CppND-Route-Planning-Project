#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    //closest of all neighborin nodes from start pt of a grid
    start_node = &m_Model.FindClosestNode(start_x,start_y);

    //closest of all neighborin nodes from end pt of a grid
    end_node = &m_Model.FindClosestNode(end_x,end_y);


    //RoutePlanner(start_node,end_node) = &m_Model.FindClosestNode(start_x,start_y_end_x,end_y);

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

  return node->distance(*end_node); //Heuristic cost

  //Here as mentioned I have cumputed the cost of the node that it will take
  //to reach the end of the grid from the present position as Hvalue

}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors
//         to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all
// - the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value.
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

  // - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all
  // - the neighbors.
  current_node->FindNeighbors();

  // - For each node in current_node.neighbors, set the parent, the h_value, the g_value.
  for(auto x : current_node->neighbors)
  {
    //parent is current_node
    x->parent = current_node;

    //So g_value of variable x is summation of g_value of current node
    //That is cost it took to reach current_node + The cost it will take to reach goal node from the current_node
    x->g_value = current_node->g_value + current_node->distance(*x);


    //Use CalculateHValue below to implement the h-Value calculation.
    x->h_value = CalculateHValue(x);//Heuristic component in [f=g+h] ----- Computed Earlier


  // - For each node in current_node.neighbors, add the neighbor
  // - to open_list and set the node's visited attribute to true.

      open_list.push_back(x);

      x->visited = true; //Node is visited and enterd in open_list. So it is marked as true.

  }

}



//  Below I have created an extra funtion called CompareNodes()
//  This function takes the g_value anf h_value of both the
//  nodes and returns whichever has the lowest value
//  which is then removed in the next function

bool CompareNodes(RouteModel::Node *firstnode,RouteModel::Node *secondnode)
{
  float res_of_firstnode,res_of_secondnode;

  res_of_firstnode = firstnode->h_value + firstnode->g_value;
  res_of_secondnode = secondnode->h_value + secondnode->g_value;

  return res_of_firstnode > res_of_secondnode; //will need to access the result using open_list.back()
}



// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {

  std::sort(open_list.begin(),open_list.end(),CompareNodes);

  RouteModel::Node *lowest_sum = open_list.back();

//  Removing the above node from the open_list vector

  open_list.pop_back();


return lowest_sum;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

     RouteModel::Node parent;

    // TODO: Implement your solution here.

    // - This method should take the current (final) node as an argument and iteratively follow the
    //   chain of parents of nodes until the starting node is found.
    // - For each node in the chain, add the distance from the node to its parent to the distance variable.

    while (current_node->parent)
    {
      /* code */
      path_found.push_back(*current_node);
      //parent = *(current_node->parent);
      distance += current_node->parent->distance(*current_node);
      current_node = current_node->parent;

    }
    path_found.push_back(*current_node);
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::reverse(path_found.begin(), path_found.end());
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    start_node->visited=true;
    open_list.push_back(start_node);
    while(open_list.size() != 0)
    {
      current_node = NextNode();
      if(current_node == end_node)
      {
        m_Model.path = ConstructFinalPath(current_node);
      }
      AddNeighbors(current_node);
    }

}
