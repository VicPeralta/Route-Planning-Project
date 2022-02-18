#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model) {
	// Convert inputs to percentage:
	start_x *= 0.01f;
	start_y *= 0.01f;
	end_x *= 0.01f;
	end_y *= 0.01f;

	// TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
	// Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
	start_node = &m_Model.FindClosestNode(start_x, start_y);
	end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
	current_node->FindNeighbors();
	current_node->visited = true;
	for (auto& neighbor : current_node->neighbors)
	{
		if (neighbor->visited == false)
		{
			neighbor->parent = current_node;
			neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
			neighbor->h_value = CalculateHValue(neighbor);
			neighbor->visited = true;
			open_list.emplace_back(neighbor);
		}
	}
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
	std::sort(open_list.begin(), open_list.end(),
		[](RouteModel::Node *lht, RouteModel::Node *rht)
	{
		// Ordena la lista de mayor a menor
		// 9 8 7 6 5
		float f1 = lht->g_value + lht->h_value;
		float f2 = rht->g_value + rht->h_value;
		return f1 > f2;
	}
	);
	// regresa el último que es el de más bajo valor
	RouteModel::Node *lowestNode = open_list.back();
	open_list.pop_back();
	return lowestNode;
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

	// TODO: Implement your solution here.
	while (current_node != start_node)
	{
		distance += current_node->distance(*current_node->parent);
		path_found.emplace_back(*current_node);
		current_node = current_node->parent;
	}
	path_found.emplace_back(*current_node);
	std::reverse(path_found.begin(), path_found.end());
	distance *= static_cast<float>(m_Model.MetricScale()); // Multiply the distance by the scale of the map to get meters.
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
	AddNeighbors(start_node);
	current_node = NextNode();
	do
	{
		if (current_node == end_node) {
			m_Model.path = ConstructFinalPath(current_node);
			return;
		}
		AddNeighbors(current_node);
		current_node = NextNode();
	} while (open_list.size() > 0);
}