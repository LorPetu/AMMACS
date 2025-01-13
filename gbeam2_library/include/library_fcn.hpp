#ifndef LIBRARY_HPP
#define LIBRARY_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"


#include <stdio.h>
#include <math.h>
#include <vector>



#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


#include "gbeam2_interfaces/msg/poly_area.hpp"
#include "gbeam2_interfaces/msg/vertex.hpp"
#include "gbeam2_interfaces/msg/free_polygon.hpp"
#include "gbeam2_interfaces/msg/graph.hpp"
#include <gbeam2_interfaces/msg/graph_adjacency.hpp>
#include "gbeam2_interfaces/msg/free_polygon_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


//*********************************************************************
//********************* POLYTOPE FUNCTIONS ****************************
//*********************************************************************

// Return the min of 2 numbers
float min(float a, float b);

// Return the max of 2 numbers
float max(float a, float b);
// return dot product between a and b
float dotProd(geometry_msgs::msg::Vector3 a, geometry_msgs::msg::Vector3 b);

// return magnitude of vector a
float magnitude(geometry_msgs::msg::Vector3 a);

// return angle of vector b (in 2D!!);
float angle(geometry_msgs::msg::Vector3 a);

// return normalized vector
geometry_msgs::msg::Vector3 normalize(geometry_msgs::msg::Vector3 a);

// return vector a scaled by factor s
geometry_msgs::msg::Vector3 scale(geometry_msgs::msg::Vector3 a, float s);

// Move point p along UNITARY vector v by distance d
geometry_msgs::msg::Point32 movePoint(geometry_msgs::msg::Point32 p, geometry_msgs::msg::Vector3 v, float d);

// Return the distance between point a and b
float distSq(geometry_msgs::msg::Point32 a, geometry_msgs::msg::Point32 b);

float distSq(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b);

// Return the distance between vertices a and b
float distSq(gbeam2_interfaces::msg::Vertex a, gbeam2_interfaces::msg::Vertex b);

// Given three colinear points p, q, r, the function checks if q lies between p and r
bool onSegment(geometry_msgs::msg::Point32 p, geometry_msgs::msg::Point32 q, geometry_msgs::msg::Point32 r);

// Find orientation of ordered triplet (p, q, r)
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(geometry_msgs::msg::Point32 p, geometry_msgs::msg::Point32 q, geometry_msgs::msg::Point32 r);

// Check intersection between segment 'p1q1' and 'p2q2'
bool doIntersect(geometry_msgs::msg::Point32 p1, geometry_msgs::msg::Point32 q1, geometry_msgs::msg::Point32 p2, geometry_msgs::msg::Point32 q2);

// Check if p lies inside the polygon poly
bool isInside(geometry_msgs::msg::Polygon poly, geometry_msgs::msg::Point32 p);

// check if p lies inside the convex polygon poly
bool isInsideConv(geometry_msgs::msg::Polygon poly, geometry_msgs::msg::Point32 p);

// check convexity of poly, true if convex
bool isConv(geometry_msgs::msg::Polygon poly);

// count the number of points from query that lay inside poly
int countInside(geometry_msgs::msg::Polygon poly, sensor_msgs::msg::PointCloud query);

// count the number of points of query inside convex polygon poly
int countInsideConv(geometry_msgs::msg::Polygon poly, sensor_msgs::msg::PointCloud query);

// count points in query closer than d_thr to point p
int countClose(geometry_msgs::msg::Point32 p, sensor_msgs::msg::PointCloud query, float d_thr);

// compute angle of the normal to points in query close to p
geometry_msgs::msg::Vector3 computeNormal(geometry_msgs::msg::Point32 p, sensor_msgs::msg::PointCloud query, float d_thr);

// create vertices_reachable by offsetting the polygon formed by vertices_obstacles by a distance d
gbeam2_interfaces::msg::FreePolygon createReachablePolygon(gbeam2_interfaces::msg::FreePolygon in, float d);

// move vertex v away from obstacle (along normal direction), only if node is obstacle
gbeam2_interfaces::msg::Vertex moveAway(gbeam2_interfaces::msg::Vertex v, float d);





//*********************************************************************
//********************* GRAPH FUNCTIONS *******************************
//*********************************************************************

// convert gbeam_library/Vertex v into geometry_msgs/Point32 p
geometry_msgs::msg::Point32 vertex2point32(gbeam2_interfaces::msg::Vertex v);

// convert gbeam_library/Vertex v into geometry_msgs/Point32 p
geometry_msgs::msg::Point vertex2point(gbeam2_interfaces::msg::Vertex v);

// convert FreePolygon to Polygon
geometry_msgs::msg::Polygon freePolyObstacles2poly(gbeam2_interfaces::msg::FreePolygon f);

// convert FreePolygon to Polygon
geometry_msgs::msg::Polygon freePolyReachable2poly(gbeam2_interfaces::msg::FreePolygon f);

// check if point p is inside the polyarea area
bool isInsideArea(gbeam2_interfaces::msg::PolyArea area, geometry_msgs::msg::PointStamped p);

// check if vertex v is inside the polyarea area
bool isInsideArea(gbeam2_interfaces::msg::PolyArea area, gbeam2_interfaces::msg::Vertex v);

// compute distance between 2 vertices
float dist(gbeam2_interfaces::msg::Vertex v, gbeam2_interfaces::msg::Vertex u);

// compute distance between a vertex and a point
float dist(gbeam2_interfaces::msg::Vertex v, geometry_msgs::msg::PointStamped p);

// compute minimum distance between vertex v and graph nodes
float vert_graph_distance(gbeam2_interfaces::msg::Graph graph, gbeam2_interfaces::msg::Vertex v);

// compute minimum distance between vertex v and obstacle graph nodes
std::pair<float,int> vert_graph_distance_noobstacle(gbeam2_interfaces::msg::Graph graph, gbeam2_interfaces::msg::Vertex v);

// compute minimum distance between vertex v and reachable graph nodes
float vert_graph_distance_obstacle(gbeam2_interfaces::msg::Graph graph, gbeam2_interfaces::msg::Vertex v);

// apply transform tf to vertex v, return resulting vertex
gbeam2_interfaces::msg::Vertex vert_transform(gbeam2_interfaces::msg::Vertex v, geometry_msgs::msg::TransformStamped tf);

// apply tf to FreePolygon p, return result
gbeam2_interfaces::msg::FreePolygon poly_transform(gbeam2_interfaces::msg::FreePolygon p, geometry_msgs::msg::TransformStamped tf);

// check if vertex v is inside polytope polygon
bool isInsideObstacles(gbeam2_interfaces::msg::FreePolygon poly, gbeam2_interfaces::msg::Vertex v);

// check if vertex v is inside polytope polygon
bool isInsideReachable(gbeam2_interfaces::msg::FreePolygon poly, gbeam2_interfaces::msg::Vertex v);

// check if vertex v is inside boundaries l_xi<x<l_xs && l_yi<y<l_ys if not move it to the nearest point inside and mark it
gbeam2_interfaces::msg::Vertex applyBoundary(gbeam2_interfaces::msg::Vertex v, float l_xi, float l_xs, float l_yi, float l_ys);

// check if vertex v is inside boundaries l_xi<x<l_xs && l_yi<y<l_ys if not move it to the nearest point inside and mark it
bool isInBoundary(gbeam2_interfaces::msg::Vertex v, float l_xi, float l_xs, float l_yi, float l_ys);

// compute data of edge between vertices vert1 and vert2
gbeam2_interfaces::msg::GraphEdge computeEdge(gbeam2_interfaces::msg::Vertex vert1, gbeam2_interfaces::msg::Vertex vert2, float node_bound_dist);

// NEW FUNCTIONS
gbeam2_interfaces::msg::GraphAdjacency matrix2GraphAdj(const std::vector<std::vector<float>>& matrix);

std::vector<std::vector<float>> GraphAdj2matrix(const gbeam2_interfaces::msg::GraphAdjacency& adj);

void addNode(gbeam2_interfaces::msg::Graph& graph, const gbeam2_interfaces::msg::Vertex& vert);


//*********************************************************************
//********************* EXPLORATION FUNCTIONS *************************
//*********************************************************************

// find minimum element in array, return its index
int iMin(float arr[], int size);

// find maximum element in array, return its index
int iMax(float arr[], int size);

// return minimum element, from ones with con[i]==true
int iMinCon(float arr[], bool con[], int size);

// return maximum element, from ones with con[i]==true
int iMaxCon(float arr[], bool con[], int size);

//compute distances matrices of graph
void shortestDistances(gbeam2_interfaces::msg::Graph graph, float dist[], int start);

// compute shortest path in graph from start to end
// using Dijkstra algorithm
std::vector<int> dijkstra(gbeam2_interfaces::msg::Graph graph, int s, int t);

// using Dijkstra but exploit matrix adjacency
std::vector<int> dijkstraWithAdj(gbeam2_interfaces::msg::Graph graph, int s, int t);

//std::pair<std::vector<int>,double> dijkstraWithAdjandPath(gbeam2_interfaces::msg::GraphCluster graph, int s, int t);

// compute best path from s to t (if t<0 it is ignored)
std::vector<int> bestPath(gbeam2_interfaces::msg::Graph graph, int s, int t);

bool isApproximatelyEqual(double a, double b);

bool hasVertexChanged(const gbeam2_interfaces::msg::Vertex& v1, 
                          const gbeam2_interfaces::msg::Vertex& v2) ;

bool hasEdgeChanged(const gbeam2_interfaces::msg::GraphEdge& e1, 
                        const gbeam2_interfaces::msg::GraphEdge& e2) ;

gbeam2_interfaces::msg::Graph graph_transform(const gbeam2_interfaces::msg::Graph& graph, const geometry_msgs::msg::TransformStamped& tf);

std::pair<gbeam2_interfaces::msg::Graph, gbeam2_interfaces::msg::FreePolygonStamped> 
graph_transform_and_get_fakepoly(const gbeam2_interfaces::msg::Graph& graph, const geometry_msgs::msg::TransformStamped& tf);

gbeam2_interfaces::msg::FreePolygonStamped
get_obstacles_and_reachable_nodes(const gbeam2_interfaces::msg::Graph::SharedPtr graph);

#endif  // LIBRARY_HPP