#include "library_fcn.hpp"
#include "sensor_msgs/msg/laser_scan.h"
#include "geometry_msgs/msg/polygon_stamped.h"
#include "geometry_msgs/msg/polygon.h"
#include "geometry_msgs/msg/point32.h"
#include "geometry_msgs/msg/vector3.h"
#include "sensor_msgs/msg/point_cloud.h"
#include <cmath>
#include <limits>

// infinite for raytracing
#define INF 100000
#define eps 10e-9


//*********************************************************************
//********************* POLYTOPE FUNCTIONS ****************************
//*********************************************************************


// Return the min of 2 numbers
float min(float a, float b)
{
  return a<b ? a : b;
}

// Return the max of 2 numbers
float max(float a, float b)
{
  return a>b ? a : b;
}

// return dot product between a and b
float dotProd(geometry_msgs::msg::Vector3 a, geometry_msgs::msg::Vector3 b)
{
	return a.x*b.x + a.y*b.y;// + a.z*b.z;
}

// return magnitude of vector a
float magnitude(geometry_msgs::msg::Vector3 a)
{
	return sqrt(pow(a.x, 2) + pow(a.y, 2));// + pow(a.z, 2));
}

// return angle of vector b (in 2D!!)
float angle(geometry_msgs::msg::Vector3 a)
{
	return atan2(a.y, a.x);
}

// return normalized vector
geometry_msgs::msg::Vector3 normalize(geometry_msgs::msg::Vector3 a)
{
	geometry_msgs::msg::Vector3 b;
	float m = magnitude(a);
	b.x = a.x/m;
	b.y = a.y/m;
	b.z = a.z/m;
	return b;
}

// return vector a scaled by factor s
geometry_msgs::msg::Vector3 scale(geometry_msgs::msg::Vector3 a, float s)
{
	geometry_msgs::msg::Vector3 b;
	b.x = a.x*s;
	b.y = a.y*s;
	b.z = a.z*s;
	return b;
}

// Move point p along UNITARY vector v by distance d
geometry_msgs::msg::Point32 movePoint(geometry_msgs::msg::Point32 p, geometry_msgs::msg::Vector3 v, float d)
{
  p.x += v.x * d;
  p.y += v.y * d;
  p.z += v.z * d;

  return p;
}

// Return the distance between point a and b
float distSq(geometry_msgs::msg::Point32 a, geometry_msgs::msg::Point32 b)
{
  return (pow(a.x-b.x,2)+pow(a.y-b.y,2)+pow(a.z-b.z,2));
}
// Return the distance between point a and b
float distSq(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b)
{
  return (pow(a.x-b.x,2)+pow(a.y-b.y,2)+pow(a.z-b.z,2));
}
// Return the distance between vertices a and b
float distSq(gbeam2_interfaces::msg::Vertex a, gbeam2_interfaces::msg::Vertex b)
{
  return (pow(a.x-b.x,2)+pow(a.y-b.y,2)+pow(a.z-b.z,2));
}
// Given three colinear points p, q, r, the function checks if q lies between p and r
bool onSegment(geometry_msgs::msg::Point32 p, geometry_msgs::msg::Point32 q, geometry_msgs::msg::Point32 r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;
    return false;
}
// Find orientation of ordered triplet (p, q, r)
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(geometry_msgs::msg::Point32 p, geometry_msgs::msg::Point32 q, geometry_msgs::msg::Point32 r)
{
    float val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    return (val > 0) ? 1 : ((val<0) ? 2 : 0);
}
// Check intersection between segment 'p1q1' and 'p2q2'
bool doIntersect(geometry_msgs::msg::Point32 p1, geometry_msgs::msg::Point32 q1, geometry_msgs::msg::Point32 p2, geometry_msgs::msg::Point32 q2)
{
    // compute the orientation of the 4 triplets
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4) return true;
    // Special Cases
    // p1, q1, p2 colinear and p2 lies on p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    // p1, q1, q2 colinear and q2 lies on p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    // p2, q2, p1 colinear and p1 lies on p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    // p2, q2, q1 colinear and q1 lies on p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // no intersection
}
// Check if p lies inside the polygon poly
bool isInside(geometry_msgs::msg::Polygon poly, geometry_msgs::msg::Point32 p)
{
    int n = poly.points.size();
    // less than 3 vertices --> no polygon
    if (n < 3)  return false;

    // Create a point for line segment from p to infinite
    geometry_msgs::msg::Point32 extreme;
    extreme.x = INF;
    extreme.y = p.y;
    extreme.z = 0;

    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
        int next = (i+1)%n;
        // Check if the line segment from 'p' to 'extreme' intersects
        // with the line segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(poly.points[i], poly.points[next], p, extreme))
        {
            // If the point 'p' is colinear with line segment 'i-next',
            // then check if it lies on segment. If it lies, return true,
            // otherwise false
            if (orientation(poly.points[i], p, poly.points[next]) == 0)
              return onSegment(poly.points[i], p, poly.points[next]);

            count++;
        }
        i = next;
    } while (i != 0);

    // Return true if count is odd, false otherwise
    return count&1;  // Same as (count%2 == 1)
}
// check if p lies inside the convex polygon poly
bool isInsideConv(geometry_msgs::msg::Polygon poly, geometry_msgs::msg::Point32 p)
{
  int n = poly.points.size();
  // less than 3 vertices --> no polygon
  if (n < 3)
    return false;

  bool inCW = true, inCCW = true; //check for both CW and CCW orientation of vertices
  int i = 0;
  do
  {
      int next = (i+1)%n;
      if (orientation(poly.points[i], poly.points[next], p) == 1) //==1 -> consider also boundary
        inCW = false;
      else
        if (orientation(poly.points[i], poly.points[next], p) >= -1)
          inCCW = false;

      i = next;
  } while (i != 0);
  return inCW || inCCW;
}
// check convexity of poly, true if convex
bool isConv(geometry_msgs::msg::Polygon poly)
{
  int n = poly.points.size();
  // less than 3 vertices --> no polygon
  if (n < 3)  return false;

  int i = 0;
  do
  {
      int next = (i+1)%n, nnext = (i+2)%n;
      if (orientation(poly.points[i], poly.points[next], poly.points[nnext]) == 1) return false;
      i = next;
  } while (i != 0);
  return true;
}
// count the number of points from query that lay inside poly
int countInside(geometry_msgs::msg::Polygon poly, sensor_msgs::msg::PointCloud query)
{
  int count = 0;
  for(int i=0; i<query.points.size(); i++)
    if (isInside(poly, query.points[i]))
      count++;
  return count;
}
// count the number of points of query inside convex polygon poly
int countInsideConv(geometry_msgs::msg::Polygon poly, sensor_msgs::msg::PointCloud query)
{
  int count = 0;
  for(int i=0; i<query.points.size(); i++)
    if (isInsideConv(poly, query.points[i]))
      count++;
  return count;
}

// count points in query closer than d_thr to point p
int countClose(geometry_msgs::msg::Point32 p, sensor_msgs::msg::PointCloud query, float d_thr)
{
  int count = 0;
  float d_thr_sq = pow(d_thr,2);
  for (int i=0; i<query.points.size(); i++)
    if (distSq(p, query.points[i]) < d_thr_sq)
      count ++;
  return count;
}

// compute obstacle normal directed away from obstacle, with norm = 1
geometry_msgs::msg::Vector3 computeNormal(geometry_msgs::msg::Point32 p, sensor_msgs::msg::PointCloud query, float d_thr)
{
  sensor_msgs::msg::PointCloud closePoints;
  float d_thr_sq = pow(d_thr,2);
  float n = 0;
  float xc = 0, yc = 0;
  for (int i=0; i<query.points.size(); i++)
    if (distSq(p, query.points[i]) < d_thr_sq)
    {
      n ++;
      xc += query.points[i].x;
      yc += query.points[i].y;
      closePoints.points.push_back(query.points[i]);
    }
  xc = xc / n;
  yc = yc / n;

  float a=0, b=0, c=0, x=0, y=0;
  for (int i=0; i<n; i++)
  {
    x = closePoints.points[i].x - xc;
    y = closePoints.points[i].y - yc;
    a += x*y;
    b += pow(y,2)-pow(x,2);
    c -= x*y;
  }
  float t = ( -b + sqrt(pow(b,2)-4*a*c) ) / (2*a);

  geometry_msgs::msg::Vector3 v;
  v.z =0;
  v.x = sqrt(1/(1+pow(t,2)));
  v.y = -t*v.x;

  // check normal orientation wrt vector between first and last point (q)
  geometry_msgs::msg::Point32 p_a, p_b;
  p_a = closePoints.points[0];
  p_b = closePoints.points[n-1];
  geometry_msgs::msg::Vector3 q;
  q.x = p_b.x - p_a.x;
  q.y = p_b.y - p_a.y;
  float orientation = v.x*q.y - v.y*q.x;

  if(orientation < 0)
  {
    v.x = -v.x;
    v.y = -v.y;
  }

  return v; // check sign
}

// create vertices_reachable by offsetting the polygon formed by vertices_obstacles by a distance d
gbeam2_interfaces::msg::FreePolygon createReachablePolygon(gbeam2_interfaces::msg::FreePolygon in, float d)
{
  // prepare output polygon, and clear vertices_reachable
  gbeam2_interfaces::msg::FreePolygon poly = in;
  poly.vertices_reachable.erase(poly.vertices_reachable.begin(), poly.vertices_reachable.end());

  // v3 <--n2--> v2 <--n1--> v1
  int num_vert = in.vertices_obstacles.size();
  int j;          
  std::vector<float> nx, ny;
  nx.resize(num_vert);
  ny.resize(num_vert);

  float nL, bx, by, bL, l;

  for(int i=0; i<num_vert; i++) //compute polygon edges normals
  {
    // normal i is computed from vertex i to vertex i+1
    j = (i+1)%num_vert; //index off successive vertex
    nx[i] = poly.vertices_obstacles[i].y - poly.vertices_obstacles[j].y;
    ny[i] = poly.vertices_obstacles[j].x - poly.vertices_obstacles[i].x;
    // normalize normals
    nL = sqrt(pow(nx[i], 2)+pow(ny[i], 2));
    nx[i] /= nL;
    ny[i] /= nL;
  }

  for(int i=0; i<num_vert; i++) //move vertices
  {
    // vertex i is moved based on normal i and normal i-1
    j = (i-1)%num_vert; //index of previous normal
    j = (num_vert + ((i-1)%num_vert)) % num_vert;
    // compute bisector vector components
    bx = nx[i]+nx[j];
    by = ny[i]+ny[j];
    // normalize bisector
    bL = sqrt(pow(bx, 2)+pow(by, 2));
    bx /= bL;
    by /= bL;

    // compute vertex movement distance l
    l = d / sqrt(1 + (nx[i]*nx[j] + ny[i]*ny[j])) * sqrt(2);    // srqt(2) scale to get right offset
    // copy vertex and move it by l in direction b
    poly.vertices_reachable.push_back(poly.vertices_obstacles[i]);
    poly.vertices_reachable[i].x += l*bx;
    poly.vertices_reachable[i].y += l*by;
    poly.vertices_reachable[i].is_reachable = true;
    poly.vertices_reachable[i].is_obstacle = false;
  }

  return poly;
}

// move vertex v away from obstacle (along normal direction), only if node is obstacle
gbeam2_interfaces::msg::Vertex moveAway(gbeam2_interfaces::msg::Vertex v, float d)
{
  if(!v.is_obstacle)  // if not an obstacle do nothing
    return v;

  // normalize obstacle_normal
  // float norm = sqrt(pow(v.obstacle_normal.x, 2) + pow(v.obstacle_normal.y, 2));
  // v.obstacle_normal.x /= norm;
  // v.obstacle_normal.y /= norm;

  v.x -= v.obstacle_normal.x * d;
  v.y -= v.obstacle_normal.y * d;
  return v;
}






//*********************************************************************
//********************* GRAPH FUNCTIONS *******************************
//*********************************************************************


// convert gbeam_library/Vertex v into geometry_msgs/Point32 p
geometry_msgs::msg::Point32 vertex2point32(gbeam2_interfaces::msg::Vertex v)
{
  geometry_msgs::msg::Point32 p;
  p.x = v.x;
  p.y = v.y;
  p.z = v.z;
  return p;
}

// convert gbeam_library/Vertex v into geometry_msgs/Point32 p
geometry_msgs::msg::Point vertex2point(gbeam2_interfaces::msg::Vertex v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x;
  p.y = v.y;
  p.z = v.z;
  return p;
}

// convert FreePolygon to Polygon
geometry_msgs::msg::Polygon freePolyObstacles2poly(gbeam2_interfaces::msg::FreePolygon f)
{
  geometry_msgs::msg::Polygon p;
  for (int i = 0; i < f.vertices_obstacles.size(); i++)
    p.points.push_back(vertex2point32(f.vertices_obstacles[i]));
  return p;
}

// convert FreePolygon to Polygon
geometry_msgs::msg::Polygon freePolyReachable2poly(gbeam2_interfaces::msg::FreePolygon f)
{
  geometry_msgs::msg::Polygon p;
  for (int i = 0; i < f.vertices_reachable.size(); i++)
    p.points.push_back(vertex2point32(f.vertices_reachable[i]));
  return p;
}

// check if point p is inside the polyarea area
bool isInsideArea(gbeam2_interfaces::msg::PolyArea area, geometry_msgs::msg::PointStamped p)
{
  int n = area.polygons.size();
  geometry_msgs::msg::Point32 point;
  for (int i=0; i<n; i++)
    if (isInsideConv(area.polygons[i], point))
      return true;
  return false;
}

// check if vertex v is inside the polyarea area
bool isInsideArea(gbeam2_interfaces::msg::PolyArea area, gbeam2_interfaces::msg::Vertex v)
{
  geometry_msgs::msg::Point32 p = vertex2point32(v);
  int n = area.polygons.size();
  for (int i=0; i<n; i++)
    if (isInsideConv(area.polygons[i], p))
      return true;
  return false;
}

// compute distance between 2 vertices
float dist(gbeam2_interfaces::msg::Vertex v, gbeam2_interfaces::msg::Vertex u)
{
  return sqrt(pow(v.x-u.x, 2) + pow(v.y-u.y, 2));
}

float dist(gbeam2_interfaces::msg::Vertex v, geometry_msgs::msg::PointStamped p)
{
  return sqrt(pow(v.x-p.point.x, 2) + pow(v.y-p.point.y, 2));
}

// compute minimum distance between vertex v and graph nodes
float vert_graph_distance(gbeam2_interfaces::msg::Graph graph, gbeam2_interfaces::msg::Vertex v)
{
  float d_min = INF;
  for (int i=0; i<graph.nodes.size(); i++)
  {
    float d = dist(graph.nodes[i], v);
    if (d < d_min)
      d_min = d;
  }
  return d_min;
}

// compute minimum distance between vertex v and graph nodes
std::pair<float,int> vert_graph_distance_noobstacle(gbeam2_interfaces::msg::Graph graph, gbeam2_interfaces::msg::Vertex v)
{
  float d_min = INF;
  int id_min = -1;
  for (int i=0; i<graph.nodes.size(); i++)
  {
    if(!graph.nodes[i].is_obstacle)
    {
      float d = dist(graph.nodes[i], v);
      if (d < d_min){
        d_min = d;
        id_min = graph.nodes[i].id;
      }
    }
  }
  return std::make_pair(d_min,id_min);
}

// compute minimum distance between vertex v and graph nodes
float vert_graph_distance_obstacle(gbeam2_interfaces::msg::Graph graph, gbeam2_interfaces::msg::Vertex v)
{
  float d_min = INF;
  for (int i=0; i<graph.nodes.size(); i++)
  {
    if(graph.nodes[i].is_obstacle)
    {
      float d = dist(graph.nodes[i], v);
      if (d < d_min)
      d_min = d;
    }
  }
  return d_min;
}

// apply transform tf to vertex v, return resulting vertex
gbeam2_interfaces::msg::Vertex vert_transform(gbeam2_interfaces::msg::Vertex v, geometry_msgs::msg::TransformStamped tf)
{
  geometry_msgs::msg::PointStamped p_in;
    p_in.point.x = v.x; p_in.point.y = v.y; p_in.point.z = v.z;
  geometry_msgs::msg::PointStamped p_out;
  tf2::doTransform(p_in, p_out, tf);

  geometry_msgs::msg::Vector3Stamped vec_in;
    vec_in.vector = v.obstacle_normal;
  geometry_msgs::msg::Vector3Stamped vec_out;
  tf2::doTransform(vec_in, vec_out, tf);

  v.x = p_out.point.x; v.y = p_out.point.y; v.z = p_out.point.z;
  v.obstacle_normal = vec_out.vector;

  return v;
}

// apply tf to FreePolygon p, return result
gbeam2_interfaces::msg::FreePolygon poly_transform(gbeam2_interfaces::msg::FreePolygon p, geometry_msgs::msg::TransformStamped tf)
{
  gbeam2_interfaces::msg::FreePolygon f;
  for (int i=0; i<p.vertices_obstacles.size(); i++)
  {
    f.vertices_obstacles.push_back(vert_transform(p.vertices_obstacles[i], tf));
  }
  for (int i=0; i<p.vertices_reachable.size(); i++)
  {
    f.vertices_reachable.push_back(vert_transform(p.vertices_reachable[i], tf));
  }
  return f;
}

// NEW FUNCTION

gbeam2_interfaces::msg::Graph graph_transform(const gbeam2_interfaces::msg::Graph& graph, const geometry_msgs::msg::TransformStamped& tf)
{
    gbeam2_interfaces::msg::Graph transformed_graph;

    // Transform nodes
    for (const auto& node : graph.nodes)
    {
        transformed_graph.nodes.push_back(vert_transform(node, tf));
    }

    // Transform edges
    for (const auto& edge : graph.edges)
    {
        gbeam2_interfaces::msg::GraphEdge transformed_edge = edge;
        
        // Transform direction vector
        geometry_msgs::msg::Vector3Stamped dir_in, dir_out;
        dir_in.vector = edge.direction;
        tf2::doTransform(dir_in, dir_out, tf);
        transformed_edge.direction = dir_out.vector;

        // Recalculate length (optional, as it might change due to scaling)
        transformed_edge.length = std::sqrt(
            std::pow(transformed_edge.direction.x, 2) +
            std::pow(transformed_edge.direction.y, 2) +
            std::pow(transformed_edge.direction.z, 2)
        );

        transformed_graph.edges.push_back(transformed_edge);
    }

    return transformed_graph;
}

std::pair<gbeam2_interfaces::msg::Graph, gbeam2_interfaces::msg::FreePolygonStamped> 
graph_transform_and_get_fakepoly(const gbeam2_interfaces::msg::Graph& graph, const geometry_msgs::msg::TransformStamped& tf)
{
    gbeam2_interfaces::msg::Graph transformed_graph;
    gbeam2_interfaces::msg::FreePolygonStamped graph_polygon;

    // Transform nodes and populate the polygon
    for (const auto& node : graph.nodes)
    {
        auto transformed_node = vert_transform(node, tf);
        transformed_graph.nodes.push_back(transformed_node);

        // Add the transformed node to the appropriate vector in the polygon
        if (transformed_node.is_obstacle) {
            graph_polygon.polygon.vertices_obstacles.push_back(transformed_node);
        } else if (transformed_node.is_reachable) {
            graph_polygon.polygon.vertices_reachable.push_back(transformed_node);
        }
    }

    // Transform edges
    for (const auto& edge : graph.edges)
    {
        gbeam2_interfaces::msg::GraphEdge transformed_edge = edge;
        
        // Transform direction vector
        geometry_msgs::msg::Vector3Stamped dir_in, dir_out;
        dir_in.vector = edge.direction;
        tf2::doTransform(dir_in, dir_out, tf);
        transformed_edge.direction = dir_out.vector;

        // Recalculate length (optional, as it might change due to scaling)
        transformed_edge.length = std::sqrt(
            std::pow(transformed_edge.direction.x, 2) +
            std::pow(transformed_edge.direction.y, 2) +
            std::pow(transformed_edge.direction.z, 2)
        );

        transformed_graph.edges.push_back(transformed_edge);
    }

    return std::make_pair(transformed_graph, graph_polygon);
}

gbeam2_interfaces::msg::FreePolygonStamped
get_obstacles_and_reachable_nodes(const gbeam2_interfaces::msg::Graph::SharedPtr graph)
{
    gbeam2_interfaces::msg::FreePolygonStamped graph_polygon;

    // Transform nodes and populate the polygon
    for (const auto node : graph->nodes)
    {
        if (node.is_obstacle) {
            graph_polygon.polygon.vertices_obstacles.push_back(node);
        } else if (node.is_reachable) {
            graph_polygon.polygon.vertices_reachable.push_back(node);
        }
    }

    return graph_polygon;
}

// check if vertex v is inside polytope polygon
bool isInsideObstacles(gbeam2_interfaces::msg::FreePolygon poly, gbeam2_interfaces::msg::Vertex v)
{
  return isInsideConv(freePolyObstacles2poly(poly), vertex2point32(v));
}

// check if vertex v is inside polytope polygon
bool isInsideReachable(gbeam2_interfaces::msg::FreePolygon poly, gbeam2_interfaces::msg::Vertex v)
{
  return isInsideConv(freePolyReachable2poly(poly), vertex2point32(v));
}

// check if vertex v is inside boundaries l_xi<x<l_xs && l_yi<y<l_ys if not move it to the nearest point inside and mark it
gbeam2_interfaces::msg::Vertex applyBoundary(gbeam2_interfaces::msg::Vertex v, float l_xi, float l_xs, float l_yi, float l_ys)
{
  if(v.x < l_xi || v.x > l_xs || v.y < l_yi || v.y > l_ys)  // if the vex is outside exploration boundaries
  {
    v.gain = 0;
    v.is_completely_connected = false;
    v.is_obstacle = true;
    v.is_reachable = false;
    v.is_visited = true;
    if(v.x < l_xi)
    {
      v.obstacle_normal.x = 0, v.obstacle_normal.y = 1;
      v.x = l_xi;
    }
    if(v.x > l_xs)
    {
      v.obstacle_normal.x = 0, v.obstacle_normal.y = -1;
      v.x = l_xs;
    }
    if(v.y < l_yi)
    {
      v.obstacle_normal.x = 1, v.obstacle_normal.y = 0;
      v.y = l_yi;
    }
    if(v.y > l_ys)
    {
      v.obstacle_normal.x = -1, v.obstacle_normal.y = 0;
      v.y = l_ys;
    }
  }
  return v;
}

// check if vertex v is inside boundaries l_xi<x<l_xs && l_yi<y<l_ys if not move it to the nearest point inside and mark it
bool isInBoundary(gbeam2_interfaces::msg::Vertex v, float l_xi, float l_xs, float l_yi, float l_ys)
{
  return v.x < l_xs && v.x > l_xi && v.y < l_ys && v.y > l_yi;
}

// compute data of edge between vertices vert1 and vert2
gbeam2_interfaces::msg::GraphEdge computeEdge(gbeam2_interfaces::msg::Vertex vert1, gbeam2_interfaces::msg::Vertex vert2, float node_bound_dist)
{
  gbeam2_interfaces::msg::GraphEdge edge;
  edge.v1 = vert1.id; edge.v2 = vert2.id;
  edge.length = dist(vert1, vert2);
  edge.direction.x = vert2.x - vert1.x;
  edge.direction.y = vert2.y - vert1.y;
  edge.direction.z = 0;
  float norm = sqrt(pow(edge.direction.x, 2) + pow(edge.direction.y, 2));
  edge.direction.x /= norm;
  edge.direction.y /= norm;
  if (vert1.is_obstacle && vert2.is_obstacle && edge.length<node_bound_dist)
    edge.is_boundary = true;
  else
    edge.is_boundary = false;
  edge.is_walkable = false; //set non walkable as default
  
  vert1.neighbors.push_back(vert2.id);
  vert2.neighbors.push_back(vert1.id);

  return edge;
}

// NEW FUNCTIONS
gbeam2_interfaces::msg::GraphAdjacency matrix2GraphAdj(const std::vector<std::vector<float>>& matrix)
{
    gbeam2_interfaces::msg::GraphAdjacency adj;
    int N = matrix.size();
    adj.size = N;
    adj.data.resize(N * N, 0.0f);  // Resize and initialize with zeros

    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            adj.data[i * N + j] = matrix[i][j];
        }
    }

    return adj;
}

std::vector<std::vector<float>> GraphAdj2matrix(const gbeam2_interfaces::msg::GraphAdjacency& adj)
{
    int N = adj.size;
    std::vector<std::vector<float>> matrix(N, std::vector<float>(N, 0.0f));  // Initialize with zeros

    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            matrix[i][j] = adj.data[i * N + j];
        }
    }

    return matrix;
}

void addNode(gbeam2_interfaces::msg::Graph& graph, const gbeam2_interfaces::msg::Vertex& vert)
{
    // Add vertex to the graph
    graph.nodes.push_back(vert);

    // Update adjacency matrix
    int N = graph.adj_matrix.size;
    graph.adj_matrix.size = N + 1;
    std::vector<float> new_data((N + 1) * (N + 1), -1.0f);

    // Copy old data to new adjacency matrix
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            new_data[i * (N + 1) + j] = graph.adj_matrix.data[i * N + j];
        }
    }

    // Initialize new row and column
    for (int i = 0; i < N + 1; ++i) {
        new_data[i * (N + 1) + N] = -1.0f;  // New column
        new_data[N * (N + 1) + i] = -1.0f;  // New row
    }

    // Replace old data with new data
    graph.adj_matrix.data = new_data;
}

//*********************************************************************
//********************* EXPLORATION FUNCTIONS *******************************
//*********************************************************************

// find minimum element in array, return its index
int iMin(float arr[], int size)
{
  float min = INF;
  int id = 0;
  for(int i=0; i<size; i++)
  {
    if(arr[i]<min)
    {
      min = arr[i];
      id = i;
    }
  }
  return id;
}

// find maximum element in array, return its index
int iMax(float arr[], int size)
{
  float max = 0;
  int id = 0;
  for(int i=0; i<size; i++)
  {
    if(arr[i]>max)
    {
      max = arr[i];
      id = i;
    }
  }
  return id;
}

// return minimum element, from ones with con[i]==true
int iMinCon(float arr[], bool con[], int size)
{
  float min = INF;
  int id = 0;
  for(int i=0; i<size; i++)
  {
    if(arr[i]<min && con[i])
    {
      min = arr[i];
      id = i;
    }
  }
  return id;
}

// return maximum element, from ones with con[i]==true
int iMaxCon(float arr[], bool con[], int size)
{
  float max = 0;
  int id = 0;
  for(int i=0; i<size; i++)
  {
    if(arr[i]>max && con[i])
    {
      max = arr[i];
      id = i;
    }
  }
  return id;
}

//compute distances matrices of graph
void shortestDistances(gbeam2_interfaces::msg::Graph graph, float dist[], int start)
{
  int E = graph.edges.size();
  int N = graph.nodes.size();
  bool Q_set[N];
  int prev[N];

  for(int i=0; i<N; i++)
    dist[i] = INF, prev[i] = -1, Q_set[i] = graph.nodes[i].is_reachable ? true : false;
    // note: add to Q_set only nodes that are reachable

  dist[start] = 0;

  for (int count=0; count<N; count++)//all vertices
  {
    int u = iMinCon(dist, Q_set, N); // select the node with shortest distance from Q
    Q_set[u] = false; // remove selected node from Q

    if(dist[u] >= INF)  // if the shortest distance is INF, then the graph is not connected.
      break;

    for(int e=0; e<E; e++)
    {
      // if an edge is connected to u, then check if the node it is connected to
      // belongs to set Q. In that case check if the total distance passing through u
      // is shorter than the current best one, in that case update it.
      if(graph.edges[e].is_walkable)
      {
        if(graph.edges[e].v1 == u && Q_set[graph.edges[e].v2] && dist[graph.edges[e].v2] > (dist[u]+graph.edges[e].length))
        {
          dist[graph.edges[e].v2] = dist[u] + graph.edges[e].length;
          prev[graph.edges[e].v2] = u;
        }
        if(graph.edges[e].v2 == u && Q_set[graph.edges[e].v1] && dist[graph.edges[e].v1] > (dist[u]+graph.edges[e].length))
        {
          dist[graph.edges[e].v1] = dist[u] + graph.edges[e].length;
          prev[graph.edges[e].v1] = u;
        }
      }
    }
  }

  return;
}

// compute shortest path in graph from start to end
// using Dijkstra algorithm
std::vector<int> dijkstra(gbeam2_interfaces::msg::Graph graph, int s, int t)
{
  int N = graph.nodes.size();
  int E = graph.edges.size();

  // initialize variables
  // dist[i] will contain the minimum distance from s to i
  // prev[i] will contain the previous node in the shortest path
  float dist[N];
  int prev[N];
  bool Q_set[N];
  for (int i=0; i<N; i++)
    dist[i] = INF, prev[i] = -1, Q_set[i] = graph.nodes[i].is_reachable ? true : false;
  // note: add to Q_set only nodes that are reachable

  dist[s] = 0;

  for (int count=0; count<N; count++)//all vertices
  {
    int u = iMinCon(dist, Q_set, N); // select the node with shortest distance from Q
    Q_set[u] = false; // remove selected node from Q

    if(dist[u] >= INF || u == t)  // if the shortest distance is INF, then the graph is not connected.
    // if u == t, then the shortest path from s to t is already available.
    break;

    for(int e=0; e<E; e++)
    {
      // if an edge is connected to u, then check if the node it is connected to
      // belongs to set Q. In that case check if the total distance passing through u
      // is shorter than the current best one, in that case update it.
      if(graph.edges[e].v1 == u && Q_set[graph.edges[e].v2] && dist[graph.edges[e].v2] > (dist[u]+graph.edges[e].length))
      {
        dist[graph.edges[e].v2] = dist[u] + graph.edges[e].length;
        prev[graph.edges[e].v2] = u;
      }
      if(graph.edges[e].v2 == u && Q_set[graph.edges[e].v1] && dist[graph.edges[e].v1] > (dist[u]+graph.edges[e].length))
      {
        dist[graph.edges[e].v1] = dist[u] + graph.edges[e].length;
        prev[graph.edges[e].v1] = u;
      }
    }
  }

  std::vector<int> path;
  path.push_back(t);
  while(path[0] != s)
  path.insert(path.begin(), prev[path[0]]);

  return path;
}

// compute best path from s to t (if t<0 it is ignored)
std::vector<int> bestPath(gbeam2_interfaces::msg::Graph graph, int s, int t)
{
  int N = graph.nodes.size();
  int E = graph.edges.size();

  // initialize variables
  // dist[i] will contain the minimum distance from s to i
  // prev[i] will contain the previous node in the shortest path
  float dist[N];
  int prev[N];
  bool Q_set[N];
  for (int i=0; i<N; i++)
  dist[i] = INF, prev[i] = -1, Q_set[i] = graph.nodes[i].is_reachable ? true : false;
  // note: add to Q_set only nodes that are reachable

  dist[s] = 0;

  for (int count=0; count<N; count++)//all vertices
  {
    int u = iMinCon(dist, Q_set, N); // select the node with shortest distance from Q
    Q_set[u] = false; // remove selected node from Q

    if(dist[u] >= INF || u == t)  // if the shortest distance is INF, then the graph is not connected.
    // if u == t, then the shortest path from s to t is already available.
    break;

    for(int e=0; e<E; e++)
    {
      // if an edge is connected to u, then check if the node it is connected to
      // belongs to set Q. In that case check if the total distance passing through u
      // is shorter than the current best one, in that case update it.
      if(graph.edges[e].is_walkable)
      {
        if(graph.edges[e].v1 == u && Q_set[graph.edges[e].v2] && dist[graph.edges[e].v2] > (dist[u]+graph.edges[e].length))
        {
          dist[graph.edges[e].v2] = dist[u] + graph.edges[e].length;
          prev[graph.edges[e].v2] = u;
        }
        if(graph.edges[e].v2 == u && Q_set[graph.edges[e].v1] && dist[graph.edges[e].v1] > (dist[u]+graph.edges[e].length))
        {
          dist[graph.edges[e].v1] = dist[u] + graph.edges[e].length;
          prev[graph.edges[e].v1] = u;
        }
      }
    }
  }

  std::vector<int> path;
  path.push_back(t);
  while(path[0] != s)
  path.insert(path.begin(), prev[path[0]]);

  return path;
}


//*********************************************************************
//********************* PARTIAL_MERGER FUNCTIONS **********************
//*********************************************************************



bool isApproximatelyEqual(double a, double b){
        return std::abs(a - b) <= eps * std::max({1.0, std::abs(a), std::abs(b)});
    }

bool hasVertexChanged(const gbeam2_interfaces::msg::Vertex& v1, 
                          const gbeam2_interfaces::msg::Vertex& v2){
        // Check owner of the node
        if (v1.belong_to==v2.belong_to)
        {
          return true;
        } else {
          // Compare fundamental attributes
          if (v1.id != v2.id) return true;
          
          // Compare positions with epsilon
          if (!isApproximatelyEqual(v1.x, v2.x)) return true;
          if (!isApproximatelyEqual(v1.y, v2.y)) return true;
          if (!isApproximatelyEqual(v1.z, v2.z)) return true;
          
          // Compare other attributes
          if (!isApproximatelyEqual(v1.gain, v2.gain)) return true;
          if (v1.is_visited != v2.is_visited) return true;
          if (v1.is_reachable != v2.is_reachable) return true;
          
          
          return false; 
        }
      }

bool hasEdgeChanged(const gbeam2_interfaces::msg::GraphEdge& e1, 
                        const gbeam2_interfaces::msg::GraphEdge& e2) {
        // Compare fundamental attributes
        if (e1.id!=e2.id) return true;
        if (e1.v1 != e2.v1 || e1.v2 != e2.v2) return true;
        
        // Compare floating-point values with epsilon
        if (!isApproximatelyEqual(e1.length, e2.length)) return true;
        
        if (e1.is_walkable!=e2.is_walkable) return true;
        
        return false; 
    }