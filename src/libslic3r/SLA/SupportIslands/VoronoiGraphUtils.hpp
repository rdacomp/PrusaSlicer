#ifndef slic3r_SLA_SuppotstIslands_VoronoiGraphUtils_hpp_
#define slic3r_SLA_SuppotstIslands_VoronoiGraphUtils_hpp_

#include <optional>
#include <map>
#include <set>
#include <libslic3r/Geometry.hpp>
#include <libslic3r/Point.hpp>
#include <libslic3r/SVG.hpp>

#include "VoronoiGraph.hpp"
#include "Parabola.hpp"
#include "SampleConfig.hpp"
#include "SupportIslandPoint.hpp"

namespace Slic3r::sla {

/// <summary>
/// Class which contain collection of static function
/// for work with Voronoi Graph.
/// </summary>
class VoronoiGraphUtils
{
    using VD = Slic3r::Geometry::VoronoiDiagram;

public:
    VoronoiGraphUtils() = delete;

    /// <summary>
    /// Convert line edge segment to slicer line
    /// only for line edge
    /// only for finite line
    /// </summary>
    /// <param name="edge">line edge</param>
    /// <returns>line</returns>
    static Slic3r::Line to_line(const VD::edge_type &edge);

    /// <summary>
    /// Private function to help convert edge without vertex to line
    /// </summary>
    /// <param name="point1">VD Source point</param>
    /// <param name="point2">VD Source point</param>
    /// <param name="maximal_distance">Maximal distance from source point</param>
    /// <returns>Line segment between lines</returns>
    static Slic3r::Line to_line(Point point1,
                                Point point2,
                                double        maximal_distance);
    /// <summary>
    /// Convert edge to line
    /// only for line edge
    /// crop infinite edge by maximal distance from source point
    /// inspiration in VoronoiVisualUtils::clip_infinite_edge
    /// </summary>
    /// <param name="edge"></param>
    /// <param name="points">Source point for voronoi diagram</param>
    /// <param name="maximal_distance">Maximal distance from source point</param>
    /// <returns>Croped line</returns>
    static Slic3r::Line to_line(const VD::edge_type & edge,
                                const Points &points,
                                double                maximal_distance);
    /// <summary>
    /// close polygon defined by lines 
    /// close points will convert to their center
    /// Mainly for convert to polygon
    /// </summary>
    /// <param name="lines">Border of polygon, sorted lines CCW</param>
    /// <param name="center">Center point of polygon</param>
    /// <param name="maximal_distance">Radius around center point</param>
    /// <param name="minimal_distance">Merge points closer than minimal_distance</param>
    /// <param name="count_points">Count checking points, create help points for result polygon</param>
    /// <returns>CCW polygon with center inside of polygon</returns>
    static Slic3r::Polygon to_polygon(const Lines &lines,
                                       const Point &center,
                                       double       maximal_distance,
                                       double minimal_distance,
                                       size_t       count_points);
    /// <summary>
    /// Convert cell to polygon
    /// Source for VD must be only point to create VD with only line segments
    /// infinite cell are croped by maximal distance from source point
    /// </summary>
    /// <param name="cell">cell from VD created only by points</param>
    /// <param name="points">source points for VD</param>
    /// <param name="maximal_distance">maximal distance from source point - only for infinite edges(cells)</param>
    /// <returns>polygon created by cell</returns>
    static Slic3r::Polygon to_polygon(const VD::cell_type & cell,
                                      const Slic3r::Points &points,
                                      double maximal_distance);

    // return node from graph by vertex, when no exists create one
    static VoronoiGraph::Node *getNode(VoronoiGraph &         graph,
                                       const VD::vertex_type *vertex,
                                       const VD::edge_type *  edge,
                                       const Lines &          lines);

    /// <summary>
    /// Extract point from lines, belongs to cell
    /// ! Source for VD must be only lines
    /// Main purpose parabola focus point from lines belongs to cell
    /// inspiration in VoronoiVisualUtils::retrieve_point
    /// </summary>
    /// <param name="lines">Source of Voronoi diagram</param>
    /// <param name="cell">Cell inside of Voronoi diagram</param>
    /// <returns>Point from source lines.</returns>
    static Point retrieve_point(const Lines &lines, const VD::cell_type &cell);

    /// <summary>
    /// Extract point from lines
    /// ! Source for VD must be only points
    /// inspiration in VoronoiVisualUtils::retrieve_point
    /// </summary>
    /// <param name="points">Source of Voronoi diagram</param>
    /// <param name="cell">Cell inside of Voronoi diagram</param>
    /// <returns>Point from source points.</returns>
    static Point retrieve_point(const Points &points, const VD::cell_type &cell);

    static Slic3r::Point get_parabola_point(const VD::edge_type &parabola, const Slic3r::Lines &lines);
    static Slic3r::Line get_parabola_line(const VD::edge_type &parabola, const Lines &lines);
    static Parabola get_parabola(const VD::edge_type &edge, const Lines &lines);

    /// <summary>
    /// Calculate length
    /// </summary>
    /// <param name="edge">curved edge</param>
    /// <returns>edge length</returns>
    static double calculate_length_of_parabola(
        const VD::edge_type &                edge,
        const Lines &                        lines);

    /// <summary>
    /// Calculate length of edge line segment or curve - parabola.
    /// </summary>
    /// <param name="edge">Input edge to calcuate length</param>
    /// <param name="lines">Source for Voronoi diagram. It contains parabola parameters</param>
    /// <returns>The length of edge</returns>
    static double calculate_length(const VD::edge_type &edge,
                                   const Lines &        lines);

    /// <summary>
    /// Calculate maximal distance to outline and multiply by two(must be similar on both side)
    /// </summary>
    /// <param name="edge">Input edge.</param>
    /// <param name="lines">Source for Voronoi diagram. It contains parabola parameters</param>
    /// <returns>Maximal island width along edge</returns>
    static double calculate_max_width(const VD::edge_type &edge,
                                      const Lines &        lines);

    /// <summary>
    /// calculate distances to border of island and length on skeleton
    /// </summary>
    /// <param name="voronoi_diagram">Input anotated voronoi diagram
    /// (use function Slic3r::Voronoi::annotate_inside_outside)</param>
    /// <param name="lines">Source lines for voronoi diagram</param>
    /// <returns>Extended voronoi graph by distances and length</returns>
    static VoronoiGraph getSkeleton(const VD &vd, const Lines &lines);

    /// <summary>
    /// For generating support point in distance from node
    /// </summary>
    /// <param name="node">Node lay on outline with only one neighbor</param>
    /// <param name="padding">Distance from outline</param>
    /// <returns></returns>
    static Slic3r::Point get_offseted_point(const VoronoiGraph::Node &node,
                                            double padding);

    /// <summary>
    /// find neighbor and return distance between nodes
    /// </summary>
    /// <param name="from">source of neighborse</param>
    /// <param name="to">neighbor node</param>
    /// <returns>When neighbor return distance between neighbor Nodes
    /// otherwise no value</returns>
    static const VoronoiGraph::Node::Neighbor *get_neighbor(
        const VoronoiGraph::Node *from, const VoronoiGraph::Node *to);

    /// <summary>
    /// use function get_neighbor
    /// when not neighbor assert
    /// </summary>
    /// <param name="from">source Node</param>
    /// <param name="to">destination Node</param>
    /// <returns>distance between Nodes or Assert when not neighbor</returns>
    static double get_neighbor_distance(const VoronoiGraph::Node *from,
                                        const VoronoiGraph::Node *to);

    /// <summary>
    /// Create longest node path over circle together with side branches
    /// </summary>
    /// <param name="circle">Source circle, can't be connected with another
    /// circle</param> <param name="side_branches">Circle side branches from
    /// nodes of circle</param> <param name="start_path">Path before circle -
    /// defince input point to circle</param> <returns>Longest nodes path and
    /// its length</returns>
    static VoronoiGraph::Path find_longest_path_on_circle(
        const VoronoiGraph::Circle &                 circle,
        const VoronoiGraph::ExPath::SideBranchesMap &side_branches);

    /// <summary>
    /// Serach longest path from input_node throw Nodes in connected circles,
    /// when circle is alone call find_longest_path_on_circle.
    /// </summary>
    /// <param name="input_node">Node on circle</param>
    /// <param name="finished_circle_index">index of circle with input
    /// node</param> <param name="ex_path">Hold Circles, connection of circles
    /// and Side branches</param> <returns>Longest path from input
    /// node</returns>
    static VoronoiGraph::Path find_longest_path_on_circles(
        const VoronoiGraph::Node &  input_node,
        size_t                      finished_circle_index,
        const VoronoiGraph::ExPath &ex_path);

    /// <summary>
    /// Function for detection circle in passed path.
    /// </summary>
    /// <param name="path">Already passed path in Graph</param>
    /// <param name="neighbor">Actual neighbor possible created circle</param>
    /// <returns>Circle when exists</returns>
    static std::optional<VoronoiGraph::Circle> create_circle(
        const VoronoiGraph::Path &          path,
        const VoronoiGraph::Node::Neighbor &neighbor);

    /// <summary>
    /// Move source connected circles into destination
    /// </summary>
    /// <param name="dst">In/Out param</param>
    /// <param name="src">Input possible modified, do not use it after this
    /// function</param> <param name="dst_circle_count">Count of destination
    /// circles before merge Source circle are append afted destination, therfore
    /// all src indexes must be increased by destination circle count</param>
    static void merge_connected_circle(
        VoronoiGraph::ExPath::ConnectedCircles &dst,
        VoronoiGraph::ExPath::ConnectedCircles &src,
        size_t                                  dst_circle_count);

    /// <summary>
    /// move data from source to destination
    /// side_branches + circles + connected_circle
    /// </summary>
    /// <param name="dst">destination extended path - append data from
    /// source</param> <param name="src">source extended path - data will be
    /// moved to dst</param>
    static void append_neighbor_branch(VoronoiGraph::ExPath &dst,
                                       VoronoiGraph::ExPath &src);

    /// <summary>
    /// Heal starting from random point.
    /// Compare length of all starting path with side branches
    /// when side branch is longer than swap with start path
    /// </summary>
    /// <param name="path">IN/OUT path to be fixed after creating longest path
    /// from one point</param>
    static void reshape_longest_path(VoronoiGraph::ExPath &path);

    /// <summary>
    /// Extract the longest path from voronoi graph
    /// by own function call stack(IStackFunction).
    /// Restructuralize path by branch created from random point.
    /// </summary>
    /// <param name="start_node">Random point from outline.</param>
    static VoronoiGraph::ExPath create_longest_path(
        const VoronoiGraph::Node *start_node);

    /// <summary>
    /// Find source node of neighbor
    /// </summary>
    /// <param name="neighbor">neighbor</param>
    /// <returns>start node</returns>
    static const VoronoiGraph::Node *VoronoiGraphUtils::get_twin_node(
        const VoronoiGraph::Node::Neighbor *neighbor);

    /// <summary>
    /// Create point on edge defined by neighbor
    /// in distance defined by edge length ratio
    /// </summary>
    static Point get_edge_point(const VD::edge_type *edge, double ratio);

    static const VoronoiGraph::Node *getFirstContourNode(
        const VoronoiGraph &graph);

    /// <summary>
    /// Get max width from edge in voronoi graph
    /// </summary>
    /// <param name="longest_path">Input point to voronoi graph</param>
    /// <returns>Maximal widht in graph</returns>
    static double get_max_width(const VoronoiGraph::ExPath &longest_path);
    static double get_max_width(const VoronoiGraph::Nodes &path);
    static double get_max_width(const VoronoiGraph::Node *node);

public: // draw function for debug
    static void draw(SVG &svg, const VoronoiGraph &graph, coord_t width);
    static void draw(SVG &                      svg,
                     const VoronoiGraph::Nodes &path,
                     coord_t                    width,
                     const char *               color,
                     bool                       finish = false);
    static void draw(SVG &                       svg,
                     const VoronoiGraph::ExPath &path,
                     coord_t                     width);
};

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_VoronoiGraphUtils_hpp_
