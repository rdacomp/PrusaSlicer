#include "SampleIslandUtils.hpp"

#include <cmath>
#include <libslic3r/VoronoiOffset.hpp>
#include "IStackFunction.hpp"
#include "EvaluateNeighbor.hpp"
#include "ParabolaUtils.hpp"
#include "VoronoiGraphUtils.hpp"
#include "VectorUtils.hpp"

#include <magic_enum/magic_enum.hpp>
#include <libslic3r/VoronoiVisualUtils.hpp>

#include <libslic3r/ClipperUtils.hpp> // allign

// comment definition of NDEBUG to enable assert()
// #define NDEBUG
#include <cassert>

using namespace Slic3r::sla;

std::unique_ptr<SupportIslandPoint> SampleIslandUtils::create_point(
    const VoronoiGraph::Node::Neighbor *neighbor,
    double                              ratio,
    SupportIslandPoint::Type            type)
{
    VoronoiGraph::Position position(neighbor, ratio);
    Slic3r::Point p = VoronoiGraphUtils::create_edge_point(position);
    return std::make_unique<SupportCenterIslandPoint>(p, position, type);
}

std::unique_ptr<SupportIslandPoint> SampleIslandUtils::create_point_on_path(
    const VoronoiGraph::Nodes &path,
    double                     distance,
    SupportIslandPoint::Type   type)
{
    const VoronoiGraph::Node *prev_node       = nullptr;
    double                    actual_distance = 0.;
    for (const VoronoiGraph::Node *node : path) {
        if (prev_node == nullptr) { // first call
            prev_node = node;
            continue;
        }
        const VoronoiGraph::Node::Neighbor *neighbor =
            VoronoiGraphUtils::get_neighbor(prev_node, node);
        actual_distance += neighbor->edge_length;
        if (actual_distance >= distance) {
            // over half point is on
            double previous_distance = actual_distance - distance;
            double over_ratio = previous_distance / neighbor->edge_length;
            double ratio      = 1. - over_ratio;
            return create_point(neighbor, ratio, type);
        }
        prev_node = node;
    }
    // distance must be inside path
    // this means bad input params
    assert(false);
    return nullptr; // unreachable
}

SupportIslandPointPtr SampleIslandUtils::create_middle_path_point(
    const VoronoiGraph::Path &path, SupportIslandPoint::Type  type)
{
    return create_point_on_path(path.nodes, path.length / 2, type);
}

SupportIslandPoints SampleIslandUtils::create_side_points(
    const VoronoiGraph::Nodes &path, double side_distance)
{
    VoronoiGraph::Nodes reverse_path = path; // copy
    std::reverse(reverse_path.begin(), reverse_path.end());
    SupportIslandPoints result;
    result.reserve(2);
    result.push_back(create_point_on_path(path, side_distance, SupportIslandPoint::Type::two_points));
    result.push_back(create_point_on_path(reverse_path, side_distance, SupportIslandPoint::Type::two_points));
    return std::move(result);
}

SupportIslandPoints SampleIslandUtils::sample_side_branch(
    const VoronoiGraph::Node *     first_node,
    const VoronoiGraph::Path       side_path,
    double                         start_offset,
    const CenterLineConfiguration &cfg)
{
    assert(cfg.max_sample_distance > start_offset);
    double distance = cfg.max_sample_distance - start_offset;
    double length   = side_path.length - cfg.side_distance - distance;
    if (length < 0.) {
        VoronoiGraph::Nodes reverse_path = side_path.nodes;
        std::reverse(reverse_path.begin(), reverse_path.end());
        reverse_path.push_back(first_node);
        SupportIslandPoints result;
        result.push_back(
            create_point_on_path(reverse_path, cfg.side_distance,
                                 SupportIslandPoint::Type::center_line_end));
        return std::move(result);
    }
    // count of segment between points on main path
    size_t segment_count = static_cast<size_t>(
        std::ceil(length / cfg.max_sample_distance));
    double                     sample_distance = length / segment_count;
    SupportIslandPoints result;
    result.reserve(segment_count + 1);
    const VoronoiGraph::Node *prev_node = first_node;
    for (const VoronoiGraph::Node *node : side_path.nodes) {
        const VoronoiGraph::Node::Neighbor *neighbor =
            VoronoiGraphUtils::get_neighbor(prev_node, node);
        auto side_item = cfg.branches_map.find(node);
        if (side_item != cfg.branches_map.end()) {
            double start_offset = (distance < sample_distance / 2.) ?
                                      distance :
                                      (sample_distance - distance);

            if (side_item->second.top().length > cfg.min_length) {
                auto side_samples = sample_side_branches(side_item,
                                                         start_offset, cfg);
                result.insert(result.end(), std::move_iterator(side_samples.begin()),
                              std::move_iterator(side_samples.end()));
            }
        }
        while (distance < neighbor->edge_length) {
            double edge_ratio = distance / neighbor->edge_length;
            result.push_back(
                create_point(neighbor, edge_ratio, SupportIslandPoint::Type::center_line)
            );
            distance += sample_distance;
        }
        distance -= neighbor->edge_length;
        prev_node = node;
    }
    assert(fabs(distance - (sample_distance - cfg.side_distance)) < 1e-5);
    result.back()->type = SupportIslandPoint::Type::center_line_end;
    return std::move(result);
}

SupportIslandPoints SampleIslandUtils::sample_side_branches(
    const VoronoiGraph::ExPath::SideBranchesMap::const_iterator
        &                          side_branches_iterator,
    double                         start_offset,
    const CenterLineConfiguration &cfg)
{
    const VoronoiGraph::ExPath::SideBranches &side_branches =
        side_branches_iterator->second;
    const VoronoiGraph::Node *first_node = side_branches_iterator->first;
    if (side_branches.size() == 1)
        return sample_side_branch(first_node, side_branches.top(),
                                  start_offset, cfg);

    SupportIslandPoints         result;
    VoronoiGraph::ExPath::SideBranches side_branches_cpy = side_branches;
    while (side_branches_cpy.top().length > cfg.min_length) {
        auto samples = sample_side_branch(first_node, side_branches_cpy.top(),
                                          start_offset, cfg);
        result.insert(result.end(), 
            std::move_iterator(samples.begin()), 
            std::move_iterator(samples.end()));
        side_branches_cpy.pop();
    }
    return std::move(result);
}

std::vector<std::set<const VoronoiGraph::Node *>> create_circles_sets(
    const std::vector<VoronoiGraph::Circle> &     circles,
    const VoronoiGraph::ExPath::ConnectedCircles &connected_circle)
{
    std::vector<std::set<const VoronoiGraph::Node *>> result;
    std::vector<bool> done_circle(circles.size(), false);
    for (size_t circle_index = 0; circle_index < circles.size();
         ++circle_index) {
        if (done_circle[circle_index]) continue;
        done_circle[circle_index] = true;
        std::set<const VoronoiGraph::Node *> circle_nodes;
        const VoronoiGraph::Circle &         circle = circles[circle_index];
        for (const VoronoiGraph::Node *node : circle.nodes)
            circle_nodes.insert(node);

        circle_nodes.insert(circle.nodes.begin(), circle.nodes.end());
        auto cc = connected_circle.find(circle_index);
        if (cc != connected_circle.end()) {
            for (const size_t &cc_index : cc->second) {
                done_circle[cc_index]              = true;
                const VoronoiGraph::Circle &circle = circles[cc_index];
                circle_nodes.insert(circle.nodes.begin(), circle.nodes.end());
            }
        }
        result.push_back(circle_nodes);
    }
    return result;
}

Slic3r::Points SampleIslandUtils::to_points(const SupportIslandPoints &support_points)
{ 
    std::function<Point(const std::unique_ptr<SupportIslandPoint> &)> transform_func = &SupportIslandPoint::point;
    return VectorUtils::transform(support_points, transform_func);
}

std::vector<Slic3r::Vec2f> SampleIslandUtils::to_points_f(const SupportIslandPoints &support_points)
{
    std::function<Vec2f(const std::unique_ptr<SupportIslandPoint> &)> transform_func =
        [](const std::unique_ptr<SupportIslandPoint> &p) {
            return p->point.cast<float>();
        };
    return VectorUtils::transform(support_points, transform_func);
}

void SampleIslandUtils::align_samples(SupportIslandPoints &samples,
                                      const ExPolygon &    island,
                                      const SampleConfig & config)
{
    assert(samples.size() > 2);
    size_t count_iteration = config.count_iteration; // copy
    coord_t max_move        = 0;
    while (--count_iteration > 1) {
        max_move = align_once(samples, island, config);        
        if (max_move < config.minimal_move) break;
    }
    //std::cout << "Align use " << config.count_iteration - count_iteration
    //          << " iteration and finish with precision " << max_move << "nano meters" << std::endl;
}

bool is_points_in_distance(const Slic3r::Point & p,
                           const Slic3r::Points &points,
                           double                max_distance)
{
    for (const auto &p2 : points) {
        double d = (p - p2).cast<double>().norm();
        if (d > max_distance) return false;
    }
    return true;
}

//#define VISUALIZE_SAMPLE_ISLAND_UTILS_ALIGN_ONCE

coord_t SampleIslandUtils::align_once(SupportIslandPoints &samples,
                                      const ExPolygon &    island,
                                      const SampleConfig & config)
{
    assert(samples.size() > 2);
    using VD = Slic3r::Geometry::VoronoiDiagram;
    VD             vd;
    Slic3r::Points points = SampleIslandUtils::to_points(samples);

#ifdef VISUALIZE_SAMPLE_ISLAND_UTILS_ALIGN_ONCE
    static int  counter = 0;
    BoundingBox bbox(island);
    SVG svg(("align_"+std::to_string(counter++)+".svg").c_str(), bbox);
    svg.draw(island, "lightblue");
#endif // VISUALIZE_SAMPLE_ISLAND_UTILS_ALIGN_ONCE

    // create voronoi diagram with points
    construct_voronoi(points.begin(), points.end(), &vd);
    coord_t max_move = 0;
    for (const VD::cell_type &cell : vd.cells()) {
        SupportIslandPointPtr &sample = samples[cell.source_index()];
        if (!sample->can_move()) continue;
        Polygon polygon = VoronoiGraphUtils::to_polygon(cell, points, config.max_distance);
        Polygons intersections = Slic3r::intersection(island, ExPolygon(polygon));
        const Polygon *island_cell   = nullptr;
        for (const Polygon &intersection : intersections) {
            if (intersection.contains(sample->point)) {
                island_cell = &intersection;
                break;
            }
        }
        assert(island_cell != nullptr);
        Point center = island_cell->centroid();
        assert(is_points_in_distance(center, island_cell->points, config.max_distance));
#ifdef VISUALIZE_SAMPLE_ISLAND_UTILS_ALIGN_ONCE
        svg.draw(polygon, "lightgray");
        svg.draw(*island_cell, "gray");
        svg.draw(sample.point, "black", config.head_radius);
        svg.draw(Line(sample.point, center), "blue", config.head_radius / 5);
#endif // VISUALIZE_SAMPLE_ISLAND_UTILS_ALIGN_ONCE 
        coord_t act_move = sample->move(center);
        if (max_move < act_move) max_move = act_move;
    }
    return max_move;
}

SupportIslandPoints SampleIslandUtils::sample_center_line(
    const VoronoiGraph::ExPath &path, const CenterLineConfiguration &cfg)
{
    const VoronoiGraph::Nodes &nodes = path.nodes;
    // like side branch separate first node from path
    VoronoiGraph::Path main_path({nodes.begin() + 1, nodes.end()},
                                 path.length);
    double start_offset        = cfg.max_sample_distance - cfg.side_distance;
    SupportIslandPoints result = sample_side_branch(nodes.front(), main_path,
                                                    start_offset, cfg);

    if (path.circles.empty()) return result;
    sample_center_circles(path, cfg, result);
    
    return std::move(result);
}

void SampleIslandUtils::sample_center_circle_end(
    const VoronoiGraph::Node::Neighbor &neighbor,
    double &                            neighbor_distance,
    const VoronoiGraph::Nodes &         done_nodes,
    double &                            node_distance,
    const CenterLineConfiguration &     cfg,
    SupportIslandPoints &               result)
{
    double distance = neighbor_distance + node_distance + neighbor.edge_length;
    if (distance < cfg.max_sample_distance) { // no need add support point
        if (neighbor_distance > node_distance + neighbor.edge_length)
            neighbor_distance = node_distance + neighbor.edge_length;
        if (node_distance > neighbor_distance + neighbor.edge_length)
            node_distance = neighbor_distance + neighbor.edge_length;
        return;
    }
    size_t count_supports = static_cast<size_t>(
        std::floor(distance / cfg.max_sample_distance));
    // distance between support points
    double distance_between = distance / (count_supports + 1);
    if (distance_between < neighbor_distance) {
        // point is calculated to be in done path, SP will be on edge point
        result.push_back(create_point(&neighbor, 1., SupportIslandPoint::Type::center_circle_end));
        neighbor_distance = 0.;
        count_supports -= 1;
        if (count_supports == 0) {
            if (node_distance > neighbor.edge_length)
                node_distance = neighbor.edge_length;
            return;
        }
        distance         = node_distance + neighbor.edge_length;
        distance_between = distance / (count_supports + 1);
    }
    VoronoiGraph::Nodes nodes = done_nodes; // copy, could be more neighbor
    nodes.insert(nodes.begin(), neighbor.node);
    for (int i = 1; i <= count_supports; ++i) {
        double distance_from_neighbor = i * (distance_between) - neighbor_distance;
        result.push_back(
            create_point_on_path(nodes, distance_from_neighbor, SupportIslandPoint::Type::center_circle_end2));
        double distance_support_to_node = fabs(neighbor.edge_length -
                                               distance_from_neighbor);
        if (node_distance > distance_support_to_node)
            node_distance = distance_support_to_node;
    }
}

// DTO store information about distance to nearest support point
// and path from start point
struct NodeDistance
{
    VoronoiGraph::Nodes nodes; // from act node to start
    double              distance_from_support_point;
    NodeDistance(const VoronoiGraph::Node *node,
                 double                    distance_from_support_point)
        : nodes({node})
        , distance_from_support_point(distance_from_support_point)
    {}
};

using SupportDistanceMap = std::map<const VoronoiGraph::Node*, double>;
double get_distance_to_support_point(const VoronoiGraph::Node *node,
                                     const SupportDistanceMap &       support_distance_map,
                                     double                    maximal_search)
{
    auto distance_item = support_distance_map.find(node);
    if (distance_item != support_distance_map.end())
        return distance_item->second;

    // wide search for nearest support point by neighbors
    struct Item
    {
        const VoronoiGraph::Node *prev_node;
        const VoronoiGraph::Node *node;
        double                    act_distance;
        bool                      exist_support_point;
        Item(const VoronoiGraph::Node *prev_node,
             const VoronoiGraph::Node *node,
             double                    act_distance,
             bool                      exist_support_point = false)
            : prev_node(prev_node)
            , node(node)
            , act_distance(act_distance)
            , exist_support_point(exist_support_point)
        {}
    };
    struct OrderDistanceFromNearest
    {
        bool operator()(const Item &first, const Item &second)
        {
            return first.act_distance > second.act_distance;
        }
    };
    std::priority_queue<Item, std::vector<Item>, OrderDistanceFromNearest> process;
    for (const VoronoiGraph::Node::Neighbor &neighbor : node->neighbors)
        process.emplace(node, neighbor.node, neighbor.edge_length);
    
    while (!process.empty()) { 
        Item i = process.top();
        if (i.exist_support_point) return i.act_distance;
        process.pop();
        auto distance_item = support_distance_map.find(i.node);
        if (distance_item != support_distance_map.end()) {
            double distance = i.act_distance + distance_item->second;
            if (distance > maximal_search) continue;
            process.emplace(i.prev_node, i.node, distance, true);
            continue;
        }
        for (const VoronoiGraph::Node::Neighbor &neighbor :i.node->neighbors) {
            if (neighbor.node == i.prev_node) continue;
            double distance = i.act_distance + neighbor.edge_length;
            if (distance > maximal_search) continue;
            process.emplace(i.node, neighbor.node, distance);
        }
    }
    return maximal_search;
}

SupportDistanceMap create_path_distances(
    const std::set<const VoronoiGraph::Node *> &circle_set,
    const std::set<const VoronoiGraph::Node *> &path_set,
    const SupportDistanceMap & support_distance_map,
    double                                      maximal_search)
{
    SupportDistanceMap path_distances;
    for (const VoronoiGraph::Node *node : circle_set) {        
        if (path_set.find(node) == path_set.end()) continue; // lay out of path
        path_distances[node] = get_distance_to_support_point(
            node, support_distance_map, maximal_search);
    }
    return path_distances;
}

// do not use
SupportDistanceMap create_support_distance_map(const SupportIslandPoints &support_points)
{
    SupportDistanceMap support_distance_map;
    for (const SupportIslandPointPtr &support_point : support_points) {
        auto ptr = dynamic_cast<SupportCenterIslandPoint*>(support_point.get()); // bad use
        const VoronoiGraph::Position &position = ptr->position;
        const VoronoiGraph::Node *node = position.neighbor->node; 
        const VoronoiGraph::Node *twin_node = VoronoiGraphUtils::get_twin_node(position.neighbor);
        double distance = (1 - position.ratio) * position.neighbor->edge_length;
        double twin_distance = position.ratio * position.neighbor->edge_length;

        auto item = support_distance_map.find(node);
        if (item == support_distance_map.end()) { 
            support_distance_map[node] = distance;
        } else if (item->second > distance)
            item->second = distance;

        auto twin_item = support_distance_map.find(twin_node);
        if (twin_item == support_distance_map.end()) {
            support_distance_map[twin_node] = twin_distance;
        } else if (twin_item->second > twin_distance)
            twin_item->second = twin_distance;
    }

    return support_distance_map;
}

template<class T, class S, class C>
const S &get_container_ref(const std::priority_queue<T, S, C> &q)
{
    struct HackedQueue : private std::priority_queue<T, S, C>
    {
        static const S &Container(const std::priority_queue<T, S, C> &q)
        {
            return q.*&HackedQueue::c;
        }
    };
    return HackedQueue::Container(q);
}

std::set<const VoronoiGraph::Node *> create_path_set(
    const VoronoiGraph::ExPath &path)
{
    std::queue<const VoronoiGraph::Node *> side_branch_nodes;
    std::set<const VoronoiGraph::Node *> path_set;
    for (const VoronoiGraph::Node *node : path.nodes) {
        path_set.insert(node);
        auto side_branch_item = path.side_branches.find(node);
        if (side_branch_item == path.side_branches.end()) continue;
        side_branch_nodes.push(node);
    }
    while (!side_branch_nodes.empty()) {
        const VoronoiGraph::Node *node = side_branch_nodes.front();
        side_branch_nodes.pop();
        auto side_branch_item = path.side_branches.find(node);
        const std::vector<VoronoiGraph::Path> &side_branches =
            get_container_ref(side_branch_item->second);
        for (const VoronoiGraph::Path& side_branch : side_branches)
            for (const VoronoiGraph::Node *node : side_branch.nodes) {
                path_set.insert(node);
                auto side_branch_item = path.side_branches.find(node);
                if (side_branch_item == path.side_branches.end()) continue;
                side_branch_nodes.push(node);
            }
    }
    return path_set;
}

void SampleIslandUtils::sample_center_circles(
    const VoronoiGraph::ExPath &   path,
    const CenterLineConfiguration &cfg,
    SupportIslandPoints& result)
{
    // vector of connected circle points
    // for detection path from circle
    std::vector<std::set<const VoronoiGraph::Node *>> circles_sets =
        create_circles_sets(path.circles, path.connected_circle);
    std::set<const VoronoiGraph::Node *> path_set = create_path_set(path);
    SupportDistanceMap support_distance_map = create_support_distance_map(result);
    for (const auto &circle_set : circles_sets) {
        SupportDistanceMap  path_distances = create_path_distances(circle_set, path_set, support_distance_map, cfg.max_sample_distance/2);
        SupportIslandPoints circle_result = sample_center_circle(circle_set, path_distances, cfg);
        result.insert(result.end(), 
            std::make_move_iterator(circle_result.begin()),
            std::make_move_iterator(circle_result.end()));
    }
}

SupportIslandPoints SampleIslandUtils::sample_center_circle(
    const std::set<const VoronoiGraph::Node *> &circle_set,
    std::map<const VoronoiGraph::Node *, double>& path_distances,
    const CenterLineConfiguration &             cfg)
{
    SupportIslandPoints result;
    // depth search
    std::stack<NodeDistance> process;

    // path_nodes are already sampled
    for (const auto &path_distanc : path_distances) {
        process.push(NodeDistance(path_distanc.first, path_distanc.second));
    }

    // when node is sampled in all side branches.
    // Value is distance to nearest support point
    std::map<const VoronoiGraph::Node *, double> dones;
    while (!process.empty()) {
        NodeDistance nd = process.top(); // copy
        process.pop();
        const VoronoiGraph::Node *node               = nd.nodes.front();
        const VoronoiGraph::Node *prev_node          = (nd.nodes.size() > 1) ?
                                                           nd.nodes[1] :
                                                           nullptr;
        auto                      done_distance_item = dones.find(node);
        if (done_distance_item != dones.end()) {
            if (done_distance_item->second > nd.distance_from_support_point)
                done_distance_item->second = nd.distance_from_support_point;
            continue;
        }
        // sign node as done with distance to nearest support
        dones[node]                = nd.distance_from_support_point;
        double &node_distance      = dones[node]; // append to done node
        auto    path_distance_item = path_distances.find(node);
        bool is_node_on_path = (path_distance_item != path_distances.end());
        if (is_node_on_path && node_distance > path_distance_item->second)
            node_distance = path_distance_item->second;
        for (const auto &neighbor : node->neighbors) {
            if (neighbor.node == prev_node) continue;
            if (circle_set.find(neighbor.node) == circle_set.end())
                continue; // out of circle points
            auto path_distance_item  = path_distances.find(neighbor.node);
            bool is_neighbor_on_path = (path_distance_item !=
                                        path_distances.end());
            if (is_node_on_path && is_neighbor_on_path)
                continue; // already sampled

            auto neighbor_done_item = dones.find(neighbor.node);
            bool is_neighbor_done   = neighbor_done_item != dones.end();
            if (is_neighbor_done || is_neighbor_on_path) {
                double &neighbor_distance = (is_neighbor_done) ?
                                                neighbor_done_item->second :
                                                path_distance_item->second;
                sample_center_circle_end(neighbor, neighbor_distance,
                                         nd.nodes, node_distance, cfg,
                                         result);
                continue;
            }

            NodeDistance next_nd = nd; // copy
            next_nd.nodes.insert(next_nd.nodes.begin(), neighbor.node);
            next_nd.distance_from_support_point += neighbor.edge_length;
            // exist place for sample:
            while (next_nd.distance_from_support_point >
                   cfg.max_sample_distance) {
                double distance_from_node = next_nd
                                                .distance_from_support_point -
                                            nd.distance_from_support_point;
                double ratio = distance_from_node / neighbor.edge_length;
                result.push_back(
                    create_point(&neighbor, ratio, SupportIslandPoint::Type::center_circle));
                next_nd.distance_from_support_point -= cfg.max_sample_distance;
            }
            process.push(next_nd);
        }
    }
    return result;
}

SupportIslandPoints SampleIslandUtils::sample_expath(
    const VoronoiGraph::ExPath &path, const SampleConfig &config)
{
    // 1) One support point
    if (path.length < config.max_length_for_one_support_point) {
        // create only one point in center
        SupportIslandPoints result;
        result.push_back(create_middle_path_point(
            path, SupportIslandPoint::Type::one_center_point));
        return std::move(result);
    }

    double max_width = VoronoiGraphUtils::get_max_width(path);
    if (max_width < config.max_width_for_center_supportr_line) {
        // 2) Two support points
        if (path.length < config.max_length_for_two_support_points)
            return create_side_points(path.nodes,
                                      config.minimal_distance_from_outline);

        // othewise sample path
        CenterLineConfiguration
            centerLineConfiguration(path.side_branches,
                                    2 * config.minimal_distance_from_outline,
                                    config.max_distance,
                                    config.minimal_distance_from_outline);
        SupportIslandPoints samples = sample_center_line(path, centerLineConfiguration);
        samples.front()->type = SupportIslandPoint::Type::center_line_end2;
        return std::move(samples);
    }

    // line of zig zag points
    if (max_width < config.max_width_for_zig_zag_supportr_line) {
        // return create_zig_zag_points();
    }

    // TODO: 3) Triangle of points
    // eval outline and find three point create almost equilateral triangle

    // TODO: divide path to sampled parts

    SupportIslandPoints points;
    points.emplace_back(create_point_on_path(path.nodes, config.minimal_distance_from_outline));

    return std::move(points);
}

SupportIslandPoints SampleIslandUtils::sample_voronoi_graph(
    const VoronoiGraph &  graph,
    const SampleConfig &  config,
    VoronoiGraph::ExPath &longest_path)
{
    const VoronoiGraph::Node *start_node =
        VoronoiGraphUtils::getFirstContourNode(graph);
    // every island has to have a point on contour
    assert(start_node != nullptr);
    longest_path = VoronoiGraphUtils::create_longest_path(start_node);
    // longest_path = create_longest_path_recursive(start_node);
    return sample_expath(longest_path, config);
}

void SampleIslandUtils::draw(SVG &                      svg,
                             const SupportIslandPoints &supportIslandPoints,
                             double                     size,
                             const char *               color,
                             bool                       write_type)
{
    for (const auto &p : supportIslandPoints) {
        svg.draw(p->point, color, size);
        if (write_type && p->type != SupportIslandPoint::Type::undefined) {
            auto type_name = magic_enum::enum_name(p->type);
            Point start     = p->point + Point(size, 0.);
            svg.draw_text(start, std::string(type_name).c_str(), color);
        }
    }
}
