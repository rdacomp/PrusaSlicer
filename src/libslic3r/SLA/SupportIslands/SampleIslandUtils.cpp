#include "SampleIslandUtils.hpp"

#include <cmath>
#include <libslic3r/VoronoiOffset.hpp>
#include "IStackFunction.hpp"
#include "EvaluateNeighbor.hpp"
#include "ParabolaUtils.hpp"
#include "VoronoiGraphUtils.hpp"

#include <magic_enum/magic_enum.hpp>
#include <libslic3r/VoronoiVisualUtils.hpp>

using namespace Slic3r::sla;

Slic3r::Point SampleIslandUtils::get_point_on_path(
    const VoronoiGraph::Nodes &path, double distance)
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
            return VoronoiGraphUtils::get_edge_point(neighbor->edge, ratio);
        }
        prev_node = node;
    }
    // distance must be inside path
    // this means bad input params
    assert(false);
    return Point(0, 0);
}

SupportIslandPoints SampleIslandUtils::create_side_points(
    const VoronoiGraph::Nodes &path, double side_distance)
{
    VoronoiGraph::Nodes reverse_path = path; // copy
    std::reverse(reverse_path.begin(), reverse_path.end());
    return {
        {get_point_on_path(path, side_distance), SupportIslandPoint::Type::two_points},
        {get_point_on_path(reverse_path, side_distance), SupportIslandPoint::Type::two_points}
    };
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
        return {get_point_on_path(reverse_path, cfg.side_distance)};
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
                result.insert(result.end(), side_samples.begin(),
                              side_samples.end());
            }
        }
        while (distance < neighbor->edge_length) {
            double edge_ratio = distance / neighbor->edge_length;
            result.emplace_back(VoronoiGraphUtils::get_edge_point(neighbor->edge, edge_ratio),
                 SupportIslandPoint::Type::center_line);
            distance += sample_distance;
        }
        distance -= neighbor->edge_length;
        prev_node = node;
    }
    assert(fabs(distance - (sample_distance - cfg.side_distance)) < 1e-5);
    return result;
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
        result.insert(result.end(), samples.begin(), samples.end());
        side_branches_cpy.pop();
    }
    return result;
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
    SupportIslandPoints result_circles = sample_center_circles(path, cfg);
    result.insert(result.end(), result_circles.begin(), result_circles.end());
    return result;
}

SupportIslandPoints SampleIslandUtils::sample_center_circle(
    const std::set<const VoronoiGraph::Node *>& circle_set,
    const VoronoiGraph::Nodes& path_nodes,
    const CenterLineConfiguration &             cfg
    )
{
    SupportIslandPoints result;
    // DTO store information about distance to nearest support point
    // and path from start point
    struct NodeDistance
    {
        VoronoiGraph::Nodes nodes; // from act node to start
        double             distance_from_support_point;
        NodeDistance(const VoronoiGraph::Node *node,
                     double                    distance_from_support_point)
            : nodes({node})
            , distance_from_support_point(distance_from_support_point)
        {}
    };
    // depth search
    std::stack<NodeDistance> process;

    // TODO: propagate distance from support point
    // distance from nearest support point 
    double path_distance = cfg.max_sample_distance / 2;
    // path can't be stored in done because it will skip begining of path
    std::map<const VoronoiGraph::Node *, double> path_distances; 
    // path_nodes are already sampled
    for (const VoronoiGraph::Node *node : path_nodes) {
        // contain
        if (circle_set.find(node) != circle_set.end()) {
            process.push(NodeDistance(node, path_distance));
            path_distances[node] = path_distance;
        }
    }

    // TODO: sample circles out of main path
    if (process.empty()) { // TODO: find side branch
    }

    // when node is sampled in all side branches. 
    // Value is distance to nearest support point
    std::map<const VoronoiGraph::Node *, double> done_distance;
    while (!process.empty()) {
        NodeDistance nd = process.top(); // copy
        process.pop();
        const VoronoiGraph::Node *node = nd.nodes.front();
        const VoronoiGraph::Node *prev_node = 
            (nd.nodes.size() > 1) ? nd.nodes[1] : nullptr;
        auto done_distance_item = done_distance.find(node);
        if (done_distance_item != done_distance.end()) { 
            if (done_distance_item->second > nd.distance_from_support_point)
                done_distance_item->second = nd.distance_from_support_point;
            continue; 
        }
        done_distance[node] = nd.distance_from_support_point;
        bool is_node_on_path = (path_distances.find(node) != path_distances.end());
        double &node_distance = done_distance[node]; // append to done node
        for (const auto &neighbor : node->neighbors) {
            if (neighbor.node == prev_node) continue;
            if (circle_set.find(neighbor.node) == circle_set.end()) continue; // out of circle points
            auto path_distance_item  = path_distances.find(neighbor.node);
            bool is_neighbor_on_path = (path_distance_item != path_distances.end());
            if (is_node_on_path && is_neighbor_on_path) continue; // already sampled
            auto done_item = done_distance.find(neighbor.node);
            bool is_done = done_item != done_distance.end();
            if (is_done || is_neighbor_on_path) {
                double &done_distance = (is_done)? done_item->second : path_distance_item->second;
                double  distance      = done_distance +
                                  nd.distance_from_support_point +
                                  neighbor.edge_length;
                if (distance < cfg.max_sample_distance) continue;
                size_t count_supports = static_cast<size_t>(
                    std::floor(distance / cfg.max_sample_distance));
                // distance between support points
                double distance_between = distance / (count_supports + 1);
                if (distance_between < done_distance) {
                    // point is calculated to be in done path, SP will be on edge point
                    result.emplace_back(
                        VoronoiGraphUtils::get_edge_point(neighbor.edge, 1.),
                         SupportIslandPoint::Type::center_circle_end);
                    if (node_distance > neighbor.edge_length)
                        node_distance = neighbor.edge_length;
                    done_distance = 0.;
                    if (count_supports == 1) continue;
                    count_supports -= 1;
                    distance -= done_distance;
                    distance_between = distance / (count_supports + 1);
                }
                VoronoiGraph::Nodes nodes = nd.nodes; // copy, could be more neighbor
                nodes.insert(nodes.begin(), neighbor.node);
                for (int i = 1; i <= count_supports; ++i) {
                    double distance_from_neighbor =
                        i * (distance_between) -done_distance;
                    result.emplace_back(
                        get_point_on_path(nodes, distance_from_neighbor),
                        SupportIslandPoint::Type::center_circle_end2);
                    double distance_to_node = neighbor.edge_length -
                                              distance_from_neighbor;
                    if (distance_to_node > 0. &&
                        node_distance > distance_to_node)
                        node_distance = distance_to_node;
                }
                
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
                result.emplace_back(VoronoiGraphUtils::get_edge_point(neighbor.edge, ratio),
                    SupportIslandPoint::Type::center_circle);
                next_nd.distance_from_support_point -= cfg.max_sample_distance;
            }
            process.push(next_nd);
        }
    }
    return result;
}

SupportIslandPoints SampleIslandUtils::sample_center_circles(
    const VoronoiGraph::ExPath &path, const CenterLineConfiguration &cfg)
{
    // vector of connected circle points
    // for detection path from circle
    std::vector<std::set<const VoronoiGraph::Node *>> circles_sets =
        create_circles_sets(path.circles, path.connected_circle);
    if (circles_sets.size() == 1)
        return sample_center_circle(circles_sets.front(), path.nodes, cfg);

    SupportIslandPoints result;
    for (const auto &circle_set : circles_sets) {
        SupportIslandPoints circle_result = sample_center_circle(circle_set, path.nodes, cfg);
        result.insert(result.end(), circle_result.begin(), circle_result.end());
    }
    return result;
}

SupportIslandPoints SampleIslandUtils::sample_expath(
    const VoronoiGraph::ExPath &path, const SampleConfig &config)
{
    // 1) One support point
    if (path.length < config.max_length_for_one_support_point) {
        // create only one point in center
        Point p = get_center_of_path(path.nodes, path.length);
        SupportIslandPoint supportIslandPoint(p, SupportIslandPoint::Type::one_center_point);
        return {supportIslandPoint};
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

        return sample_center_line(path, centerLineConfiguration);
    }

    // line of zig zag points
    if (max_width < config.max_width_for_zig_zag_supportr_line) {
        // return create_zig_zag_points();
    }

    // TODO: 3) Triangle of points
    // eval outline and find three point create almost equilateral triangle

    // TODO: divide path to sampled parts

    SupportIslandPoints points;
    points.push_back(VoronoiGraphUtils::get_offseted_point(
        *path.nodes.front(), config.minimal_distance_from_outline));

    return points;
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
        svg.draw(p.point, color, size);
        if (write_type && p.type != SupportIslandPoint::Type::undefined) {
            auto type_name = magic_enum::enum_name(p.type);
            Point start     = p.point + Point(size, 0.);
            svg.draw_text(start, std::string(type_name).c_str(), color);
        }
    }
}
