#include "SupportIslandPoint.hpp"
#include "VoronoiGraphUtils.hpp"
#include "LineUtils.hpp"

using namespace Slic3r::sla;

SupportIslandPoint::SupportIslandPoint(Slic3r::Point point, Type type)
    : point(std::move(point)), type(type)
{}

bool SupportIslandPoint::can_move(const Type &type)
{
    // use shorter list
    /*
    static const std::set<Type> can_move({
        Type::center_line,
        Type::center_circle,
        Type::center_circle_end,
        Type::center_circle_end2, 
        Type::outline, 
        Type::inner
    });
    return can_move.find(type) != can_move.end();
    /*/     // switch comment center
    static const std::set<Type> cant_move({
        Type::one_center_point,
        Type::two_points,
        Type::center_line_end,
        Type::center_line_end2,
        Type::center_line_end3,
        Type::center_line_start
    });
    return cant_move.find(type) == cant_move.end();
    //*/
}

bool SupportIslandPoint::can_move() const { return can_move(type); }

coord_t SupportIslandPoint::move(const Point &destination)
{
    Point diff = destination - point;
    point      = destination;
    return abs(diff.x()) + abs(diff.y()); // Manhatn distance
}

std::string SupportIslandPoint::to_string(const Type &type)
{
    static std::map<Type, std::string> type_to_tring=
        {{Type::one_center_point, "one_center_point"},
         {Type::two_points,"two_points"},
         {Type::center_line, "center_line"},
         {Type::center_line_end, "center_line_end"},
         {Type::center_line_end2, "center_line_end2"},
         {Type::center_line_end3, "center_line_end3"},
         {Type::center_line_start, "center_line_start"},
         {Type::center_circle, "center_circle"},
         {Type::center_circle_end, "center_circle_end"},
         {Type::center_circle_end2, "center_circle_end2"},
         {Type::outline, "outline"},
         {Type::inner, "inner"},
         {Type::undefined, "undefined"}};
    auto it = type_to_tring.find(type);
    if (it == type_to_tring.end()) return "UNDEFINED";
    return it->second;
}

///////////////
// Point on VD
///////////////

SupportCenterIslandPoint::SupportCenterIslandPoint(
    VoronoiGraph::Position         position,
    const SampleConfig *   configuration,
    Type                           type)
    : SupportIslandPoint(VoronoiGraphUtils::create_edge_point(position), type)
    , configuration(configuration)
    , position(position)
{}

coord_t SupportCenterIslandPoint::move(const Point &destination)
{        
    // move only along VD
    // TODO: Start respect minimum distance from outline !!
    position =
        VoronoiGraphUtils::align(position, destination,
                                 configuration->max_align_distance);
    Point new_point  = VoronoiGraphUtils::create_edge_point(position);
    return SupportIslandPoint::move(new_point);
}

///////////////
// Point on Outline
///////////////

SupportOutlineIslandPoint::SupportOutlineIslandPoint(
    Position position, std::shared_ptr<Restriction> restriction, Type type)
    : SupportIslandPoint(calc_point(position, *restriction), type)
    , position(position)
    , restriction(std::move(restriction))
{}

bool SupportOutlineIslandPoint::can_move() const { return true; }

coord_t SupportOutlineIslandPoint::move(const Point &destination)
{
    size_t index   = position.index;
    MoveResult closest = create_result(index, destination);

    const double &length = restriction->lengths[position.index];
    double distance = (1.0 - position.ratio) * length;
    while (distance < restriction->max_align_distance) {
        auto next_index = restriction->next_index(index);
        if (!next_index.has_value()) break;
        index = *next_index;
        update_result(closest, index, destination);
        distance += restriction->lengths[index];
    }

    index    = position.index;
    distance = static_cast<coord_t>(position.ratio) * length;
    while (distance < restriction->max_align_distance) {
        auto prev_index = restriction->prev_index(index);
        if (!prev_index.has_value()) break;
        index         = *prev_index;
        update_result(closest, index, destination);
        distance += restriction->lengths[index];
    }

    // apply closest result of move
    this->point = closest.point;
    this->position = closest.position;
    return closest.distance;
}

Slic3r::Point SupportOutlineIslandPoint::calc_point(const Position &position, const Restriction &restriction)
{
    const Line &line = restriction.lines[position.index];
    Point direction = LineUtils::direction(line);
    return line.a + direction * position.ratio;
}

SupportOutlineIslandPoint::MoveResult SupportOutlineIslandPoint::create_result(
    size_t index, const Point &destination)
{
    const Line &line       = restriction->lines[index];
    double      line_ratio_full = LineUtils::foot(line, destination);
    double      line_ratio      = std::clamp(line_ratio_full, 0., 1.);
    Position    new_position(index, line_ratio);
    Point       new_point = calc_point(new_position, *restriction);
    double point_distance = (new_point - destination).cast<double>().norm();
    return MoveResult(new_position, new_point, point_distance);
}

void SupportOutlineIslandPoint::update_result(MoveResult & result,
                                              size_t       index,
                                              const Point &destination)
{
    const Line &line       = restriction->lines[index];
    double      line_ratio_full = LineUtils::foot(line, destination);
    double      line_ratio = std::clamp(line_ratio_full, 0., 1.);
    Position    new_position(index, line_ratio);
    Point       new_point = calc_point(new_position, *restriction);
    Point       diff      = new_point - destination;
    if (abs(diff.x()) > result.distance) return;
    if (abs(diff.y()) > result.distance) return;
    double point_distance = diff.cast<double>().norm();
    if (result.distance > point_distance) {
        result.distance = point_distance;
        result.position = new_position;
        result.point    = new_point;
    }
}