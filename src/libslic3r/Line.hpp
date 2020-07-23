#ifndef slic3r_Line_hpp_
#define slic3r_Line_hpp_

#include "libslic3r.h"
#include "Point.hpp"

namespace Slic3r {

class BoundingBox;
class Polyline;
class ThickLine;
typedef std::vector<Line> Lines;
typedef std::vector<ThickLine> ThickLines;

// Strip const and volatile qualifiers and references from a type
template<class T> using rm_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

// A simple pair of points
template<size_t N, class T> class VecPair {
public:
    static const constexpr size_t Dim = N;
    using ValT = T;
    using VecT = Vec<N, T>;

    // IsLine_ will be defined as std::true_type if this definition is present
    // wich is true for all subclasses.
    using LineCompatible = void;

    VecT a, b;

    VecPair() : a{VecT::Zero()}, b{VecT::Zero()} {}
    VecPair(const VecT &a_, const VecT &b_) : a{a_}, b{b_} {}
};

// Special case with the Point class
template<> class VecPair<2, coord_t> {
public:
    static const constexpr size_t Dim = 2;
    using ValT = coord_t;
    using VecT = Point;

    using LineCompatible = void;

    Point a, b;
    VecPair() = default;
    VecPair(const Point &a_, const Point &b_) : a{a_}, b{b_} {}
};

namespace line {

// `Line` concept definition (C++17 style) /////////////////////////////////////

// Interface of a class implementing the `Line` concept
template<class L, class Enable = void> struct LineTraits_ {
    static const constexpr size_t Dim = L::Dim;
    using ValT = typename L::ValT;
    using VecT = typename L::VecT;

    static const VecT &a(const L &l) noexcept { return l.a; }
    static const VecT &b(const L &l) noexcept { return l.b; }
    static VecT &a(L &l) noexcept { return l.a; }
    static VecT &b(L &l) noexcept { return l.b; }
};

// Traits should be used with arguments stripped from qualifiers and refs
template<class L> using LineTraits = LineTraits_<rm_cvref_t<L>>;

// Get the dimension of a Line class
template<class L> static const constexpr size_t LineDim = LineTraits<L>::Dim;

// Get the coordinate type used be a Line class
template<class L> using LineValT = typename LineTraits<L>::ValT;

// Get the vector type used by Line class
template<class L> using LineVecT = typename LineTraits<L>::VecT;

// Predicate telling if a class implements the Line concept
template<class L, class Enable = void> struct IsLine_: public std::false_type {};
template<class L> struct IsLine_<L, typename L::LineCompatible>: public std::true_type {};

// Short form for IsLine_<L>::value
template<class L> static const constexpr bool IsLine = IsLine_<rm_cvref_t<L>>::value;

// `Line` concept enforcer
template<class L, class T = L> using LineOnly = std::enable_if_t<IsLine<L>, T>;

// `Line` concept enforcer for line with N dimensions
template<class L, size_t N, class T = L> using LineDimOnly = std::enable_if_t<IsLine<L> && N == LineDim<L>, T>;

// Note: the following methods can take any object that is properly registered
// as a model of the `Line` concept -- by specializing LineTraits_ and IsLine_

// Operations on models of the `Line` concept: /////////////////////////////////

template<class L> const LineVecT<L>& get_a(const L &l) { return LineTraits<L>::a(l); }
template<class L> const LineVecT<L>& get_b(const L &l) { return LineTraits<L>::b(l); }

// Distance to the closest point of line.
template<class L, class = LineOnly<L>>
double distance_to_squared(const L &line, const LineVecT<L> &point)
{
    const Vec<LineDim<L>, double>  v  = line.vector().template cast<double>();
    const Vec<LineDim<L>, double>  va = (point  - get_a(line)).template cast<double>();
    const double  l2 = v.squaredNorm();  // avoid a sqrt
    if (l2 == 0.0)
        // a == b case
        return va.squaredNorm();
    // Consider the line extending the segment, parameterized as a + t (b - a).
    // We find projection of this point onto the line.
    // It falls where t = [(this-a) . (b-a)] / |b-a|^2
    const double t = va.dot(v) / l2;
    if (t < 0.0)      return va.squaredNorm();  // beyond the 'a' end of the segment
    else if (t > 1.0) return (point - get_b(line)).template cast<double>().squaredNorm();  // beyond the 'b' end of the segment
    return (t * v - va).squaredNorm();
}

template<class L, class = LineOnly<L>>
double distance_to(const L &line, const LineVecT<L> &point)
{
    return std::sqrt(distance_to_squared(line, point));
}

template<class L, class Ft, class = FloatingOnly<Ft> >
LineOnly<L> transform(const L& line, const Transform<LineDim<L>, Ft>& t)
{
    using LineInMatrixForm = Eigen::Matrix<LineValT<L>, LineDim<L>, 2>;

    LineInMatrixForm world_line;
    ::memcpy((void*)world_line.col(0).data(), (const void*) get_a(line).data(), LineDim<L> * sizeof(double));
    ::memcpy((void*)world_line.col(1).data(), (const void*) get_b(line).data(), LineDim<L> * sizeof(double));

    LineInMatrixForm local_line = t * world_line.colwise().homogeneous();

    return L(LineVecT<L>(local_line.col(0)), LineVecT<L>(local_line.col(1)));
}

template<class L> LineOnly<L, void> scale(L &line, double factor)
{
    get_a(line) *= factor; get_b(line) *= factor;
}

template<class L> LineOnly<L, void> translate(L &line, const LineVecT<L> &offs)
{
    get_a(line) += offs; get_b(line) += offs;
}

template<class L> LineOnly<L, void> reverse(L &line)
{
    std::swap(get_a(line), get_b(line));
}

template<class L> LineOnly<L, LineVecT<L>> get_vector(const L &line)
{
    return get_b(line) - get_a(line);
}

template<class L> LineOnly<L, double> length(const L &line)
{
    return get_vector(line).template cast<double>().norm();
}

template<class L> LineOnly<L, LineVecT<L> > midpoint(const L &line) {
    return (get_a(line) + get_b(line)) / LineValT<L>{2};
}

template<class L> LineOnly<L, Vec3d> unit_vector(const L &line)
{
    return (length(line) == 0.0) ?
               Vec3d::Zero() :
               get_vector(line).template cast<double>().normalized();
}

template<class L> LineDimOnly<L, 3, Vec3d> intersect_plane(const L &line, double z)
{
    auto   v = get_vector(line).template cast<double>();
    double t = (z - get_a(line).z()) / v.z();
    return {get_a(line).x() + v.x() * t, get_a(line).y() + v.y() * t, z};
}

} // namespace line

using line::transform;

template<size_t N, class T> class Line_: public VecPair<N, T> {
public:
    using VecPair<N, T>::VecPair;
    using VecT = typename VecPair<N, T>::VecT;

    // Define methods for backwards compatibility
    void   scale(double factor) { line::scale(*this, factor); }
    void   translate(double x, double y) { line::translate(*this, {x, y}); }
    void   reverse() { line::reverse(*this); }
    double length() const { return line::length(*this); }
    VecT   midpoint() const { return line::midpoint(*this); }
    double distance_to_squared(const Vec<N, T> &point) const { return line::distance_to_squared(*this, point); }
    double distance_to(const Vec<N, T> &point) const { return line::distance_to(*this, point); }
    VecT   vector() const { return line::get_vector(*this); }
    VecT   unit_vector() const { return line::unit_vector(*this); }
};

class Line: public Line_<2, coord_t>
{
public:
    using Line_<2, coord_t>::Line_;

    explicit operator Lines() const { Lines lines; lines.emplace_back(*this); return lines; }

    // TODO: migrate methods to line:: namespace where feasible...

    void   rotate(double angle, const Point &center) { this->a.rotate(angle, center); this->b.rotate(angle, center); }
    bool   intersection_infinite(const Line &other, Point* point) const;
    bool   operator==(const Line &rhs) const { return this->a == rhs.a && this->b == rhs.b; }
    double perp_distance_to(const Point &point) const;
    bool   parallel_to(double angle) const;
    bool   parallel_to(const Line &line) const { return this->parallel_to(line.direction()); }
    double atan2_() const { return atan2(this->b(1) - this->a(1), this->b(0) - this->a(0)); }
    Vector normal() const { return Vector((this->b(1) - this->a(1)), -(this->b(0) - this->a(0))); }
    double orientation() const;
    double direction() const;

    bool   intersection(const Line& line, Point* intersection) const;
    double ccw(const Point& point) const { return point.ccw(*this); }
    // Clip a line with a bounding box. Returns false if the line is completely outside of the bounding box.
    bool   clip_with_bbox(const BoundingBox &bbox);
};

class ThickLine : public Line
{
public:
    ThickLine() : a_width(0), b_width(0) {}
    ThickLine(const Point& a, const Point& b) : Line(a, b), a_width(0), b_width(0) {}
    ThickLine(const Point& a, const Point& b, double wa, double wb) : Line(a, b), a_width(wa), b_width(wb) {}

    double a_width, b_width;
};

using Line3 = Line_<3, coord_t>;
using Lines3 = std::vector<Line3>;
using Linef = Line_<2, double>;
class Linef3 : public Line_<3, double> {
public:
    using Line_<3, double>::Line_;

    Vec3d intersect_plane(double z) const { return line::intersect_plane(*this, z); }
};

BoundingBox get_extents(const Lines &lines);

} // namespace Slic3r

// start Boost
#include <boost/polygon/polygon.hpp>
namespace boost { namespace polygon {
    template <>
    struct geometry_concept<Slic3r::Line> { typedef segment_concept type; };

    template <>
    struct segment_traits<Slic3r::Line> {
        typedef coord_t coordinate_type;
        typedef Slic3r::Point point_type;

        static inline point_type get(const Slic3r::Line& line, direction_1d dir) {
            return dir.to_int() ? line.b : line.a;
        }
    };
} }
// end Boost

#endif // slic3r_Line_hpp_
