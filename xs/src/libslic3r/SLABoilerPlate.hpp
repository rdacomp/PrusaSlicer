#ifndef SLABOILERPLATE_HPP
#define SLABOILERPLATE_HPP

#include <iostream>

#include <functional>
#include <numeric>

#include "ExPolygon.hpp"
#include "TriangleMesh.hpp"
#include "ClipperUtils.hpp"

#include "boost/log/trivial.hpp"
// for concave hull merging decisions
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace Slic3r { namespace sla {

/// Index of minimum corner of the box.
std::size_t const min_corner = 0;

/// Index of maximum corner of the box.
std::size_t const max_corner = 1;
}
}

namespace boost {
namespace geometry {
namespace traits {

/* ************************************************************************** */
/* Point concept adaptation ************************************************* */
/* ************************************************************************** */

template<> struct tag<Slic3r::Point> {
    using type = point_tag;
};

template<> struct coordinate_type<Slic3r::Point> {
    using type = coord_t;
};

template<> struct coordinate_system<Slic3r::Point> {
    using type = cs::cartesian;
};

template<> struct dimension<Slic3r::Point>: boost::mpl::int_<2> {};

template<int d> struct access<Slic3r::Point, d > {
    static inline coord_t get(Slic3r::Point const& a) {
        return a(d);
    }

    static inline void set(Slic3r::Point& a, coord_t const& value) {
        a(d) = value;
    }
};

/* ************************************************************************** */
/* Box concept adaptation *************************************************** */
/* ************************************************************************** */

template<> struct tag<Slic3r::BoundingBox> {
    using type = box_tag;
};

template<> struct point_type<Slic3r::BoundingBox> {
    using type = Slic3r::Point;
};

template<std::size_t d>
struct indexed_access<Slic3r::BoundingBox, min_corner, d> {
    static inline coord_t get(Slic3r::BoundingBox const& box) {
        return box.min(d);
    }
    static inline void set(Slic3r::BoundingBox &box, coord_t const& coord) {
        box.min(d) = coord;
    }
};

template<std::size_t d>
struct indexed_access<Slic3r::BoundingBox, max_corner, d> {
    static inline coord_t get(Slic3r::BoundingBox const& box) {
        return box.max(d);
    }
    static inline void set(Slic3r::BoundingBox &box, coord_t const& coord) {
        box.max(d) = coord;
    }
};

}
}
}

namespace Slic3r {
namespace sla {

namespace bgi = boost::geometry::index;
using SpatElement = std::pair<BoundingBox, unsigned>;
using SpatIndex = bgi::rtree< SpatElement, bgi::rstar<16, 4> >;

using coord_t = Point::coord_type;

/// get the scaled clipper units for a millimeter value
inline coord_t mm(double v) { return coord_t(v/SCALING_FACTOR); }

/// Get x and y coordinates (because we are eigenizing...)
inline coord_t x(const Point& p) { return p(0); }
inline coord_t y(const Point& p) { return p(1); }
inline coord_t& x(Point& p) { return p(0); }
inline coord_t& y(Point& p) { return p(1); }

inline coordf_t x(const Vec3d& p) { return p(0); }
inline coordf_t y(const Vec3d& p) { return p(1); }
inline coordf_t z(const Vec3d& p) { return p(2); }
inline coordf_t& x(Vec3d& p) { return p(0); }
inline coordf_t& y(Vec3d& p) { return p(1); }
inline coordf_t& z(Vec3d& p) { return p(2); }

inline coord_t& x(Vec3crd& p) { return p(0); }
inline coord_t& y(Vec3crd& p) { return p(1); }
inline coord_t& z(Vec3crd& p) { return p(2); }
inline coord_t x(const Vec3crd& p) { return p(0); }
inline coord_t y(const Vec3crd& p) { return p(1); }
inline coord_t z(const Vec3crd& p) { return p(2); }

inline void triangulate(const ExPolygon& expoly, Polygons& triangles) {
    expoly.triangulate_p2t(&triangles);
}

inline Polygons triangulate(const ExPolygon& expoly) {
    Polygons tri; triangulate(expoly, tri); return tri;
}

using Indices = std::vector<Vec3crd>;

/// Intermediate struct for a 3D mesh
struct Contour3D {
    Pointf3s points;
    Indices indices;

    void merge(const Contour3D& ctr) {
        auto s3 = coord_t(points.size());
        auto s = coord_t(indices.size());

        points.insert(points.end(), ctr.points.begin(), ctr.points.end());
        indices.insert(indices.end(), ctr.indices.begin(), ctr.indices.end());

        for(auto n = s; n < indices.size(); n++) {
            auto& idx = indices[n]; x(idx) += s3; y(idx) += s3; z(idx) += s3;
        }
    }
};

/// Convert the triangulation output to an intermediate mesh.
Contour3D convert(const Polygons& triangles, coord_t z, bool dir);

/// Mesh from an existing contour.
inline TriangleMesh mesh(const Contour3D& ctour) {
    return {ctour.points, ctour.indices};
}

/// Mesh from an evaporating 3D contour
inline TriangleMesh mesh(Contour3D&& ctour) {
    return {std::move(ctour.points), std::move(ctour.indices)};
}

}
}

#endif // SLABOILERPLATE_HPP
