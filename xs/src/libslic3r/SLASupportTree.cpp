#include "SLASupportTree.hpp"
#include "SLABoilerPlate.hpp"

#include "Model.hpp"

#include "boost/geometry/index/rtree.hpp"
#include <igl/ray_mesh_intersect.h>
#include <igl/point_mesh_squared_distance.h>

namespace Slic3r {
namespace sla {

using Coordf = double;
using Portion = std::tuple<double, double>;
inline Portion make_portion(double a, double b) {
    return std::make_tuple(a, b);
}

struct EigenMesh3D {
//    Eigen::Matrix<double, Eigen::Dynamic, 3> V;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
};

using PointSet = Eigen::MatrixXd;

Contour3D sphere(double rho, Portion portion = make_portion(0.0, 2.0*PI),
                 double fa=(2*PI/360)) {

    Contour3D ret;

    // prohibit close to zero radius
    if(rho <= 1e-6 && rho >= -1e-6) return ret;

    auto& vertices = ret.points;
    auto& facets = ret.indices;

    // Algorithm:
    // Add points one-by-one to the sphere grid and form facets using relative
    // coordinates. Sphere is composed effectively of a mesh of stacked circles.

    // adjust via rounding to get an even multiple for any provided angle.
    double angle = (2*PI / floor(2*PI / fa));

    // Ring to be scaled to generate the steps of the sphere
    std::vector<double> ring;

    for (double i = 0; i < 2*PI; i+=angle) ring.emplace_back(i);

    const auto sbegin = size_t(2*std::get<0>(portion)/angle);
    const auto send = size_t(2*std::get<1>(portion)/angle);

    const size_t steps = ring.size();
    const double increment = (double)(1.0 / (double)steps);

    // special case: first ring connects to 0,0,0
    // insert and form facets.
    if(sbegin == 0)
        vertices.emplace_back(Vec3d(0.0, 0.0, -rho + increment*sbegin*2.0*rho));

    auto id = coord_t(vertices.size());
    for (size_t i = 0; i < ring.size(); i++) {
        // Fixed scaling
        const double z = -rho + increment*rho*2.0 * (sbegin + 1.0);
        // radius of the circle for this step.
        const double r = sqrt(abs(rho*rho - z*z));
        Vec2d b = Eigen::Rotation2Dd(ring[i]) * Eigen::Vector2d(0, r);
        vertices.emplace_back(Vec3d(b(0), b(1), z));

        if(sbegin == 0)
        facets.emplace_back((i == 0) ? Vec3crd(coord_t(ring.size()), 0, 1) :
                                       Vec3crd(id - 1, 0, id));
        ++ id;
    }

    // General case: insert and form facets for each step,
    // joining it to the ring below it.
    for (size_t s = sbegin + 2; s < send - 1; s++) {
        const double z = -rho + increment*(double)s*2.0*rho;
        const double r = sqrt(abs(rho*rho - z*z));

        for (size_t i = 0; i < ring.size(); i++) {
            Vec2d b = Eigen::Rotation2Dd(ring[i]) * Eigen::Vector2d(0, r);
            vertices.emplace_back(Vec3d(b(0), b(1), z));
            auto id_ringsize = coord_t(id - ring.size());
            if (i == 0) {
                // wrap around
                facets.emplace_back(Vec3crd(id - 1, id,
                                            id + coord_t(ring.size() - 1)));
                facets.emplace_back(Vec3crd(id - 1, id_ringsize, id));
            } else {
                facets.emplace_back(Vec3crd(id_ringsize - 1, id_ringsize, id));
                facets.emplace_back(Vec3crd(id - 1, id_ringsize - 1, id));
            }
            id++;
        }
    }

    // special case: last ring connects to 0,0,rho*2.0
    // only form facets.
    if(send >= size_t(2*PI / angle)) {
        vertices.emplace_back(Vec3d(0.0, 0.0, -rho + increment*send*2.0*rho));
        for (size_t i = 0; i < ring.size(); i++) {
            auto id_ringsize = coord_t(id - ring.size());
            if (i == 0) {
                // third vertex is on the other side of the ring.
                facets.emplace_back(Vec3crd(id - 1, id_ringsize, id));
            } else {
                auto ci = coord_t(id_ringsize + i);
                facets.emplace_back(Vec3crd(ci - 1, ci, id));
            }
        }
    }
    id++;

    return ret;
}

Contour3D cylinder(double r, double h, double fa=(2*PI/360)) {
    Contour3D ret;

    auto& vertices = ret.points;
    auto& facets = ret.indices;

    // 2 special vertices, top and bottom center, rest are relative to this
    vertices.emplace_back(Vec3d(0.0, 0.0, 0.0));
    vertices.emplace_back(Vec3d(0.0, 0.0, h));

    // adjust via rounding to get an even multiple for any provided angle.
    double angle = (2*PI / floor(2*PI / fa));

    // for each line along the polygon approximating the top/bottom of the
    // circle, generate four points and four facets (2 for the wall, 2 for the
    // top and bottom.
    // Special case: Last line shares 2 vertices with the first line.
    auto id = coord_t(vertices.size() - 1);
    vertices.emplace_back(Vec3d(sin(0) * r , cos(0) * r, 0));
    vertices.emplace_back(Vec3d(sin(0) * r , cos(0) * r, h));
    for (double i = 0; i < 2*PI; i+=angle) {
        Vec2d p = Eigen::Rotation2Dd(i) * Eigen::Vector2d(0, r);
        vertices.emplace_back(Vec3d(p(0), p(1), 0.));
        vertices.emplace_back(Vec3d(p(0), p(1), h));
        id = coord_t(vertices.size() - 1);
        facets.emplace_back(Vec3crd( 0, id - 1, id - 3)); // top
        facets.emplace_back(Vec3crd(id,      1, id - 2)); // bottom
        facets.emplace_back(Vec3crd(id, id - 2, id - 3)); // upper-right of side
        facets.emplace_back(Vec3crd(id, id - 3, id - 1)); // bottom-left of side
    }
    // Connect the last set of vertices with the first.
    facets.emplace_back(Vec3crd( 2, 0, id - 1));
    facets.emplace_back(Vec3crd( 1, 3,     id));
    facets.emplace_back(Vec3crd(id, 3,      2));
    facets.emplace_back(Vec3crd(id, 2, id - 1));

    return ret;
}

struct Head {
    Contour3D mesh;

    size_t steps = 45;
    Vec3d dir = {0, 0, -1};
    Vec3d tr = {0, 0, 0};

    double r_back_mm = 1;
    double r_pin_mm = 0.5;
    double width_mm = 2;
};

Head create_head(double r_big_mm,
                 double r_small_mm,
                 double width_mm,
                 Vec3d dir = {0, 0, -1},    // direction (normal to the "ass" )
                 Vec3d tr = {0, 0, 0},      // displacement
                 const size_t steps = 45) {

    using Quaternion = Eigen::Quaternion<double>;

    // We create two spheres which will be connected with a robe that fits
    // both circles perfectly.

    Head ret;

    // Set up the model detail level
    const double detail = 2*PI/steps;

    // We don't generate whole circles. Instead, we generate only the portions
    // which are visible (not covered by the robe)
    // To know the exact portion of the bottom and top circles we need to use
    // some rules of tangent circles from which we can derive (using simple
    // triangles the following relations:

    // The height of the whole mesh
    const double h = r_big_mm + r_small_mm + width_mm;
    double phi = PI/2 - std::acos( (r_big_mm - r_small_mm) / h );

    // To generate a whole circle we would pass a portion of (0, Pi)
    // To generate only a half horizontal circle we can pass (0, Pi/2)
    // The calculated phi is an offset to the half circles needed to smooth
    // the transition from the circle to the robe geometry

    auto&& s1 = sphere(r_big_mm, make_portion(PI/8, PI/2 + phi), detail);
    auto&& s2 = sphere(r_small_mm, make_portion(PI/2 + phi, PI), detail);

    for(auto& p : s2.points) z(p) += h;

    ret.mesh.merge(s1);
    ret.mesh.merge(s2);

    for(size_t idx1 = s1.points.size() - steps, idx2 = s1.points.size();
        idx1 < s1.points.size() - 1;
        idx1++, idx2++)
    {
        coord_t i1s1 = coord_t(idx1), i1s2 = coord_t(idx2);
        coord_t i2s1 = i1s1 + 1, i2s2 = i1s2 + 1;

        ret.mesh.indices.emplace_back(i1s1, i2s1, i2s2);
        ret.mesh.indices.emplace_back(i1s1, i2s2, i1s2);
    }

    auto i1s1 = coord_t(s1.points.size()) - steps;
    auto i2s1 = coord_t(s1.points.size()) - 1;
    auto i1s2 = coord_t(s1.points.size());
    auto i2s2 = coord_t(s1.points.size()) + steps - 1;

    ret.mesh.indices.emplace_back(i2s2, i2s1, i1s1);
    ret.mesh.indices.emplace_back(i1s2, i2s2, i1s1);

    // To simplify further processing, we translate the mesh so that the
    // last vertex of the pointing sphere (the pinpoint) will be at (0,0,0)

    auto quatern = Quaternion::FromTwoVectors(Vec3d{0, 0, -1}, dir);
    tr(2) -= h + r_small_mm;

    // Then slide everything so that the max z will be (0,0,0) and rotate
    // the whole head into the desired direction
    for(auto& p : ret.mesh.points) {
        p = quatern * p + tr;
    }

    ret.steps = steps;

    // The head's pointing side is facing upwards so this means that it would
    // hold a support point with a normal pointing straight down. This is the
    // reason of the -1 z coordinate
    ret.dir = dir;
    ret.tr = tr;

    ret.r_back_mm = r_big_mm;
    ret.r_pin_mm = r_small_mm;
    ret.width_mm = width_mm;

    return ret;
}

void create_head(TriangleMesh& out, double r1_mm, double r2_mm, double width_mm)
{
    out = mesh(create_head(r1_mm, r2_mm, width_mm).mesh);
}

EigenMesh3D to_eigenmesh(const Model& model) {
    TriangleMesh combined_mesh;

    for(ModelObject *o : model.objects) {
        TriangleMesh tmp = o->raw_mesh();
        for(ModelInstance * inst: o->instances) {
            TriangleMesh ttmp(tmp);
            inst->transform_mesh(&ttmp);
            combined_mesh.merge(ttmp);
        }
    }

    const stl_file& stl = combined_mesh.stl;

    EigenMesh3D outmesh;
    auto& V = outmesh.V;
    auto& F = outmesh.F;

    V.resize(3*stl.stats.number_of_facets, 3);
    F.resize(stl.stats.number_of_facets, 3);
    for (unsigned int i=0; i<stl.stats.number_of_facets; ++i) {
        const stl_facet* facet = stl.facet_start+i;
        V(3*i+0, 0) = facet->vertex[0](0); V(3*i+0, 1) =
                facet->vertex[0](1); V(3*i+0, 2) = facet->vertex[0](2);
        V(3*i+1, 0) = facet->vertex[1](0); V(3*i+1, 1) =
                facet->vertex[1](1); V(3*i+1, 2) = facet->vertex[1](2);
        V(3*i+2, 0) = facet->vertex[2](0); V(3*i+2, 1) =
                facet->vertex[2](1); V(3*i+2, 2) = facet->vertex[2](2);

        F(i, 0) = 3*i+0;
        F(i, 1) = 3*i+1;
        F(i, 2) = 3*i+2;
    }

    return outmesh;
}

Vec3d model_coord(const ModelInstance& object, const Vec3f& mesh_coord) {
    return object.transform_vector(mesh_coord.cast<double>());
}

PointSet support_points(const Model& model) {
    size_t sum = 0;
    for(auto *o : model.objects)
        sum += o->instances.size() * o->sla_support_points.size();

    PointSet ret(sum, 3);

    for(ModelObject *o : model.objects)
        for(ModelInstance *inst : o->instances) {
            int i = 0;
            for(Vec3f& msource : o->sla_support_points) {
                ret.row(i++) = model_coord(*inst, msource);
            }
        }

    return ret;
}

PointSet ground_points(const PointSet& supportps, const EigenMesh3D& mesh) {
    PointSet ret(supportps.rows(), 3);

    for(int i = 0; i < supportps.rows(); i++) {
        Vec3d sp = supportps.row(i);
        igl::Hit hit;
        Vec3d dir(0, 0, -1);
        igl::ray_mesh_intersect(sp, dir, mesh.V, mesh.F, hit);
        ret.row(i) = sp + hit.t*dir;

        // may not need this when the sla pool will be used
        if(ret.row(i)(2) < 0 ) ret.row(i)(2) = 0;
    }

    return ret;
}

Pointf3s ground_points(const Model& model) {
    EigenMesh3D m = to_eigenmesh(model);
    Pointf3s ret;

    for(ModelObject *o : model.objects)
        for(ModelInstance *inst : o->instances) {
            for(Vec3f& msource : o->sla_support_points) {
                auto source = model_coord(*inst, msource);
                igl::Hit hit;
                Vec3d dir(0, 0, -1);
                igl::ray_mesh_intersect(source, dir, m.V, m.F, hit);
                ret.emplace_back(source + hit.t*dir);
            }
        }

    return ret;
}


PointSet normals(const PointSet& points, const EigenMesh3D& mesh) {
    Eigen::VectorXd dists;
    Eigen::VectorXi I;
    PointSet C;
    igl::point_mesh_squared_distance( points, mesh.V, mesh.F, dists, I, C);

    PointSet ret(I.rows(), 3);
    for(int i = 0; i < I.rows(); i++) {
        auto idx = I(i);
        auto trindex = mesh.F.row(idx);

        auto& p1 = mesh.V.row(trindex(0));
        auto& p2 = mesh.V.row(trindex(1));
        auto& p3 = mesh.V.row(trindex(2));

        Eigen::Vector3d U = p2 - p1;
        Eigen::Vector3d V = p3 - p1;
        ret.row(i) = U.cross(V).normalized();
    }

    return ret;
}

using ClusteredPoints = std::array<Pointf3s, 4>;

template<class Vec> double distance(const Vec& p) {
    return std::sqrt(p.transpose() * p);
}

Vec2d to_vec2(const Vec3d& v3) {
    return {v3(0), v3(1)};
}

double distance(const Vec2d& pp1, const Vec2d& pp2) {
    auto p = pp2 - pp1;
    return distance(p);
}

namespace bgi = boost::geometry::index;
using SpatElement = std::pair<Vec2d, unsigned>;
using SpatIndex = bgi::rtree< SpatElement, bgi::rstar<16, 4> /* ? */ >;

bool operator==(const SpatElement& e1, const SpatElement& e2) {
    return e1.second == e2.second;
}

ClusteredPoints cluster(const sla::PointSet& points, double max_distance) {

    SpatIndex sindex;

    for(unsigned idx = 0; idx < points.rows(); idx++)
        sindex.insert( std::make_pair(to_vec2(points.row(idx)), idx));

    ClusteredPoints result;

    using Elems = std::vector<SpatElement>;

    std::function<void(double, Elems&, Elems&)> group =
    [&sindex, &group](double d_max,
              Elems& pts,
              Elems& cluster)
    {
        for(auto& p : pts) {
            std::vector<SpatElement> tmp;

            sindex.query(
                bgi::satisfies( [p, d_max](const SpatElement& v){
                    return distance(p.first, v.first) < d_max;
                }),
                std::back_inserter(tmp)
            );

            auto cmp = [](const SpatElement& e1, const SpatElement& e2){
                return e1.second < e2.second;
            };

            std::sort(tmp.begin(), tmp.end(), cmp);

            Elems newpts;
            std::set_difference(tmp.begin(), tmp.end(),
                                cluster.begin(), cluster.end(),
                                std::back_inserter(newpts), cmp);

            cluster.insert(cluster.end(), newpts.begin(), newpts.end());
            std::sort(cluster.begin(), cluster.end(), cmp);

            if(!newpts.empty()) group(d_max, newpts, cluster);
        }
    };

    std::vector<Elems> clusters;
    for(auto it = sindex.begin(); it != sindex.end();) {
        Elems cluster = {};
        Elems pts = {*it};
        group(max_distance, pts, cluster);

        for(auto& c : cluster) sindex.remove(c);
        it = sindex.begin();

        clusters.emplace_back(cluster);
    }

    int i = 0;
    for(auto& cluster : clusters) {
        std::cout << "cluster no. " << i++ << std::endl;

        for(auto c : cluster)
            std::cout << c.first << " " << c.second << std::endl;

        std::cout << std::endl;
    }

    return result;
}

void create_support_tree(const Model &model,
                         TriangleMesh &output,
                         const SupportConfig& cfg)
{
//    ModelObject *o = model.objects.front();

//    sla::PointSet points(o->sla_support_points.size(), 3);
//    for(int i = 0; i < o->sla_support_points.size(); i++) {
//        points.row(i) = model_coord(*o->instances.front(),
//                                    o->sla_support_points[i]);
//    }

    auto points = support_points(model);

    std::cout << "support points " << points << std::endl;

    auto mesh = sla::to_eigenmesh(model);
    auto nmls = sla::normals(points, mesh);

    std::cout << "normals " << nmls << std::endl;

    PointSet headconns(nmls.rows(), 3);

    for(int i = 0; i < nmls.rows(); i++) {
        auto n = nmls.row(i);
        // for all normals we generate the spherical coordinates and saturate
        // the polar angle to 45 degrees from the bottom then convert back to
        // standard coordinates to get the new normal. Then we just create a
        // quaternion from the two normals (Quaternion::FromTwoVectors) and
        // apply the rotation to the arrow head.

        double z = n(2);
        double r = 1.0;     // for normalized vector
        double polar = std::acos(z / r);
        double azimuth = std::atan2(n(1), n(0));

        if(polar >= PI / 2) { // skip if the tilt is not sane

            // We saturate the polar angle to pi/4 measured from the bottom
            polar = std::max(polar, 3*PI / 4);

            // Reassemble the now corrected normal
            Vec3d nn(std::cos(azimuth) * std::sin(polar),
                     std::sin(azimuth) * std::sin(polar),
                     std::cos(polar));

            // Place to move to
            Vec3d mv = points.row(i);
            double w = cfg.head_width_mm +
                       cfg.head_back_radius_mm +
                       2*cfg.head_front_radius_mm;

            headconns.row(i) = mv + w*nn;

            auto head = create_head(cfg.head_back_radius_mm,
                                    cfg.head_front_radius_mm,
                                    cfg.head_width_mm,
                                    nn,     // dir
                                    mv      // displacement
                                    );

            output.merge(sla::mesh(head.mesh));
        }
    }

    cluster(points, 4 /*mm*/);

    std::cout << "headconns " << headconns << std::endl;
    auto gps = ground_points(headconns, mesh);

    std::cout << "gps " << gps << std::endl;
    auto ds = (headconns - gps);

    for(int i = 0; i < ds.rows(); i++) {
        auto h = ds.row(i) * ds.row(i).transpose();
        std::cout << "h = " << h << std::endl;
        auto cyl = cylinder(0.85*cfg.head_back_radius_mm, std::sqrt(h(0)));
        for(auto& p : cyl.points) p += gps.row(i);
        output.merge(sla::mesh(cyl));

    }

}

}
}
