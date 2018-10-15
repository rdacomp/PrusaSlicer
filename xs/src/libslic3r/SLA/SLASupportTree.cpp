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

    struct Tail {
        Contour3D mesh;
        size_t steps = 45;
        double length = 3;
    } tail;

    Head(double r_big_mm,
         double r_small_mm,
         double length_mm,
         Vec3d direction = {0, 0, -1},    // direction (normal to the "ass" )
         Vec3d offset = {0, 0, 0},        // displacement
         const size_t circlesteps = 45):
            steps(circlesteps), dir(direction), tr(offset),
            r_back_mm(r_big_mm), r_pin_mm(r_small_mm), width_mm(length_mm)
    {

        // We create two spheres which will be connected with a robe that fits
        // both circles perfectly.

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

        auto&& s1 = sphere(r_big_mm, make_portion(0, PI/2 + phi), detail);
        auto&& s2 = sphere(r_small_mm, make_portion(PI/2 + phi, PI), detail);

        for(auto& p : s2.points) z(p) += h;

        mesh.merge(s1);
        mesh.merge(s2);

        for(size_t idx1 = s1.points.size() - steps, idx2 = s1.points.size();
            idx1 < s1.points.size() - 1;
            idx1++, idx2++)
        {
            coord_t i1s1 = coord_t(idx1), i1s2 = coord_t(idx2);
            coord_t i2s1 = i1s1 + 1, i2s2 = i1s2 + 1;

            mesh.indices.emplace_back(i1s1, i2s1, i2s2);
            mesh.indices.emplace_back(i1s1, i2s2, i1s2);
        }

        auto i1s1 = coord_t(s1.points.size()) - steps;
        auto i2s1 = coord_t(s1.points.size()) - 1;
        auto i1s2 = coord_t(s1.points.size());
        auto i2s2 = coord_t(s1.points.size()) + steps - 1;

        mesh.indices.emplace_back(i2s2, i2s1, i1s1);
        mesh.indices.emplace_back(i1s2, i2s2, i1s1);

        // To simplify further processing, we translate the mesh so that the
        // last vertex of the pointing sphere (the pinpoint) will be at (0,0,0)
        for(auto& p : mesh.points) { z(p) -= (h + r_small_mm); }

    }

    void transform()
    {
        using Quaternion = Eigen::Quaternion<double>;

        // We rotate the head to the specified direction The head's pointing
        // side is facing upwards so this means that it would hold a support
        // point with a normal pointing straight down. This is the reason of
        // the -1 z coordinate
        auto quatern = Quaternion::FromTwoVectors(Vec3d{0, 0, -1}, dir);
        for(auto& p : mesh.points) {
            p = quatern * p + tr;
        }
    }

    void add_tail(double length, double radius = -1) {
        auto& cntr = tail.mesh;
        Head& head = *this;
        tail.length = length;

        cntr.points.reserve(2*steps);

        auto h = head.r_back_mm + 2*head.r_pin_mm + head.width_mm;
        Vec3d c = head.tr + head.dir * h;

        double r = head.r_back_mm * 0.9;
        double r_low = radius < 0 || radius > head.r_back_mm * 0.65 ?
                       head.r_back_mm * 0.65 : radius;

        double a = 2*PI/steps;
        double z = c(2);
        for(int i = 0; i < steps; ++i) {
            double phi = i*a;
            double x = c(0) + r*std::cos(phi);
            double y = c(1) + r*std::sin(phi);
            cntr.points.emplace_back(x, y, z);
        }

        for(int i = 0; i < steps; ++i) {
            double phi = i*a;
            double lx = c(0) + r_low*std::cos(phi);
            double ly = c(1) + r_low*std::sin(phi);
            cntr.points.emplace_back(lx, ly, z - length);
        }

        cntr.indices.reserve(2*steps);
        auto offs = steps;
        for(int i = 0; i < steps - 1; ++i) {
            cntr.indices.emplace_back(i, i + offs, offs + i + 1);
            cntr.indices.emplace_back(i, offs + i + 1, i + 1);
        }

        auto last = steps - 1;
        cntr.indices.emplace_back(0, last, offs);
        cntr.indices.emplace_back(last, offs + last, offs);
    }
};

struct ColumnStick {
    Contour3D mesh;
    Contour3D base;
    double r = 1;
    size_t steps = 0;
    Vec3d endpoint;

    ColumnStick(const Head& head, const Vec3d& endp, double radius = 1) :
        endpoint(endp)
    {
        steps = head.steps;

        r = radius < head.r_back_mm ? radius : head.r_back_mm * 0.65;

        auto& points = mesh.points; points.reserve(head.tail.steps*2);
        points.insert(points.end(),
                      head.tail.mesh.points.begin() + steps,
                      head.tail.mesh.points.end()
                      );

        for(auto it = head.tail.mesh.points.begin() + steps;
            it != head.tail.mesh.points.end();
            ++it)
        {
            auto& s = *it;
            points.emplace_back(s(0), s(1), endp(2));
        }

        auto& indices = mesh.indices;
        auto offs = steps;
        for(int i = 0; i < steps - 1; ++i) {
            indices.emplace_back(i, i + offs, offs + i + 1);
            indices.emplace_back(i, offs + i + 1, i + 1);
        }

        auto last = steps - 1;
        indices.emplace_back(0, last, offs);
        indices.emplace_back(last, offs + last, offs);
    }

    void add_base(double height = 3, double radius = 2) {
        if(height <= 0) return;

        if(radius < r ) radius = r;

        double a = 2*PI/steps;
        double z = endpoint(2) + height;
        for(int i = 0; i < steps; ++i) {
            double phi = i*a;
            double x = endpoint(0) + 0.5*(radius + r)*std::cos(phi);
            double y = endpoint(1) + 0.5*(radius + r)*std::sin(phi);
            base.points.emplace_back(x, y, z);
        }

        for(int i = 0; i < steps; ++i) {
            double phi = i*a;
            double x = endpoint(0) + radius*std::cos(phi);
            double y = endpoint(1) + radius*std::sin(phi);
            base.points.emplace_back(x, y, z - height);
        }

        auto ep = endpoint; ep(2) += height;
        base.points.emplace_back(endpoint);
        base.points.emplace_back(ep);

        auto& indices = base.indices;
        auto hcenter = base.points.size() - 1;
        auto lcenter = base.points.size() - 2;
        auto offs = steps;
        for(int i = 0; i < steps - 1; ++i) {
            indices.emplace_back(i, i + offs, offs + i + 1);
            indices.emplace_back(i, offs + i + 1, i + 1);
            indices.emplace_back(i, i + 1, hcenter);
            indices.emplace_back(lcenter, offs + i + 1, offs + i);
        }

        auto last = steps - 1;
        indices.emplace_back(0, last, offs);
        indices.emplace_back(last, offs + last, offs);
        indices.emplace_back(hcenter, last, 0);
        indices.emplace_back(offs, offs + last, lcenter);

    }
};

struct Junction {
    Contour3D mesh;
    double r = 1;

    Junction(const Vec3d& pos, double r_mm): r(r_mm) {}
};

struct BridgeStick {
    Contour3D mesh;
    double r = 0.8;

    BridgeStick(const Junction& j1, const Junction& j2) {}

    BridgeStick(const Head& h, const Junction& j2) {}

    BridgeStick(const Junction& j, const ColumnStick& cl) {}

};

EigenMesh3D to_eigenmesh(const Contour3D& cntr) {
    EigenMesh3D emesh;

    auto& V = emesh.V;
    auto& F = emesh.F;

    V.resize(cntr.points.size(), 3);
    F.resize(cntr.indices.size(), 3);

    for (int i = 0; i < V.rows(); ++i) {
        V.row(i) = cntr.points[i];
        F.row(i) = cntr.indices[i];
    }

    return emesh;
}

void create_head(TriangleMesh& out, double r1_mm, double r2_mm, double width_mm)
{
    Head head(r1_mm, r2_mm, width_mm, {0, std::sqrt(0.5), -std::sqrt(0.5)},
              {0, 0, 30});
    out.merge(mesh(head.mesh));
    out.merge(mesh(head.tail.mesh));

    ColumnStick cst(head, {0, 0, 0});
    cst.add_base();

    out.merge(mesh(cst.mesh));
    out.merge(mesh(cst.base));
}

//enum class ClusterType: double {
static const double /*constexpr*/ D_SP   = 3;
static const double /*constexpr*/ D_BRIDGED_TRIO  = 3;
//static const double /*constexpr*/ D_SSDH = 1.0;  // Same stick different heads
//static const double /*constexpr*/ D_DHCS = 3.0;  // different heads, connected sticks
//static const double /*constexpr*/ D_DH3S = 5.0;  // different heads, additional 3rd stick
//static const double /*constexpr*/ D_DHDS = 8.0;  // different heads, different stick
//};

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

template<class Vec> double distance(const Vec& p) {
    return std::sqrt(p.transpose() * p);
}

Vec2d to_vec2(const Vec3d& v3) {
    return {v3(0), v3(1)};
}

template<class Vec> double distance(const Vec& pp1, const Vec& pp2) {
    auto p = pp2 - pp1;
    return distance(p);
}

namespace bgi = boost::geometry::index;
using SpatElement = std::pair<Vec3d, unsigned>;
using SpatIndex = bgi::rtree< SpatElement, bgi::rstar<16, 4> /* ? */ >;

using ClusteredPoints = std::vector<std::vector<unsigned>>;

bool operator==(const SpatElement& e1, const SpatElement& e2) {
    return e1.second == e2.second;
}

ClusteredPoints cluster(
        const sla::PointSet& points,
        std::function<bool(const SpatElement&, const SpatElement&)> pred,
        unsigned max_points = 0) {

    SpatIndex sindex;

    for(unsigned idx = 0; idx < points.rows(); idx++)
        sindex.insert( std::make_pair(points.row(idx), idx));

    using Elems = std::vector<SpatElement>;

    std::function<void(Elems&, Elems&)> group =
    [&sindex, &group, pred, max_points](Elems& pts, Elems& cluster)
    {
        for(auto& p : pts) {
            std::vector<SpatElement> tmp;

            sindex.query(
                bgi::satisfies([p, pred](const SpatElement& se) { return pred(p, se); }),
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

            int c = max_points && newpts.size() + cluster.size() > max_points?
                        int(max_points - cluster.size()) : int(newpts.size());

            cluster.insert(cluster.end(), newpts.begin(), newpts.begin() + c);
            std::sort(cluster.begin(), cluster.end(), cmp);

            if(!newpts.empty() && (!max_points || cluster.size() < max_points))
                group(newpts, cluster);
        }
    };

    std::vector<Elems> clusters;
    for(auto it = sindex.begin(); it != sindex.end();) {
        Elems cluster = {};
        Elems pts = {*it};
        group(pts, cluster);

        for(auto& c : cluster) sindex.remove(c);
        it = sindex.begin();

        clusters.emplace_back(cluster);
    }

    ClusteredPoints result;
    for(auto& cluster : clusters) {
        result.emplace_back();
        for(auto c : cluster) result.back().emplace_back(c.second);
    }

    return result;
}

class SLASupportTree::Impl {
public:
    std::vector<Head> heads;
    std::vector<ColumnStick> column_sticks;
    std::vector<Junction> junctions;
    std::vector<BridgeStick> bridge_sticks;
};

bool SLASupportTree::generate(const Model& model,
                              const SupportConfig& cfg,
                              const Controller& ctl) {
    auto points = support_points(model);
    auto mesh = sla::to_eigenmesh(model);

    PointSet filtered_pts;
    PointSet pt_normals;
    PointSet head_positions;
    auto& result = *m_impl;

    enum Steps {
        BEGIN,
        FILTER,
        PINHEADS,
        CLASSIFY,
        DONE,
        HALT,
        ABORT,
        NUM_STEPS
        //...
    };

    auto filterfn =
    [&points, &filtered_pts, &pt_normals, &head_positions, &mesh, cfg] () {

        /* ******************************************************** */
        /* Filtering step                                           */
        /* ******************************************************** */

        auto aliases = cluster(points,
                               [cfg](const SpatElement& p,
                               const SpatElement& se){
            return distance(Vec2d(p.first(0), p.first(1)),
                            Vec2d(se.first(0), se.first(1))) <
                    2*cfg.base_radius_mm;
        }, 2);

        filtered_pts.resize(aliases.size(), 3);
        int count = 0;
        for(auto& a : aliases) {
            filtered_pts.row(count++) = points.row(a.front());
        }

        auto nmls = sla::normals(filtered_pts, mesh);

        pt_normals.resize(count, 3);
        head_positions.resize(count, 3);

        PointSet headconns(count, 3);

        // Not all of the support points have to be a valid position for
        // support creation. The angle may be inappropriate or there may
        // not be enough space for the pinhead. Filtering is applied for
        // these reasons.

        int pcount = 0;
        for(int i = 0; i < count; i++) {
            auto n = nmls.row(i);

            // for all normals we generate the spherical coordinates and
            // saturate the polar angle to 45 degrees from the bottom then
            // convert back to standard coordinates to get the new normal.
            // Then we just create a quaternion from the two normals
            // (Quaternion::FromTwoVectors) and apply the rotation to the
            // arrow head.

            double z = n(2);
            double r = 1.0;     // for normalized vector
            double polar = std::acos(z / r);
            double azimuth = std::atan2(n(1), n(0));

            if(polar >= PI / 2) { // skip if the tilt is not sane

                // We saturate the polar angle to 3pi/4
                polar = std::max(polar, 3*PI / 4);

                // Reassemble the now corrected normal
                Vec3d nn(std::cos(azimuth) * std::sin(polar),
                         std::sin(azimuth) * std::sin(polar),
                         std::cos(polar));

                // save the verified and corrected normal
                pt_normals.row(pcount) = nn;

                // save the head (pinpoint) position
                head_positions.row(pcount) = filtered_pts.row(i);

                // the full width of the head
                double w = cfg.head_width_mm +
                           cfg.head_back_radius_mm +
                           2*cfg.head_front_radius_mm;

                // position to start the column sticks (TODO may not need them)
                headconns.row(pcount) = Vec3d(filtered_pts.row(i)) + w*nn;
                ++pcount;
            }
        }
    };

    auto pinheadfn = [&pt_normals, &head_positions, &result, cfg] () {

        /* ******************************************************** */
        /* Generating Pinheads                                      */
        /* ******************************************************** */

        for (int i = 0; i < pt_normals.rows(); ++i) {
            result.heads.emplace_back(
                        cfg.head_back_radius_mm,
                        cfg.head_front_radius_mm,
                        cfg.head_width_mm,
                        pt_normals.row(i),         // dir
                        head_positions.row(i)      // displacement
                        );
        }
    };

    auto classifyfn = [&filtered_pts, &result, &mesh, cfg] () {

        /* ******************************************************** */
        /* Classification                                           */
        /* ******************************************************** */

        // search for suitable trios
        auto trios = cluster(filtered_pts, [](const SpatElement& p,
                             const SpatElement& se) {
            return distance(p.first, se.first) < D_BRIDGED_TRIO;
        }, 3);

        for(auto& trio: trios) {

        }

        // TODO: only some heads will receive a column stick
        for(auto& head : result.heads) {
            head.add_tail(2, cfg.pillar_radius_mm);
            head.transform();

            double headsize = 2*head.r_pin_mm + head.width_mm +
                              head.r_back_mm;
            Vec3d startpoint = head.tr + head.dir*headsize;

            igl::Hit hit;
            hit.t = std::numeric_limits<float>::infinity();

            Vec3d dir(0, 0, -1);
            igl::ray_mesh_intersect(startpoint, dir, mesh.V, mesh.F, hit);

            Vec3d gp = startpoint + hit.t*dir;
            bool ground = false;

            if(std::isinf(hit.t) || std::isnan(hit.t)) {
                gp = startpoint; gp(2) = 0;
                ground = true;
            } else {
                gp(2) += (headsize - head.r_pin_mm) ;
            }

            auto endpoint = gp;

            ColumnStick cs(head, endpoint, cfg.pillar_radius_mm);

            if(ground)
                cs.add_base(cfg.base_height_mm, cfg.base_radius_mm);
            else {
                Head base_head(cfg.head_back_radius_mm,
                     cfg.head_front_radius_mm,
                     cfg.head_width_mm,
                     {0.0, 0.0, 1.0},
                     {gp(0), gp(1), gp(2) - headsize});
                base_head.transform();
                cs.base = base_head.mesh;
            }

            result.column_sticks.emplace_back(cs);
        }
    };

    std::array<std::function<void()>, NUM_STEPS> program = {
        [] () {
            // Begin
            // clear up the shared data
        },
        filterfn,
        pinheadfn,
        classifyfn,
        [] () {
            // Done
        },
        [] () {
            // Halt
        },
        [] () {
            // Abort
        }
    };

    Steps pc = BEGIN, pc_prev = BEGIN;

    auto progress = [&ctl, &model, &pc, &pc_prev] () {
        static const std::array<std::string, NUM_STEPS> stepstr {
            ""
            "Filtering",
            "Generate pinheads"
            "Classification",
            "Done",
            "Halt",
            "Abort"
        };

        static const std::array<unsigned, NUM_STEPS> stepstate {
            0,
            10,
            30,
            50,
            100,
            0,
            0
        };

        auto cmd = ctl.nextcmd(/* block if: */ pc == HALT);

        switch(cmd) {
        case Controller::Cmd::START_RESUME:
            switch(pc) {
            case BEGIN: pc = FILTER; break;
            case FILTER: pc = PINHEADS; break;
            case PINHEADS: pc = CLASSIFY; break;
            case CLASSIFY: pc = DONE; break;
            case HALT: pc = pc_prev; break;
            case ABORT: break; // we should never get here
            }
            ctl.statuscb(stepstate[pc], stepstr[pc]);
            break;
        case Controller::Cmd::PAUSE:
            pc_prev = pc;
            pc = HALT;
            ctl.statuscb(stepstate[pc], stepstr[pc]);
            break;
        case Controller::Cmd::STOP:
            pc = ABORT; ctl.statuscb(stepstate[pc], stepstr[pc]); break;
        case Controller::Cmd::SYNCH:
            pc = BEGIN;
            // TODO
        }
    };

    while(pc < DONE || pc == HALT) {
        progress();
        program[pc]();
    }

    return pc == ABORT;

//    if(!progress(10, "Filtering")) return false;

//    // find small clusters of very close points which can be treated as the same
//    auto aliases = cluster(points, D_SP, 2);
//    PointSet filtered_pts(points.rows(), 3);
//    int count = 0;
//    for(auto& a : aliases) {
//        std::cout << "a size = " << a.size() << std::endl;
//        std::cout << "idx = " << a.front() << std::endl;
//        filtered_pts.row(count++) = points.row(a.front());
//    }
//    filtered_pts.conservativeResize(count, Eigen::NoChange);

//    auto nmls = sla::normals(filtered_pts, mesh);

//    PointSet head_positions(count, 3);
//    PointSet correct_normals(count, 3);
//    PointSet headconns(count, 3);

//    // Not all of the support points have to be a valid position for support
//    // creation. The angle may be inappropriate or there may not be enough space
//    // for the pinhead. Filtering is applied for these reasons.
//    int pcount = 0;
//    for(int i = 0; i < count; i++) {
//        auto n = nmls.row(i);
//        // for all normals we generate the spherical coordinates and saturate
//        // the polar angle to 45 degrees from the bottom then convert back to
//        // standard coordinates to get the new normal. Then we just create a
//        // quaternion from the two normals (Quaternion::FromTwoVectors) and
//        // apply the rotation to the arrow head.

//        double z = n(2);
//        double r = 1.0;     // for normalized vector
//        double polar = std::acos(z / r);
//        double azimuth = std::atan2(n(1), n(0));

//        if(polar >= PI / 2) { // skip if the tilt is not sane

//            // We saturate the polar angle to pi/4 measured from the bottom
//            polar = std::max(polar, 3*PI / 4);

//            // Reassemble the now corrected normal
//            Vec3d nn(std::cos(azimuth) * std::sin(polar),
//                     std::sin(azimuth) * std::sin(polar),
//                     std::cos(polar));

//            // save the verified and corrected normal
//            correct_normals.row(pcount) = nn;

//            // save the head (pinpoint) position
//            head_positions.row(pcount) = filtered_pts.row(i);

//            // the full width of the head
//            double w = cfg.head_width_mm +
//                       cfg.head_back_radius_mm +
//                       2*cfg.head_front_radius_mm;

//            // position to start the column sticks (TODO may not need them)
//            headconns.row(pcount) = Vec3d(filtered_pts.row(i)) + w*nn;
//            ++pcount;
//        }
//    }

//    /* ********************************************************************** */
//    /* Generate the heads                                                     */
//    /* ********************************************************************** */
//    if(!progress(20, "Generating heads")) return false;

//    for (int i = 0; i < pcount; ++i) {

//        m_impl->heads.emplace_back(cfg.head_back_radius_mm,
//                                   cfg.head_front_radius_mm,
//                                   cfg.head_width_mm,
//                                   correct_normals.row(i),     // dir
//                                   head_positions.row(i)      // displacement
//                                  );
//    }

//    /* ********************************************************************** */
//    /* Classification                                                         */
//    /* ********************************************************************** */

//    if(!progress(10, "Classify support points")) return false;

//    // We search for clusters where the points are in a certain distance class
//    // (interval). Each of the classes will be treated differently when the
//    // support columns and their connections are going to be generated.

//    cluster(filtered_pts, D_BRIDGED_TRIO /*mm*/, 3);

////    std::cout << "headconns " << headconns << std::endl;
////    auto gps = ground_points(headconns, mesh);

////    std::cout << "gps " << gps << std::endl;
////    auto ds = (headconns - gps);

////    for(int i = 0; i < ds.rows(); i++) {
////        auto h = ds.row(i) * ds.row(i).transpose();
////        std::cout << "h = " << h << std::endl;
////        auto cyl = cylinder(0.85*cfg.head_back_radius_mm, std::sqrt(h(0)));
////        for(auto& p : cyl.points) p += gps.row(i);
////        output.merge(sla::mesh(cyl));
////    }

//    return pc == ABORT;
//    return progress(100, "Done");
}

SLASupportTree::SLASupportTree(): m_impl(new Impl()) {}

SLASupportTree::SLASupportTree(const SLASupportTree &c):
    m_impl( new Impl(*c.m_impl)) {}

SLASupportTree &SLASupportTree::operator=(const SLASupportTree &c)
{
    m_impl = make_unique<Impl>(*c.m_impl);
    return *this;
}

SLASupportTree::~SLASupportTree() {}

void add_sla_supports(Model &model,
                      const SupportConfig &cfg,
                      const Controller &ctl)
{
    SLASupportTree _stree;
    _stree.generate(model, cfg, ctl);

    SLASupportTree::Impl& stree = _stree.get();
    ModelObject* o = model.add_object();
    o->add_instance();

    for(auto& head : stree.heads) {
        o->add_volume(mesh(head.mesh));
        o->add_volume(mesh(head.tail.mesh));
    }

    for(auto& stick : stree.column_sticks) {
        o->add_volume(mesh(stick.mesh));
        o->add_volume(mesh(stick.base));
    }

}

}
}
