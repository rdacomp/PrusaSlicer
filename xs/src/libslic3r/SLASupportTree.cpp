#include "SLASupportTree.hpp"
#include "SLABoilerPlate.hpp"

#include "Model.hpp"
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

Contour3D create_head(double r1_mm, double r2_mm, double width_mm) {

    Contour3D ret;

    const size_t steps = 45;
    const double detail = 2*PI/steps;

    const double h = r1_mm + r2_mm + width_mm;

    double phi = PI/2 - std::acos( (r1_mm - r2_mm)/(r1_mm + r2_mm + width_mm));
    std::cout << "phi is: " << phi*180/PI << std::endl;

    auto&& s1 = sphere(r1_mm, make_portion(0, PI/2 + phi), detail);
    auto&& s2 = sphere(r2_mm, make_portion(PI/2 + phi, PI), detail);

    for(auto& p : s2.points) z(p) += h;

    ret.merge(s1);
    ret.merge(s2);

    for(size_t idx1 = s1.points.size() - steps, idx2 = s1.points.size();
        idx1 < s1.points.size() - 1;
        idx1++, idx2++)
    {
        coord_t i1s1 = coord_t(idx1), i1s2 = coord_t(idx2);
        coord_t i2s1 = i1s1 + 1, i2s2 = i1s2 + 1;

        ret.indices.emplace_back(i1s1, i2s1, i2s2);
        ret.indices.emplace_back(i1s1, i2s2, i1s2);
    }

    auto i1s1 = coord_t(s1.points.size()) - steps;
    auto i2s1 = coord_t(s1.points.size()) - 1;
    auto i1s2 = coord_t(s1.points.size());
    auto i2s2 = coord_t(s1.points.size()) + steps - 1;

    ret.indices.emplace_back(i2s2, i2s1, i1s1);
    ret.indices.emplace_back(i1s2, i2s2, i1s1);

    return ret;
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


Pointf3s normals(const PointSet& points, const EigenMesh3D& mesh) {
    Eigen::VectorXd dists;
    Eigen::VectorXi I;
    PointSet C;
    igl::point_mesh_squared_distance( points, mesh.V, mesh.F, dists, I, C);

    Pointf3s ret;
    for(int i= 0; i < I.rows(); i++) {
        auto idx = I(i);
        auto trindex = mesh.F.row(idx);

        // TODO: do it in one line
        auto& p1 = mesh.V.row(trindex(0));
        auto& p2 = mesh.V.row(trindex(1));
        auto& p3 = mesh.V.row(trindex(2));

        Eigen::Vector3d U = p2 - p1;
        Eigen::Vector3d V = p3 - p1;
        ret.emplace_back(U.cross(V));
    }

    return ret;
}

void create_support_tree(const Model &model,
                         TriangleMesh &output,
                         const SupportConfig& cfg)
{
    ModelObject *o = model.objects.front();

    auto mesh = sla::to_eigenmesh(model);
    sla::PointSet points(o->sla_support_points.size(), 3);
    for(int i = 0; i < o->sla_support_points.size(); i++) {
        points.row(i) = model_coord(*o->instances.front(), o->sla_support_points[i]);
    }

    sla::normals(points, mesh);
//    auto gpoints = ground_points(model);

//    std::cout << "ground points: " << gpoints.size() << std::endl;
//    output = mesh(create_head(cfg.head_back_radius_mm,
//                              cfg.head_front_radius_mm,
//                              cfg.head_width_mm));
}

}
}
