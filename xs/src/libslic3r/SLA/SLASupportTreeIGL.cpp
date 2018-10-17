#include "SLA/SLASupportTree.hpp"
#include "SLA/SLABoilerPlate.hpp"

// HEAVY headers... takes eternity to compile

// for concave hull merging decisions
#include "SLABoostAdapter.hpp"
#include "boost/geometry/index/rtree.hpp"

#include <igl/ray_mesh_intersect.h>
#include <igl/point_mesh_squared_distance.h>


namespace Slic3r {
namespace sla {

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

double ray_mesh_intersect(const Vec3d& s,
                          const Vec3d& dir,
                          const EigenMesh3D& m)
{
    igl::Hit hit;
    hit.t = std::numeric_limits<float>::infinity();
    igl::ray_mesh_intersect(s, dir, m.V, m.F, hit);
    return hit.t;
}

// Clustering a set of points by the given criteria
ClusteredPoints cluster(
        const sla::PointSet& points,
        std::function<bool(const SpatElement&, const SpatElement&)> pred,
        unsigned max_points = 0)
{

    namespace bgi = boost::geometry::index;
    using SpatIndex = bgi::rtree< SpatElement, bgi::rstar<16, 4> /* ? */ >;

    // A spatial index for querying the nearest points
    SpatIndex sindex;

    // Build the index
    for(unsigned idx = 0; idx < points.rows(); idx++)
        sindex.insert( std::make_pair(points.row(idx), idx));

    using Elems = std::vector<SpatElement>;

    // Recursive function for visiting all the points in a given distance to
    // each other
    std::function<void(Elems&, Elems&)> group =
    [&sindex, &group, pred, max_points](Elems& pts, Elems& cluster)
    {
        for(auto& p : pts) {
            std::vector<SpatElement> tmp;

            sindex.query(
                bgi::satisfies([p, pred](const SpatElement& se) {
                    return pred(p, se);
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

}
}
