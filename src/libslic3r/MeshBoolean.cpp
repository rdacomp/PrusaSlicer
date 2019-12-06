#include "MeshBoolean.hpp"

#include "TriangleMesh.hpp"

#include <igl/readOFF.h>
//#undef IGL_STATIC_LIBRARY
#include <igl/copyleft/cgal/mesh_boolean.h>

#include <Eigen/Core>
#include <iostream>

#include <admesh/stl.h>

#include <boost/nowide/cstdio.hpp>

namespace Slic3r {

bool its_write_obj(const Eigen::MatrixXd &V, Eigen::MatrixXi &F, const char *file)
{

  	FILE *fp = boost::nowide::fopen(file, "w");
  	if (fp == nullptr) {
		BOOST_LOG_TRIVIAL(error) << "stl_write_obj: Couldn't open " << file << " for writing";
    	return false;
  	}

	for (size_t i = 0; i < V.rows(); ++ i)
    	fprintf(fp, "v %lf %lf %lf\n", V(i, 0), V(i, 1), V(i, 2));
  	for (size_t i = 0; i < F.rows(); ++ i)
    	fprintf(fp, "f %d %d %d\n", F(i, 0) + 1, F(i, 1) + 1, F(i, 2) + 1);
  	fclose(fp);
  	return true;
}

void mesh_boolean_test()
{
  using namespace Eigen;
  using namespace std;
//  igl::readOFF(TUTORIAL_SHARED_PATH "/cheburashka.off",VA,FA);
//  igl::readOFF(TUTORIAL_SHARED_PATH "/decimated-knight.off",VB,FB);
  // Plot the mesh with pseudocolors
//  igl::opengl::glfw::Viewer viewer;

  // Initialize
//  update(viewer);

  //igl::copyleft::cgal::mesh_boolean(VA,FA,VB,FB,boolean_type,VC,FC,J);


	TriangleMesh mesh;
    std::string fname = "D:\\3dprint\\treefrog_45_cut.stl";
	mesh.ReadSTLFile(fname.c_str());
	mesh.repair(true);
    its_write_obj(mesh.its, (fname + "-imported0.obj").c_str());

	Eigen::MatrixXd VA,VB,VC;
	Eigen::VectorXi J,I;
	Eigen::MatrixXi FA,FB,FC;
	igl::MeshBooleanType boolean_type(igl::MESH_BOOLEAN_TYPE_UNION);


  	typedef Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor | Eigen::DontAlign>> MapMatrixXfUnaligned;
    typedef Eigen::Map<const Eigen::Matrix<int,   Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor | Eigen::DontAlign>> MapMatrixXiUnaligned;

	Eigen::MatrixXd V = MapMatrixXfUnaligned(mesh.its.vertices.front().data(), mesh.its.vertices.size(), 3).cast<double>();
    Eigen::MatrixXi F = MapMatrixXiUnaligned(mesh.its.indices.front().data(), mesh.its.indices.size(), 3);

    its_write_obj(V, F, (fname + "-imported.obj").c_str());

    // Self-union to clean up 
    igl::copyleft::cgal::mesh_boolean(V, F, Eigen::MatrixXd(), Eigen::MatrixXi(), boolean_type, VC, FC);

    its_write_obj(VC, FC, (fname + "-fixed.obj").c_str());
}

} // namespace Slic3r
