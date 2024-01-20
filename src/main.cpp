// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "open3d/Open3D.h"
#include "curvature_computation/TotalCurvaturePointCloud.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;


Eigen::VectorXd curv(const RowMatrixXd& V_PCD, const RowMatrixXd& N_PCD)
{
    Eigen::VectorXd k_S_PCD(V_PCD.rows());

    // calculate total curvature on point cloud
    open3d::geometry::TotalCurvaturePointCloud::TotalCurvaturePCD(V_PCD, N_PCD, k_S_PCD, 20);

    return k_S_PCD;
}


Eigen::VectorXd curvFile(std::string filename)
{
    // Create an empty PointCloud object
    auto point_cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloud(filename, *point_cloud_ptr);

    // Get the vertex matrix V from the point cloud
    std::vector<Eigen::Vector3d> points_v = point_cloud_ptr->points_;
    std::vector<Eigen::Vector3d> points_n = point_cloud_ptr->normals_;
    RowMatrixXd V_PCD(points_v.size(), 3);
    RowMatrixXd N_PCD(points_v.size(), 3);
    for (size_t i = 0; i < points_v.size(); ++i)
    {
        V_PCD.row(i) = points_v[i];
        N_PCD.row(i) = points_n[i];
    }

    return curv(V_PCD, N_PCD);
}


std::vector<int> curvFilter(const RowMatrixXd& V_PCD, const RowMatrixXd& N_PCD, double threshold)
{
    Eigen::VectorXd k_S_PCD(V_PCD.rows());

    // calculate total curvature on point cloud
    open3d::geometry::TotalCurvaturePointCloud::TotalCurvaturePCD(V_PCD, N_PCD, k_S_PCD, 20);

    Eigen::VectorXd k_S_PCD_vis = k_S_PCD.array().abs().pow(0.0425);
    double min_val_pcd = k_S_PCD_vis.minCoeff();
    double max_val_pcd = k_S_PCD_vis.maxCoeff();
    double interval = max_val_pcd - min_val_pcd;
    double thre = threshold * interval + min_val_pcd;

    std::vector<int> idxs;
    for (int i = 0; i < V_PCD.rows(); ++i)
    {
        if (k_S_PCD_vis(i) < thre)
        {
            idxs.push_back(i);
        }
    }
    return idxs;
}


PYBIND11_MODULE(curvature_computation, m) {
    m.def("curv", &curv, "Get the curvature of a point cloud. (Input position and normal matrices)",
          pybind11::return_value_policy::reference_internal);
    m.def("curvFile", &curvFile, "Get the curvature of a point cloud (input FilePath).",
          pybind11::return_value_policy::reference_internal);
    m.def("curvFilter", &curvFilter, "Filter cloud based on curvature (input also threshold for filtering).",
          pybind11::return_value_policy::reference_internal);
}