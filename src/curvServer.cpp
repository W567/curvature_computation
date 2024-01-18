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

int main(int argc, char **argv) {

    std::string filename_v = argv[1];
    // Create an empty PointCloud object
    auto point_cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloud(filename_v, *point_cloud_ptr);

    // Get the vertex matrix V from the point cloud
    std::vector<Eigen::Vector3d> points_v = point_cloud_ptr->points_;
    std::vector<Eigen::Vector3d> points_n = point_cloud_ptr->normals_;
    Eigen::MatrixXd V_PCD(points_v.size(), 3);
    Eigen::MatrixXd N_PCD(points_v.size(), 3);
    for (size_t i = 0; i < points_v.size(); ++i) {
        V_PCD.row(i) = points_v[i];
        N_PCD.row(i) = points_n[i];
    }

    Eigen::VectorXd k_S_PCD(V_PCD.rows());

    // calculate total curvature on point cloud
    open3d::geometry::TotalCurvaturePointCloud::TotalCurvaturePCD(V_PCD, N_PCD, k_S_PCD, 20);

    return 0;
}