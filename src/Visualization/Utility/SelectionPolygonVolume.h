// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2015 Qianyi Zhou <Qianyi.Zhou@gmail.com>
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

#pragma once

#include <vector>
#include <string>
#include <Eigen/Core>
#include <IO/ClassIO/IJsonConvertible.h>

namespace three {

class Geometry;
class PointCloud;

class SelectionPolygonVolume : public IJsonConvertible
{
public:
	~SelectionPolygonVolume() override {}
	
public:
	bool ConvertToJsonValue(Json::Value &value) const override;
	bool ConvertFromJsonValue(const Json::Value &value) override;
	void CropGeometry(const Geometry &input, Geometry &output) const;

private:
	void CropPointCloudInPolygon(const PointCloud &input,
			PointCloud &output) const;
	void CropInPolygon(const std::vector<Eigen::Vector3d> &input,
			std::vector<size_t> &output_index) const;

public:
	std::string orthogonal_axis_ = "";
	std::vector<Eigen::Vector3d> bounding_polygon_;
	double axis_min_ = 0.0;
	double axis_max_ = 0.0;
};

}	// namespace three
