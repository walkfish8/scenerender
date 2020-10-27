/***********************************************************************
* Software License Agreement (Ruler License)
*
* Copyright 2008-2011  Li YunQiang (liyunqiang@91ruler.com). All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
* THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/
#include "trimesh.h"

#include <fstream>
#include <sstream>
#include <iostream>

static const unsigned OBJ_INDEX_OFFSET = 1;
static const unsigned NO_ID = std::numeric_limits<unsigned>::max();

Ruler::TriMesh::TriMesh() {}
Ruler::TriMesh::~TriMesh() {}

void Ruler::TriMesh::clear()
{
    vertices.clear();
    normals.clear();
    texcoords.clear();
    faces.clear();
    vertex_faces_.clear();
}

void Ruler::TriMesh::listIncidenteFaces()
{
    vertex_faces_.clear();
    vertex_faces_.resize(vertices.size());
    for (int i = 0; i < faces.size(); i++)
        for (int j = 0; j < 3; j++)
            vertex_faces_[faces[i].vertices[j]].push_back(i);
}

void Ruler::TriMesh::rotate_axis()
{
	for (int i = 0; i < vertices.size(); ++i)
	{
		cv::Point3f& point = vertices[i];
		point = cv::Point3f{ point.x, -point.z, point.y };
	}

	for (int i = 0; i < normals.size(); ++i)
	{
		cv::Point3f& normal = normals[i];
		normal = cv::Point3f{ normal.x, -normal.z, normal.y };
	}
}

void Ruler::TriMesh::transfrom(const CameraD& param)
{
	float a0 = param.m[0][0]; float a1 = param.m[0][1]; float a2 = param.m[0][2];
	float b0 = param.m[1][0]; float b1 = param.m[1][1]; float b2 = param.m[1][2];
	float c0 = param.m[2][0]; float c1 = param.m[2][1]; float c2 = param.m[2][2];
	float tx = param.t[0]; float ty = param.t[1]; float tz = param.t[2];

	float x, y, z;
	for (int i = 0; i < vertices.size(); ++i)
	{
		cv::Point3f& point = vertices[i];
		x = a0*point.x + a1*point.y + a2*point.z + tx;
		y = b0*point.x + b1*point.y + b2*point.z + ty;
		z = c0*point.x + c1*point.y + c2*point.z + tz;

		point.x = x;
		point.y = y;
		point.z = z;
	}

	for (int i = 0; i < normals.size(); ++i)
	{
		cv::Point3f& normal = normals[i];
		x = a0*normal.x + a1*normal.y + a2*normal.z;
		y = b0*normal.x + b1*normal.y + b2*normal.z;
		z = c0*normal.x + c1*normal.y + c2*normal.z;

		normal.x = x;
		normal.y = y;
		normal.z = z;
	}
}

void Ruler::TriMesh::getAdjacencyFaces(Index face_index,
                                       std::vector<Index>& adjacency_faces)
{
    if (vertex_faces_.empty()) listIncidenteFaces();

    adjacency_faces.clear();
    const auto& faces0 = vertex_faces_[faces[face_index].vertices[0]];
    const auto& faces1 = vertex_faces_[faces[face_index].vertices[1]];
    const auto& faces2 = vertex_faces_[faces[face_index].vertices[2]];

    adjacency_faces.assign(faces0.begin(), faces0.end());
    adjacency_faces.insert(adjacency_faces.end(), faces1.begin(), faces1.end());
    adjacency_faces.insert(adjacency_faces.end(), faces2.begin(), faces2.end());
    std::sort(adjacency_faces.begin(), adjacency_faces.end());
    adjacency_faces.erase(unique(adjacency_faces.begin(), adjacency_faces.end()), adjacency_faces.end());
}

bool Ruler::TriMesh::loadOBJ(const std::string& obj_path, bool is_rotate_axis, float scale)
{
    this->clear();
    std::ifstream fs(obj_path.c_str(), std::ios::in);
    std::istringstream in;
    std::string line, keyword;

    std::string material_name;
    float x(0), y(0), z(0);
    while (std::getline(fs, line, '\n'))
    {
        if (line.empty() || line[0] == '#')
            continue;

        in.clear();
        in.str(line);
        in >> keyword;

        if (keyword == "v")
        {
            in >> x >> y >> z;
            //vertices.push_back(is_rotate_axis ? cv::Point3f(x, -z, y) : cv::Point3f(x, y, z));
            vertices.push_back(is_rotate_axis ? cv::Point3f(x * scale, -z * scale, y*scale) : cv::Point3f(x * scale, y * scale, z * scale));
        }
        else if (keyword == "vt")
        {
            in >> x >> y;
            texcoords.push_back(cv::Point2f(x,y));
        }
        else if (keyword == "vn")
        {
            in >> x >> y >> z;
            normals.push_back(is_rotate_axis ? cv::Point3f(x, -z, y) : cv::Point3f(x, y, z));
        }
        else if (keyword == "f")
        {
            Face f;
            memset(&f, 0xFF, sizeof(Face));
            for (auto k = 0; k < 3; ++k)
            {
                in >> keyword;
                switch (sscanf_s(keyword.c_str(), "%u/%u/%u", f.vertices + k, f.texcoords + k, f.normals + k))
                {
                case 1:
                    f.vertices[k] -= OBJ_INDEX_OFFSET;
                    break;
                case 2:
                    f.vertices[k] -= OBJ_INDEX_OFFSET;
                    f.texcoords[k] -= f.texcoords[k] != std::numeric_limits<Index>::max() ? OBJ_INDEX_OFFSET : 0;
                    f.normals[k] -= f.normals[k] != std::numeric_limits<Index>::max() ? OBJ_INDEX_OFFSET : 0;
                    break;
                case 3:
                    f.vertices[k] -= OBJ_INDEX_OFFSET;
                    f.texcoords[k] -= OBJ_INDEX_OFFSET;
                    f.normals[k] -= OBJ_INDEX_OFFSET;
                    break;
                default:
                    break;
                }
            }
            faces.push_back(f);
        }
        else if (keyword == "mtllib")
        {
            in >> material_name;
        }
    }
    fs.close();

    return !vertices.empty();
}

bool Ruler::TriMesh::loadTexture(const std::string& tex_path)
{
    //teximage = cv::imread(tex_path, cv::IMREAD_UNCHANGED);
	teximage = readImageOnlyOnes(tex_path);
    return !teximage.empty();
}

bool Ruler::TriMesh::saveOBJ(const std::string& obj_path)
{
    std::ofstream ofs(obj_path, std::ios::out);
    std::string mtl_path = obj_path.substr(0, obj_path.length() - 4) + ".mtl";
    ofs << "mtllib " << mtl_path << std::endl;

    std::for_each(vertices.begin(), vertices.end(), [&](const cv::Point3f& pt)
    {
        ofs << "v " << pt.x << " " << pt.y << " " << pt.z << std::endl;
    });

    std::for_each(normals.begin(), normals.end(), [&](const cv::Point3f& pt)
    {
        ofs << "vn " << pt.x << " " << pt.y << " " << pt.z << std::endl;
    });

    ofs << "usemtl material_0" << std::endl;
    std::for_each(texcoords.begin(), texcoords.end(), [&](const cv::Point2f& pt)
    {
        ofs << "vt " << pt.x << " " << pt.y << std::endl;
    });

    for (int i = 0; i < faces.size(); i++)
    {
        ofs << "f";
        for (int j = 0; j < 3; j++)
        {
            ofs << " " << faces[i].vertices[j] + OBJ_INDEX_OFFSET;
            ofs << "/" << faces[i].texcoords[j] + OBJ_INDEX_OFFSET;
        }
        ofs << std::endl;
    }
    ofs.close();

    std::string texture_path = obj_path.substr(0, obj_path.length() - 4) + ".jpg";
    if (!teximage.empty()) cv::imwrite(texture_path, teximage);

    std::ofstream ofs2(mtl_path, std::ios::out);
    ofs2 << "newmtl material_0" << std::endl
        << "Ka 1 1 1" << std::endl
        << "Kd 1 1 1" << std::endl
        << "Ks 1 1 1" << std::endl
        << "Ns 1000" << std::endl
        << "map_Kd " << texture_path << std::endl;
    ofs2.close();
    return true;
}

static bool g_read_image_flag = true;
static std::map<std::string, cv::Mat> g_image_datas;

void Ruler::clearImageOnlyOnes()
{
	g_image_datas.clear();
}

void Ruler::setReadImageOnlyOnes(bool _on)
{
	g_read_image_flag = _on;
}

cv::Mat Ruler::readImageOnlyOnes(const std::string & _file_name)
{
	if (g_read_image_flag)
	{
		if (g_image_datas.find(_file_name) == g_image_datas.end())
		{
			cv::Mat image = cv::imread(_file_name, cv::IMREAD_UNCHANGED);
			g_image_datas.insert(std::make_pair(_file_name, image));
		}
		return g_image_datas.find(_file_name)->second;
	}
	return cv::imread(_file_name, cv::IMREAD_UNCHANGED);
}
