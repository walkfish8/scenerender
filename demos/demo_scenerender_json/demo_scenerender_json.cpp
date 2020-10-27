#include <iostream>
#include <fstream>
#include "json\json.h"
#include "timer.h"
#include "logger.h"
#include "scenerender.h"

#include <chrono>

#include <Windows.h>

std::vector<std::string> FindFilesInDirectory(const std::string& _dir,
	const std::string& _format);

std::string FileName(const std::string& _file_path)
{
	return _file_path.substr(_file_path.rfind('/') + 1, std::string::npos);
}

std::string RemoveExtension(const std::string& _file_path)
{
	return _file_path.substr(0, _file_path.rfind('.'));
}

void render_objs(Ruler::SceneRender& sr, const char* json_path, const char* obj_dir, float scale = 1.0f);
void render_images(Ruler::SceneRender& sr, const char* json_path, const char* picture_dir, float scale = 1.0f);

void main()
{
    auto start = std::chrono::system_clock::now();

	std::string out_dir = R"(..\..\..\datas\fc43ad5c-4a78-02f4-be90-3a334224b3f7\out)";
    std::string pano_dir = R"(..\..\..\datas\fc43ad5c-4a78-02f4-be90-3a334224b3f7\pano)";
    std::string picture_dir = R"(..\..\..\datas\fc43ad5c-4a78-02f4-be90-3a334224b3f7\pics)";
    std::string json_path = R"(..\..\..\datas\fc43ad5c-4a78-02f4-be90-3a334224b3f7\json\sweeps.json)";
    std::string scene_json_path = R"(..\..\..\datas\fc43ad5c-4a78-02f4-be90-3a334224b3f7\json\scene.json)";
	std::string extobj_json_path = R"(..\..\..\datas\fc43ad5c-4a78-02f4-be90-3a334224b3f7\json\ExtObjs.json)";

	std::string obj_dir = R"(..\..\..\datas\fc43ad5c-4a78-02f4-be90-3a334224b3f7\model)";
	auto obj_paths = FindFilesInDirectory(obj_dir, ".obj");
    std::string obj_path = R"(..\..\..\datas\fc43ad5c-4a78-02f4-be90-3a334224b3f7\model\75677dfc-a532-bc0c-cbb5-af7b1785dda0.obj)";

	std::cout << "obj_paths : " << obj_paths.size() << std::endl;
	std::cout << "obj_paths : " << obj_paths[0] << std::endl;

    std::ifstream ifs;
    ifs.open(json_path);
    if (!ifs.is_open())
    {
        std::cout << "Reading json file failed!" << std::endl;
        return;
    }

    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(ifs, root, false))
    {
        return;
    }

    //float scale = 1000;
    float scale = 1.0f;

    char tmp[256];
    Ruler::CameraD cameraparam;
    Ruler::SceneRender sr(cameraparam, 2048, 9000, 4500, 1);
	for (int index = 0; index < root.size(); ++index)
    {
        cameraparam.SetQuaternionRotation(
            root[index]["rotation"]["x"].asDouble(),
            root[index]["rotation"]["y"].asDouble(), 
            root[index]["rotation"]["z"].asDouble(), 
            root[index]["rotation"]["w"].asDouble());
        cameraparam.SetTranslation(
            root[index]["position"]["x"].asDouble() * scale,
            root[index]["position"]["y"].asDouble() * scale,
            root[index]["position"]["z"].asDouble() * scale);

        sr.clearDepthAndImage();
        sr.setCameraParam(cameraparam);

        std::string sweep_uuid = root[index]["sweep_uuid"].asString();
        std::string pano_box1 = pano_dir + "\\" + sweep_uuid + "_skybox1.jpg";
        std::string pano_box2 = pano_dir + "\\" + sweep_uuid + "_skybox2.jpg";
        std::string pano_box3 = pano_dir + "\\" + sweep_uuid + "_skybox3.jpg";
        std::string pano_box4 = pano_dir + "\\" + sweep_uuid + "_skybox4.jpg";
        std::string pano_box0 = pano_dir + "\\" + sweep_uuid + "_skybox0.jpg";
        std::string pano_box5 = pano_dir + "\\" + sweep_uuid + "_skybox5.jpg";
        const char *sixpath[6] = {
            pano_box1.c_str(),
            pano_box2.c_str(),
            pano_box3.c_str(),
            pano_box4.c_str(),
            pano_box0.c_str(),
            pano_box5.c_str() };
        for (int i = 0; i < 6; ++i)
            std::cout << sixpath[i] << std::endl;
        sr.renderSixBox(sixpath);

		sr.renderTrimesh(obj_path.c_str(), "", 0, true);
		//for(int i=0;i <obj_paths.size(); ++i)
		//	sr.renderTrimesh(obj_paths[i].c_str(), "", 0, true);
		render_objs(sr, extobj_json_path.c_str(), obj_dir.c_str(), scale);
        render_images(sr, scene_json_path.c_str(), picture_dir.c_str(), scale);

        std::cout << root[index]["sweep_uuid"].asString() << std::endl;
        std::cout << root[index]["position"]["x"].asDouble() << std::endl;
        std::cout << root[index]["position"]["y"].asDouble() << std::endl;
        std::cout << root[index]["position"]["z"].asDouble() << std::endl;
        std::cout << root[index]["rotation"]["x"].asDouble() << std::endl;
        std::cout << root[index]["rotation"]["y"].asDouble() << std::endl;
        std::cout << root[index]["rotation"]["z"].asDouble() << std::endl;
        std::cout << root[index]["rotation"]["w"].asDouble() << std::endl;

        sprintf_s(tmp, "%s\\%02d.jpg", out_dir.c_str(), index);
        //sr.saveSixBoxSimulateImage(tmp);
		sr.savePanoSimulateImage(tmp);

        sprintf_s(tmp, "%s\\%02d.png", out_dir.c_str(), index);
        sr.savePanoDepthImage(tmp, 1000.0f);
    }

    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Spent" << double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << " seconds." << std::endl;
    return;
}

void render_objs(Ruler::SceneRender& sr, const char* json_path, const char* obj_dir, float scale)
{
	std::ifstream ifs;
	ifs.open(json_path);
	if (!ifs.is_open())
	{
		std::cout << "Reading json file failed!" << std::endl;
		return;
	}

	Json::Reader reader;
	Json::Value root;
	if (!reader.parse(ifs, root, false))
	{
		return;
	}

	Ruler::CameraD objparam;

	//std::cout << "scene " << root["data"]["extobjs"].size() << std::endl;
	for (int index = 0; index < root.size(); ++index)
	{
		if (root[index]["type"].asInt() == 4)
		{
			//std::string obj_path = root[index]["obj"].asString();
			//obj_path = std::string(obj_dir) + "\\" + RemoveExtension(FileName(obj_path)) + ".obj";
			std::string obj_path = std::string(obj_dir) + "\\" + root[index]["id"].asString() + ".obj";

			objparam.SetQuaternionRotation(
				root[index]["quaternion"]["x"].asDouble(),
				root[index]["quaternion"]["y"].asDouble(),
				root[index]["quaternion"]["z"].asDouble(),
				root[index]["quaternion"]["w"].asDouble());
			objparam.SetTranslation(
				root[index]["position"]["x"].asDouble()*scale,
				root[index]["position"]["y"].asDouble()*scale,
				root[index]["position"]["z"].asDouble()*scale);

			std::cout << obj_path << std::endl;
			sr.renderTrimesh(obj_path.c_str(), "", objparam, 0, true, root[index]["scale"]["x"].asFloat());
		}
		//std::cout << root[index]["type"].asInt() << std::endl;
		//std::cout << root[index]["position"]["x"].asDouble() << std::endl;
		//std::cout << root[index]["position"]["y"].asDouble() << std::endl;
		//std::cout << root[index]["position"]["z"].asDouble() << std::endl;
		//std::cout << root[index]["quaternion"]["x"].asDouble() << std::endl;
		//std::cout << root[index]["quaternion"]["y"].asDouble() << std::endl;
		//std::cout << root[index]["quaternion"]["z"].asDouble() << std::endl;
		//std::cout << root[index]["quaternion"]["w"].asDouble() << std::endl;



		////std::cout << root["data"]["extobjs"][index]["scale"]["x"].asDouble() << std::endl;
		////std::cout << root["data"]["extobjs"][index]["scale"]["y"].asDouble() << std::endl;
		////std::cout << root["data"]["extobjs"][index]["scale"]["z"].asDouble() << std::endl;

		//std::string image_path = std::string(picture_dir) + "\\" + root["data"]["extobjs"][index]["id"].asString() + ".jpg";
		//sr.renderRectangle(image_path.c_str(), rectparam,
		//	root["data"]["extobjs"][index]["scale"]["x"].asDouble()*scale / 2,
		//	root["data"]["extobjs"][index]["scale"]["y"].asDouble()*scale / 2, 1, true);
	}

	ifs.close();
}

void render_images(Ruler::SceneRender& sr, const char* json_path, const char* picture_dir, float scale)
{
    std::ifstream ifs;
    ifs.open(json_path);
    if (!ifs.is_open())
    {
        std::cout << "Reading json file failed!" << std::endl;
        return;
    }

    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(ifs, root, false))
    {
        return;
    }

    Ruler::CameraD rectparam;

    //std::cout << "scene " << root["data"]["extobjs"].size() << std::endl;
    for (int index = 0; index < root["data"]["extobjs"].size(); ++index)
    {
        //std::cout << root["data"]["extobjs"][index]["id"].asString() << std::endl;
        //std::cout << root["data"]["extobjs"][index]["position"]["x"].asDouble() << std::endl;
        //std::cout << root["data"]["extobjs"][index]["position"]["y"].asDouble() << std::endl;
        //std::cout << root["data"]["extobjs"][index]["position"]["z"].asDouble() << std::endl;
        //std::cout << root["data"]["extobjs"][index]["quaternion"]["x"].asDouble() << std::endl;
        //std::cout << root["data"]["extobjs"][index]["quaternion"]["y"].asDouble() << std::endl;
        //std::cout << root["data"]["extobjs"][index]["quaternion"]["z"].asDouble() << std::endl;
        //std::cout << root["data"]["extobjs"][index]["quaternion"]["w"].asDouble() << std::endl;

        rectparam.SetQuaternionRotation(
            root["data"]["extobjs"][index]["quaternion"]["x"].asDouble(),
            root["data"]["extobjs"][index]["quaternion"]["y"].asDouble(),
            root["data"]["extobjs"][index]["quaternion"]["z"].asDouble(),
            root["data"]["extobjs"][index]["quaternion"]["w"].asDouble());
        rectparam.SetTranslation(
            root["data"]["extobjs"][index]["position"]["x"].asDouble()*scale,
            root["data"]["extobjs"][index]["position"]["y"].asDouble()*scale,
            root["data"]["extobjs"][index]["position"]["z"].asDouble()*scale);

        //std::cout << root["data"]["extobjs"][index]["scale"]["x"].asDouble() << std::endl;
        //std::cout << root["data"]["extobjs"][index]["scale"]["y"].asDouble() << std::endl;
        //std::cout << root["data"]["extobjs"][index]["scale"]["z"].asDouble() << std::endl;

        std::string image_path = std::string(picture_dir) + "\\" + root["data"]["extobjs"][index]["id"].asString() + ".jpg";
        sr.renderRectangle(image_path.c_str(), rectparam,
            root["data"]["extobjs"][index]["scale"]["x"].asDouble()*scale / 2, 
            root["data"]["extobjs"][index]["scale"]["y"].asDouble()*scale / 2, 1, true);
    }

    ifs.close();
}

std::vector<std::string> FindFilesInDirectory(const std::string& _dir,
	const std::string& _format)
{
	std::string              name;
	WIN32_FIND_DATA          FindFileData;
	std::vector<std::string> names;
	std::string              dir_path =
		_dir.at(_dir.length() - 1) != '\\' ? _dir + "\\" : _dir;
	std::string findstr =
		dir_path + (!_format.empty() && _format[0] == '.' ? ("*" + _format)
			: ("*." + _format));

	HANDLE hFind = ::FindFirstFile(findstr.c_str(), &FindFileData);
	if (hFind == INVALID_HANDLE_VALUE) return std::move(names);

	do
	{
		if (!(FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
		{
			names.push_back(
				name.assign(dir_path).append(FindFileData.cFileName));
		}
	} while (::FindNextFile(hFind, &FindFileData));
	return std::move(names);
}