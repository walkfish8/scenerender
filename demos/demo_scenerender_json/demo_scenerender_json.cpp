#include <iostream>d2368e61-ecf2-2d8f-5fcf-bbcad19abfac
#include <fstream>
#include "json\json.h"
#include "timer.h"
#include "logger.h"
#include "scenerender.h"

#include <chrono>

void render_images(Ruler::SceneRender& sr, const char* json_path, const char* picture_dir, float scale = 1.0f);

void main()
{
    auto start = std::chrono::system_clock::now();

	std::string out_dir = R"(..\..\..\datas\d2368e61-ecf2-2d8f-5fcf-bbcad19abfac\out)";
    std::string pano_dir = R"(..\..\..\datas\d2368e61-ecf2-2d8f-5fcf-bbcad19abfac\pano)";
    std::string picture_dir = R"(..\..\..\datas\d2368e61-ecf2-2d8f-5fcf-bbcad19abfac\pics)";
    std::string json_path = R"(..\..\..\datas\d2368e61-ecf2-2d8f-5fcf-bbcad19abfac\json\sweeps.json)";
    std::string scene_json_path = R"(..\..\..\datas\d2368e61-ecf2-2d8f-5fcf-bbcad19abfac\json\scene.json)";
    std::string obj_path = R"(..\..\..\datas\d2368e61-ecf2-2d8f-5fcf-bbcad19abfac\model\21047246-c1bf-ee80-38c7-5854d64526af.obj)";

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
    Ruler::SceneRender sr(cameraparam, 2048, 9000, 4500, 2);
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
        sr.saveSixBoxSimulateImage(tmp);
		//sr.savePanoSimulateImage(tmp);

        //sprintf_s(tmp, "%s\\%02d.png", out_dir.c_str(), index);
        //sr.savePanoDepthImage(tmp, 1000.0f);
    }

    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Spent" << double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den << " seconds." << std::endl;
    return;
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