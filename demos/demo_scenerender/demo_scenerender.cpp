#include "scenerender.h"

void main()
{
    float scale = 1000;
    //float scale = 1.0f;

    Ruler::CameraD cameraparam;
    //cameraparam.SetQuaternionRotation(0.0, 0.0, 0.6994630464255629, 0.7146687671117795);
    //cameraparam.SetTranslation(3.40076238 * scale, -8.40359721 * scale, 1.4 * scale);

    //Ruler::CameraD rectparam;
    //rectparam.SetQuaternionRotation(7.056712382872147e-8, 0.7071067905528803, -7.056712569818394e-8, 0.7071067718202149);
    //rectparam.SetTranslation(-6.958500385284424*scale, 1.2746762037277222*scale, 4.447582721710205*scale);

    //Ruler::SceneRender sr(cameraparam, 2048, 6000, 3000);
    //sr.renderPanorama("..\\..\\..\\datas\\scene01\\全景图.jpg");
    //sr.renderTrimesh("..\\..\\..\\datas\\scene01\\obj\\012.obj", "", 0, true);
    ////sr.renderTrimesh("..\\..\\..\\datas\\scene01\\obj\\012.obj", "..\\..\\..\\datas\\scene01\\obj\\12.jpg", 0, true);
    //sr.renderRectangle("..\\..\\..\\datas\\scene01\\拖放进去的图.png", rectparam, 8.18169057590558*scale / 2, 4.686696682403695*scale / 2, 1, true);

    cameraparam.SetQuaternionRotation(0.0, 0.0, 0.7071067811865475, 0.7071067811865476);
    cameraparam.SetTranslation(19.2855 * scale, -7.28958 * scale, 1.4 * scale);

    Ruler::CameraD rectparam;
    rectparam.SetQuaternionRotation(-0.00000030064244437350754, 0.30029866609612915, 0.00000009465112657261254, 0.9538452238915841);
    rectparam.SetTranslation(17.643327102396*scale, 2.017847887922609*scale, 2.373871050770903*scale);

    const char* sixpath[6] = {
        "..\\..\\..\\datas\\scene02\\六面体_0.jpg",
        "..\\..\\..\\datas\\scene02\\六面体_1.jpg",
        "..\\..\\..\\datas\\scene02\\六面体_2.jpg",
        "..\\..\\..\\datas\\scene02\\六面体_3.jpg",
        "..\\..\\..\\datas\\scene02\\六面体_4.jpg",
        "..\\..\\..\\datas\\scene02\\六面体_5.jpg" };

    Ruler::SceneRender sr(cameraparam, 4096, 9000, 4500);
    sr.clearDepthAndImage();
    sr.renderSixBox(sixpath);
    //sr.renderSixBox("..\\..\\..\\datas\\scene02\\六面体.jpg");
    //sr.renderPanorama("..\\..\\..\\datas\\scene02\\全景图.jpg");
    sr.renderTrimesh("..\\..\\..\\datas\\scene02\\obj\\500-1.obj", "", 0, true);
    //sr.renderTrimesh("..\\..\\..\\datas\\scene02\\obj\\500-1.obj", "..\\..\\..\\datas\\scene02\\obj\\12.jpg", 0, true);
    sr.renderRectangle("..\\..\\..\\datas\\scene02\\拖进去的图片.png", rectparam, 6.684f*scale / 2, 3.742f*scale / 2, 1, true);

    sr.savePanoSimulateImage("..\\..\\..\\datas\\scene02\\out\\panoimage.jpg");
    sr.savePanoDepthImage("..\\..\\..\\datas\\scene02\\out\\panodepth.png");
    sr.saveSixBoxSimulateImage("..\\..\\..\\datas\\scene02\\out\\siximage.jpg");

 //   cameraparam.SetQuaternionRotation(0.0, 0.0, 0.7071067811865475, 0.7071067811865476);
 //   cameraparam.SetTranslation(-6.60114 * scale, -2.59308 * scale, 1.4 * scale);

 //   Ruler::CameraD rectparam;
 //   rectparam.SetQuaternionRotation(0.0, 0.7071067811865475, 0.0, 0.7071067811865476);
 //   rectparam.SetTranslation(-9.48090934753418 * scale, 2.092496385574341 * scale, 2.286712884902954 * scale);

	//const char* sixpath[6] = {
	//	"..\\..\\..\\datas\\high\\00c95159423b30e1ae5c188adbe2499b_skybox1.jpg",
	//	"..\\..\\..\\datas\\high\\00c95159423b30e1ae5c188adbe2499b_skybox2.jpg",
	//	"..\\..\\..\\datas\\high\\00c95159423b30e1ae5c188adbe2499b_skybox3.jpg",
	//	"..\\..\\..\\datas\\high\\00c95159423b30e1ae5c188adbe2499b_skybox4.jpg",
	//	"..\\..\\..\\datas\\high\\00c95159423b30e1ae5c188adbe2499b_skybox0.jpg",
	//	"..\\..\\..\\datas\\high\\00c95159423b30e1ae5c188adbe2499b_skybox5.jpg" };

	//Ruler::SceneRender sr(cameraparam, 1024, 9000, 4500);
	//sr.renderSixBox(sixpath);
	////sr.renderSixBox("..\\..\\..\\datas\\scene02\\六面体.jpg");
 //   //sr.renderPanorama("..\\..\\..\\datas\\scene02\\全景图.jpg");
	//sr.renderTrimesh("..\\..\\..\\datas\\high\\0b3c3fd5-c759-e6a6-7e3b-5af6b2ea2513\\Material__2339.obj", "", 0, true, 1000.0f);
	////sr.renderTrimesh("..\\..\\..\\datas\\scene02\\obj\\500-1.obj", "..\\..\\..\\datas\\scene02\\obj\\12.jpg", 0, true);
	////sr.renderRectangle("..\\..\\..\\datas\\scene02\\拖进去的图片.png", rectparam, 6.684f*scale / 2, 3.742f*scale / 2, 1, true);
 //   sr.renderRectangle("..\\..\\..\\datas\\scene02\\拖进去的图片.png", rectparam, 3.2386f * scale / 2, 2.3910f * scale / 2, 1, true);

 //   sr.savePanoSimulateImage("..\\..\\..\\datas\\high\\out\\panoimage.jpg");
 //   sr.savePanoDepthImage("..\\..\\..\\datas\\high\\out\\panodepth.png");
 //   sr.saveSixBoxSimulateImage("..\\..\\..\\datas\\high\\out\\siximage.jpg");
 //   sr.saveSixBoxDepthImage("..\\..\\..\\datas\\high\\out\\sixdepth.png");

    sr.showPanoSimulateWithOpenGL();
}