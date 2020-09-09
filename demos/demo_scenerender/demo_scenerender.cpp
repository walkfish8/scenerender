#include "scenerender.h"

void main()
{
    float scale = 1000;

    Ruler::CameraD cameraparam;
    cameraparam.SetQuaternionRotation(0.0, 0.0, 0.6994630464255629, 0.7146687671117795);
    cameraparam.SetTranslation(3.40076238 * scale, -8.40359721 * scale, 1.4 * scale);

    Ruler::CameraD rectparam;
    rectparam.SetQuaternionRotation(7.056712382872147e-8, 0.7071067905528803, -7.056712569818394e-8, 0.7071067718202149);
    rectparam.SetTranslation(-6.958500385284424*scale, 1.2746762037277222*scale, 4.447582721710205*scale);

    Ruler::SceneRender sr(cameraparam, 2048, 6000, 3000);
    //sr.renderPanorama("..\\..\\..\\datas\\scene01\\全景图.jpg");
    //sr.renderTrimesh("..\\..\\..\\datas\\scene01\\obj\\012.obj", "", 0, true);
    sr.renderTrimesh("..\\..\\..\\datas\\scene01\\obj\\012.obj", "..\\..\\..\\datas\\scene01\\obj\\12.jpg", 0, true);
    sr.renderRectangle("..\\..\\..\\datas\\scene01\\拖放进去的图.png", rectparam, 8.18169057590558*scale / 2, 4.686696682403695*scale / 2, 1, true);

    sr.savePanoSimulateImage("..\\..\\..\\datas\\scene01\\out\\panoimage.bmp");
    sr.savePanoDepthImage("..\\..\\..\\datas\\scene01\\out\\panodepth.png");
    sr.saveSixBoxSimulateImage("..\\..\\..\\datas\\scene01\\out\\siximage.bmp");

    //sr.showPanoSimulateWithOpenGL();
}