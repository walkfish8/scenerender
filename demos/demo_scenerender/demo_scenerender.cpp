#include "scenerender.h"


void main()
{
    float scale = 1000;

    Ruler::CameraD cameraparam;
    cameraparam.SetQuaternionRotation(0.0, 0.0, 0.6994630464255629, 0.7146687671117795);
    cameraparam.SetPositionAfterRotation(3.40076238 * scale, -8.40359721 * scale, 1.4 * scale);

    Ruler::CameraD rectparam;
    rectparam.SetQuaternionRotation(7.056712382872147e-8, 0.7071067905528803, -7.056712569818394e-8, 0.7071067718202149);
    rectparam.SetPositionAfterRotation(-6.958500385284424*scale, 1.2746762037277222*scale, 4.447582721710205*scale);

    Ruler::SceneRender sr(cameraparam, 2048, 6000, 3000);
    sr.render(Ruler::RenderPanorama("..\\..\\..\\datas\\scene01\\全景图.jpg"));
    sr.render(Ruler::RenderTrimesh("..\\..\\..\\datas\\scene01\\obj\\012.obj", "..\\..\\..\\datas\\scene01\\obj\\12.jpg", true));
    sr.render(Ruler::RenderRectangle("..\\..\\..\\datas\\scene01\\拖放进去的图.png", rectparam, 8.18169057590558*scale, 4.686696682403695*scale, true));

    cv::imwrite("d:\\pano.bmp", sr.getPanoSimulate());
    cv::imwrite("d:\\depth.png", sr.getPanoDepth());
    //cv::imwrite("d:\\record.png", sr.getPanoRecord());
}