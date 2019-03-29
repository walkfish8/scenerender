#include "scenerender.h"


void main()
{
    //Ruler::TriMesh mesh;
    //mesh.loadOBJ("D:\\scenerender\\datas\\È«¾°²âÊÔ\\12\\012.obj", true);
    //mesh.loadTexture("D:\\scenerender\\datas\\È«¾°²âÊÔ\\12\\12.jpg");
    //mesh.saveOBJ("d:\\2.obj");
    //return;

    float scale = 1000;

    Ruler::CameraD cameraparam;
    cameraparam.SetQuaternionRotation(0.0, 0.0, 0.6994630464255629, 0.7146687671117795);
    cameraparam.SetPositionAfterRotation(3.40076238 * scale, -8.40359721 * scale, 1.4 * scale);

    Ruler::CameraD rectparam;
    rectparam.SetQuaternionRotation(7.056712382872147e-8, 0.7071067905528803, -7.056712569818394e-8, 0.7071067718202149);
    rectparam.SetPositionAfterRotation(-6.958500385284424*scale, 1.2746762037277222*scale, 4.447582721710205*scale);

    Ruler::SceneRender sr(cameraparam, 2048, 6000, 3000);
    sr.render(Ruler::RenderPanorama("D:\\scenerender\\datas\\È«¾°²âÊÔ\\È«¾°Í¼.jpg"));
    sr.render(Ruler::RenderTrimesh("D:\\scenerender\\datas\\È«¾°²âÊÔ\\12\\012.obj", "D:\\scenerender\\datas\\È«¾°²âÊÔ\\12\\12.jpg", true));
    sr.render(Ruler::RenderRectangle("D:\\scenerender\\datas\\È«¾°²âÊÔ\\ÍÏ·Å½øÈ¥µÄÍ¼.png", rectparam, 8.18169057590558*scale, 4.686696682403695*scale, true));

    cv::imwrite("d:\\pano.bmp", sr.getPanoSimulate());
    cv::imwrite("d:\\depth.png", sr.getPanoDepth());
}