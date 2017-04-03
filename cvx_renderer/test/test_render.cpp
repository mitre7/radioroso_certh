#include <cvx/renderer/renderer.hpp>

using namespace cvx::renderer;
using namespace std ;
using namespace Eigen ;

int main(int argc, char *argv[])
{
 //   const aiScene *sc = aiImportFile("/home/malasiot/makehuman/v1/exports/man-hair.obj", aiProcessPreset_TargetRealtime_MaxQuality);
/*
    std::vector<Eigen::Matrix4d> views ;

    makeViewsOnSphere(Eigen::Vector3d(0, 0, 1.58),
                        0.3,
                        1.9, 2.1, 0.1,
                        -M_PI/6, -M_PI/6, M_PI/20,
//                        -M_PI/12, M_PI/12, M_PI/24,
                        400,
                        views) ;

     PinholeCamera cam(540.686, 540.686, 479.75, 269.75, cv::Size(960, 540)) ;
     //PinholeCamera cam(525, 525, 640/2, 480/2, cv::Size(640, 480))

    OffscreenRenderer rdr(sc, "/home/malasiot/makehuman/v1/exports/", cam, OffscreenRenderer::RenderSolid ) ;
    rdr.setBackgroundColor(0, 0, 1);

    uint ws = 48 ;

    //cv::Rect rc(320-ws, 240-ws, 2*ws, 2*ws) ;
    cv::Rect rc(960/2-ws, 540/2-ws, 2*ws, 2*ws) ;

    for(int i=0 ; i<views.size() ; i++)
    {
        rdr.render(views[i]) ;
        imwritef(rdr.getColor(false)(rc), "/home/malasiot/tmp/head_and_shoulder/rgb_%05d.png", i) ;
        imwritef(rdr.getDepth()(rc), "/home/malasiot/tmp/head_and_shoulder/depth_%05d.png", i) ;
    }
*/

    RenderingContextPtr ctx = getOffscreenContext(640, 480) ;

    cout << "ok" ;
}

