#include <cvx/renderer/scene.hpp>
#include <cvx/renderer/renderer.hpp>
#include <cvx/renderer/viewpoint_sampler.hpp>
#include <cvx/util/geometry/util.hpp>
#include <boost/format.hpp>
#include <fstream>

using namespace std ;
using namespace Eigen ;
using namespace cvx::renderer ;
using namespace cvx::util ;

int main(int argc, char *argv[])
{
    ofstream param1_file;
    ofstream param2_file;
    param1_file.open ("/tmp/spring_images/param1_file.txt");
    param2_file.open ("/tmp/spring_images/param2_file.txt");

    ViewPointSampler sampler ;
    vector<Matrix4f> views;
    sampler.setRadius(0.09, 0.11, 0.02);
    sampler.setCenter(Vector3f(0, 0, 0)) ;
    sampler.setRoll(0, 0, M_PI/2);
//    sampler.setRoll(-M_PI/15, M_PI/15, M_PI/45);
//    sampler.setAzimuth(0, M_PI/2);
    sampler.generate(2000, views);

 //   ScenePtr s ;
 //   s = Scene::load("/home/malasiot/Downloads/kuka-kr5-r850.zae") ;
  //  s = Scene::load("/home/malasiot/Downloads/human.dae") ;
//    s = Scene::load("/home/malasiot/Downloads/box.dae") ;

    ScenePtr s = Scene::loadAssimp("/home/echord/Mitre/blender_models/spring_toy_real_sizev6.obj") ;
    //ScenePtr s = Scene::load("/home/malasiot/tmp/table.dae") ;
    //ScenePtr s = Scene::loadAssimp("/home/echord/Mitre/blender_models/iron_closed_nemonic_spring.ply") ;

 //  s->addLight(LightPtr(new DirectionalLight(Vector3f(0, 0, -1), Vector3f(0.5, 0.5, 0.5)))) ;
   s->addLight(LightPtr(new DirectionalLight(Vector3f(0, 0, 1), Vector3f(0.5, 0.5, 0.5)))) ;
    SceneRenderer rdr(s, getOffscreenContext(640, 480)) ;
    rdr.setBackgroundColor(Vector4f(0, 0, 0, 1));

    //    PerspectiveCamera pcam(PinholeCamera(550, 550, 640/2.0, 480/2.0, cv::Size(640, 480)),
      //                         lookAt(Vector3f(0, 0.5, 4), Vector3f(0, 0.5, 0), Vector3f(0, 1, 0))) ;

    for( uint i=0 ; i<views.size() ; i++ ) {
        PerspectiveCamera pcam(PinholeCamera(550, 550, 640/2.0, 480/2.0, cv::Size(640, 480)), views[i]) ;
        rdr.render(pcam, SceneRenderer::RENDER_SMOOTH) ;
        //rdr.render(pcam, SceneRenderer::RENDER_FLAT) ;

        cv::imwrite(str(boost::format("/tmp/spring_images/spring/rgb_%05d.png") % i), rdr.getColor()) ;

        param1_file << sampler.param1[i] << std::endl;
        param2_file << sampler.param2[i] << std::endl;
    }

    param1_file.close();
    param2_file.close();

    ViewPointSampler::exportCameras("/tmp/cameras.dae", views, PerspectiveCamera(PinholeCamera(550, 550, 640/2.0, 480/2.0, cv::Size(640, 480)))) ;

  //  cv::imwrite("/tmp/oo.png", rdr.getColor()) ;

}
