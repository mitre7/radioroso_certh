#include <cvx/renderer/scene.hpp>
#include <cvx/renderer/renderer.hpp>
#include <cvx/renderer/viewpoint_sampler.hpp>
#include <cvx/util/geometry/util.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/program_options/options_description.hpp>

#include <fstream>

namespace fs = boost::filesystem ;
namespace po = boost::program_options ;

using namespace std ;
using namespace Eigen ;
using namespace cvx::renderer ;
using namespace cvx::util ;

int main(int argc, char *argv[])
{
    string model_path, out_folder, camera_info_file, render_mode = "phong";
    vector<float> radius_range, roll_range ;
    vector<float> altitude_range ;
    uint n_views = 500 ;
    bool z_up = false ;

    po::options_description desc;

    desc.add_options()
            ("help", "produce help")
            ("model", po::value<string>(&model_path)->value_name("model_path")->required(), "path to the 3D model")
            ("out", po::value<string>(&out_folder)->value_name("out_folder")->required(), "output folder where rendered images will be stored")
            ("camera", po::value<string>(&camera_info_file)->value_name("caminfo"), "camera info file") ;
            ("views", po::value< uint >(&n_views)->value_name("num_views"), "number of views")
          ;
    desc.add_options()
            ("radius", po::value<vector<float> >(&radius_range)->value_name("<radmin> <radmax> <radstep>")->multitoken(), "radius range")
            ("roll", po::value< vector<float> >(&roll_range)->value_name("<rolmin> <rolmax> <rollstep>")->multitoken(), "roll range (degrees)")
            ("altitude", po::value< vector<float> >(&altitude_range)->value_name("<amin> <amax>")->multitoken(), "altitude range (degrees)")
            ("z-up", "Z axis up")
            ("render", po::value< string >(&render_mode)->value_name("flat|phong|gouraud"), "rendering mode")
            ;

    po::variables_map vm;

    try {
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        po::notify(vm);

        if (vm.count("help") ) {
            cout << "Usage: render_model [options] <dir>\n";
            cout << desc << endl ;
            return 1;
        }
    }
    catch( po::error &e )
    {
        cerr << e.what() << endl ;
        cout << "Usage: render_model [options] <dir>\n";
        cerr << desc << endl ;
        return 0;
    }

    z_up = vm.count("z-up") ;

    ViewPointSampler sampler ;

    PinholeCamera cam(525, 525, 640/2.0 + 0.5, 480/2 + 0.5, cv::Size(640, 480)) ;

    if ( !camera_info_file.empty() )
    {
        if ( !cam.read(camera_info_file) )
        {
            cerr << "Cannot read camera parameters from file: " << camera_info_file << endl ;
            exit(1) ;
        }
    }

    if ( radius_range.size() == 3 )
        sampler.setRadius(radius_range[0], radius_range[1], radius_range[2]);

    if ( roll_range.size() == 3 )
        sampler.setRoll(roll_range[0] * M_PI/180.0, roll_range[1] * M_PI/180.0, roll_range[2] * M_PI/180.0);

    if ( altitude_range.size() == 2 )
        sampler.setAltitude(altitude_range[0] * M_PI/180.0, altitude_range[1] * M_PI/180.0);


    ScenePtr s ;

    try {
        s = Scene::loadAssimp(model_path) ;
    }
    catch ( runtime_error &e ) {
        cerr << e.what() << endl ;
        return 1 ;
    }


    vector<Matrix4f> views;
    sampler.generate(n_views, views);

    SceneRenderer rdr(s, getOffscreenContext(cam.sz().width, cam.sz().height)) ;
    rdr.setBackgroundColor(Vector4f(0, 0.5, 1, 1));

    SceneRenderer::RenderMode rmode ;

    if ( render_mode == "flat" )
        rmode = SceneRenderer::RENDER_FLAT ;
    else if ( render_mode == "gouraud" )
        rmode = SceneRenderer::RENDER_GOURAUD ;
    else if ( render_mode == "phong" )
        rmode = SceneRenderer::RENDER_SMOOTH ;

    fs::path op(out_folder) ;

    for( uint i=0 ; i<views.size() ; i++ ) {
        Matrix4f mat ;
        if ( z_up ) {
            Matrix4f sw ;
            sw << 1, 0, 0, 0,
                  0, 0, 1, 0,
                  0, -1, 0, 0,
                  0, 0, 0, 1 ;
            mat = views[i] * sw ;
        }
        else
            mat = views[i] ;

        PerspectiveCamera pcam(cam, mat) ;
        rdr.render(pcam, rmode) ;

        cv::imwrite( ( op / str(boost::format("rgb_%05d.png") % i)).native(), rdr.getColor()) ;
        cv::imwrite( ( op / str(boost::format("depth_%05d.png") % i)).native(), rdr.getDepth()) ;
        ofstream strm ( (op / str(boost::format("pose_%05d.txt") % i)).native().c_str()) ;
        strm << mat ;
    }

    ViewPointSampler::exportCameras( (op / "cameras.dae").native(), views, PerspectiveCamera(cam)) ;

}
