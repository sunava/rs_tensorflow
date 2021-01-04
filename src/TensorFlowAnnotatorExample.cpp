#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <robosherlock/types/all_types.h>
//RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include "../include/cppflow/include/cppflow/cppflow.h"

#include <ros/package.h>


using namespace uima;


class TenserFlowAnnotator : public Annotator
{
private:
    std::string modelName;

public:
    std::string rosPath;
    std::string fullPath;
    std::string picturePath;
    TyErrorId initialize(AnnotatorContext &ctx)
    {
        outInfo("initialize");
        //example if you use your own model
       /* rosPath = ros::package::getPath("rs_tensorflow");
        fullPath = rosPath + "/data/EASE_R02_1obj_test" ;
        picturePath = rosPath + "/data/pictures/";*/

       //just for the cppflow example purpose
        rosPath = ros::package::getPath("rs_tensorflow");
        fullPath = rosPath + "/include/cppflow/examples/load_model/model" ;
        picturePath = rosPath + "/data/pictures/";
        return UIMA_ERR_NONE;
    }

    TyErrorId destroy()
    {
        outInfo("destroy");
        return UIMA_ERR_NONE;
    }

    TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
    {
        outInfo("process start");
        rs::StopWatch clock;

        /*in the follow the code is from https://github.com/serizba/cppflow/tree/master/examples/load_model to
        show a working example*/

        auto input = cppflow::fill({10, 5}, 1.0f);
        cppflow::model model(fullPath);
        auto output = model(input);

        outInfo("" << output);

        auto values = output.get_data<float>();

        for (auto v : values) {
            outInfo("" << v);
        }


        return UIMA_ERR_NONE;
    }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(TenserFlowAnnotator)