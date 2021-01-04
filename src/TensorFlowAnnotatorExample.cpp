#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <robosherlock/types/all_types.h>
//RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>
#include "../include/cppflow/include/cppflow.h"

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
        rosPath = ros::package::getPath("rs_tensorflow");
        fullPath = rosPath + "/data/EASE_R02_1obj_test" ;
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
        rs::SceneCas cas(tcas);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
        cas.get(VIEW_CLOUD,*cloud_ptr);

        std::string img_paths[] { picturePath + "000335-color.jpg", picturePath + "000409-color.jpg", picturePath + "000437-color.jpg", picturePath + "000467-color.jpg", picturePath + "000568-color.jpg"};

        cppflow::model model(fullPath);


        cppflow::tensor cm {1066.778, 0.0, 312.9869, 0.0, 1067.487, 241.3109, 0.0, 0.0, 1.0};
        cppflow::tensor cm_shape {1,3,3};

        cm = cppflow::reshape(cppflow::cast(cm, TF_DOUBLE, TF_FLOAT), cm_shape);

        outInfo("Input cameramatrix: " << std::endl << cm);

        for (auto img_path : img_paths)
        {
            outInfo("" << img_path << ":");

            auto input = cppflow::decode_jpeg(cppflow::read_file(std::string(img_path)));
            input = cppflow::cast(input, TF_UINT8, TF_FLOAT);
            input = cppflow::expand_dims(input, 0);

            auto output = model({{"serving_default_camera_matrix_input:0", cm}, {"serving_default_input_2:0", input}}, {"StatefulPartitionedCall:0", "StatefulPartitionedCall:1"})[0]; // this ([0]) takes only the pose

            outInfo("output : " << output);
        }


        return UIMA_ERR_NONE;
    }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(TenserFlowAnnotator)