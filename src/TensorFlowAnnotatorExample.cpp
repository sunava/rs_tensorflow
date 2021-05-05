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
        rosPath = ros::package::getPath("rs_tensorflow");
        fullPath = rosPath + "/include/pose_head_test_networks/test_inference_csl_5" ;
        picturePath = rosPath + "/include/pose_head_test_networks/";
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
        cppflow::tensor cam_k {1075.65091572, 0.0, 376.06888344, 0.0, 1073.90347929, 260.72159802, 0.0, 0.0, 1.0};
        cppflow::tensor cam_k_shape {1,3,3};
        cam_k = cppflow::reshape(cppflow::cast(cam_k, TF_DOUBLE, TF_FLOAT), cam_k_shape);

        cppflow::tensor coord_K {1.1188747655293558, 1.1188747655293558, 385.97404256353514, 336.62304132232975};
        cppflow::tensor coord_k_shape {1,2,2};
        coord_K = cppflow::reshape(cppflow::cast(coord_K, TF_DOUBLE, TF_FLOAT), coord_k_shape);



        std::string img_paths[] { picturePath + "rgb_5_1.png", picturePath + "rgb_5_2.png"};
        std::string img_paths_depth[] { picturePath + "depth_5_1.png",picturePath + "depth_5_2.png"};


        cppflow::model model(fullPath);




        int i = 0;
        for (auto img : img_paths) {
            //we need to define channels here otherwise cppflow takes alpha channel into the decode
            auto input = cppflow::decode_png(cppflow::read_file(std::string(img_paths[i])), 3);
            input = cppflow::cast(input, TF_UINT8, TF_FLOAT);
            input = cppflow::expand_dims(input, 0);



            //it was importantto change to TF_UINT16 here
            auto input_d = cppflow::decode_png(cppflow::read_file(std::string(img_paths_depth[i])),1, TF_UINT16);
            input_d = cppflow::cast(input_d,TF_UINT16,  TF_FLOAT);
            input_d = cppflow::expand_dims(input_d, 0);

            //need to squeeze the input tensor is not correct
            cppflow::tensor deleteDim {3};
            input_d = cppflow::squeeze(input_d, std::vector<long int> {3});
            input_d = input_d / float(10.0);



            auto output = model({{"serving_default_input_36:0", input},{"serving_default_input_37:0", input_d},{"serving_default_input_39:0", cam_k},{"serving_default_input_40:0", coord_K}}, {"StatefulPartitionedCall:0", "StatefulPartitionedCall:1"});

            //to control if the outputs are correct you can check the info_X.txt R = rgb output the last numbers in each vector
            outInfo("output RGB: " << output[1]);

            i++;
        }


        return UIMA_ERR_NONE;
    }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(TenserFlowAnnotator)

