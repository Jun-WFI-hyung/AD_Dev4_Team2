#include "ros/lane_detection_ros.h"
#include "core/system.h"
#include "../3rd/popl/include/popl.hpp"
#include "core/tracking_module.h"
#include "data/frame.h"
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>
#include <fstream>


void save_csv(vector<shared_ptr<data::frame>> frames,string csv_path){
    ofstream outfile;
    outfile.open(csv_path, ios::out);
    outfile << "id,lposl,lposr, rposl, rposr" << endl;
    for (int j = 0; j < frames.size(); j++)
    {
        outfile << (frames[j]->id_)/30 <<","<< frames[j]->lposl_ <<"," << frames[j]->lposr_ <<"," << frames[j]->rposl_ <<"," << frames[j]->rposr_ <<"," << endl;
    }
    
    outfile.close();    
}

int main(int argc, char* argv[]) {

    popl::OptionParser op("Allowed options");
    auto config = op.add<popl::Value<std::string>>("c", "config", "setting config file path");
    auto csv_save = op.add<popl::Value<std::string>>("o", "cvs-save", "setting csv file path");
    
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!csv_save->is_set() || !config->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    
    const YAML::Node yaml_node_ = YAML::LoadFile(config->value());
    auto file = yaml_node_["File"];
    string file_type = file["type"].as<string>();

    auto lane_detection = std::make_shared<lane_detection::system>();
    vector<shared_ptr<data::frame>> frames;

    // By Video File
    if(file_type=="Video"){
        frames = lane_detection->tracking_->video_start(file["path"].as<std::string>());
    }
    // By Ros Bag File
    else if(file_type=="Bag"){
        ros::init(argc, argv, "lane_detection",ros::init_options::AnonymousName);
        auto lane_detection_ros = std::make_shared<lane_detection_ros::system>(lane_detection);
        ros::Rate rate(50);
        while(ros::ok()){
            ros::spin();
            rate.sleep();
        }
    }
    else{
        std::cerr << "invalid File Type" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    // save_csv(frames,csv_save->value());
    return 0;
}