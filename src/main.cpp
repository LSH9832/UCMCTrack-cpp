#include <opencv2/opencv.hpp>
#include "argparse/argparser.h"

#include "image_utils/tracker.h"
#include "print_utils.h"
#include "yaml-cpp/yaml.h"

using namespace std;


argsutil::argparser get_args(int argc, char* argv[]) {
    auto parser = argsutil::argparser("UCMCTrack argument parser.");

    parser.add_argument<string>("result_file", "result file path")
          .add_option<string>("-f", "--file", "track config file", "cfg/track.yaml")
          .add_option<bool>("-p", "--pause", "pause playing at first. default is false", false)
          .add_help_option()
          .parse(argc, argv);
    
    return parser;
}

bool isnum(string s) {
    stringstream sin(s);
    double t;
    char p;
    if (!(sin >> t))
        return false;
    if (sin >> p)
        return false;
    else
        return true;
}

void decode_results(YAML::Node node, std::vector<std::vector<detect::Object>>& dist) {
    dist.clear();
    int total_frame = node["frame"].as<int>();
    for (int i=0;i<total_frame;i++) {
        YAML::Node this_frame = node["results"][i];
        std::vector<detect::Object> objs;
        std::vector<std::vector<double>> res = this_frame.as<std::vector<std::vector<double>>>();
        for (auto re: res) {
            detect::Object obj;
            obj.rect.x = (int)re[0];
            obj.rect.y = (int)re[1];
            obj.rect.width = (int)re[2];
            obj.rect.height = (int)re[3];
            obj.prob = re[4];
            obj.label = (int)re[5];
            objs.push_back(obj);
        }
        dist.push_back(objs);
    }
}

int main(int argc, char* argv[]) {
    auto args = get_args(argc, argv);

    std::string result_file = args.get_argument_string("result_file");
    YAML::Node result_cfg = YAML::LoadFile(result_file);
    std::string source = result_cfg["source"].as<std::string>();
    std::vector<std::string> names = result_cfg["names"].as<std::vector<std::string>>();
    std::vector<std::vector<detect::Object>> results;
    decode_results(result_cfg, results);

    cv::VideoCapture cap;
    if (isnum(source))
        cap.open(atoi(source.c_str()));
    else
        cap.open(source);

    std::string track_config_file = args.get_option_string("--file");
    YAML::Node track_cfg = YAML::LoadFile(track_config_file);
    
    UCMC::Params track_params;
    track_params.a1 = track_cfg["a1"].as<double>();
    track_params.a2 = track_cfg["a2"].as<double>();
    track_params.wx = track_cfg["wx"].as<double>();
    track_params.wy = track_cfg["wy"].as<double>();
    track_params.vmax = track_cfg["vmax"].as<double>();
    track_params.max_age = track_cfg["max_age"].as<double>();
    track_params.high_score = track_cfg["high_score"].as<double>();
    track_params.conf_threshold = track_cfg["conf_threshold"].as<double>();
    track_params.dt = 1./ MAX(10, cap.get(cv::CAP_PROP_FPS));
    track_params.Ki = track_cfg["Ki"].as<std::vector<double>>();
    track_params.Ko = track_cfg["Ko"].as<std::vector<double>>();

    UCMC::Tracker tracker(track_params);

    cv::Mat frame;
    int key = -1;
    int delay = args.get_option_bool("--pause") ? 0 : 1;
    TimeCount t;

    float total_latency=0.;
    int frame_id=0;
    for(auto preds: results) {
        cap >> frame;
        if (frame.empty()) {
            // cout << "frame is empty, exit." << endl;
            cv::destroyAllWindows();
            break;
        }

        t.tic(0);
        std::vector<UCMC::Obj> track_result = tracker.update(preds);
        t.toc(0);


        frame = Track::draw_boxes(frame, track_result, names, 20, true);

        float latency=t.get_timeval_f(0);
        total_latency += latency;
        printf("%sframe %d ---> track latency: %.2f ms\n%s", 
               GREEN, 
               frame_id++,
               latency,
               END);
        
        cv::imshow("UCMCTrack result", frame);
        key = cv::waitKey(delay);
        switch (key) {
        case 27:   // esc
            cap.release();
            break;
        case 32:   // space
            delay = 1 - delay;
            break;
        default:
            break;
        }
    }
    cv::destroyAllWindows();
    printf("%saverage latency: %.2f ms%s\n", 
           GREEN, 
           total_latency / results.size(),
           END);
    return 0;
}
