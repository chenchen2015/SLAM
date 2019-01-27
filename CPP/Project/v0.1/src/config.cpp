#include <xslam/config.h>

namespace xslam {

shared_ptr<Config> Config::config_ = nullptr;

void Config::setParameterFile(const std::string& filename) {
    if (!config_) config_ = shared_ptr<Config>(new Config);
    config_->file_.open(filename.c_str(), cv::FileStorage::READ);
    if (config_->file_.isOpened() == false) {
        cout << "[Config ERROR]: cannot open parameter file " << filename
             << endl;
        std::cerr << "parameter file " << filename << " does not exist."
                  << std::endl;
        config_->file_.release();
        return;
    }
}

Config::~Config() {
    if (file_.isOpened()) file_.release();
}

}  // namespace xslam
