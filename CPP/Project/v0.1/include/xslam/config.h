#pragma once

// config.h

#include <xslam/xslam.h>

namespace xslam {

class Config {
   private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

   public:
    Config() = delete;  // delete constructor makes a singleton
    ~Config();          // close the file when deconstructing

    // set a new config file
    static void setParameterFile(const std::string& filename);

    // access the parameter values
    template <typename T>
    static T get(const std::string& key) {
        return T(Config::config_->file_[key]);
    }
};

}  // namespace xslam