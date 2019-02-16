// Copyright 2018 Chen, Zhuhui
//
// Permission is hereby granted, free of charge, to any person
// obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without
// restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies
// of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
// BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
// ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef _X_ROS_BASE_NODE_HPP_
#define _X_ROS_BASE_NODE_HPP_

#include <string>
#include <exception>

#include <ros/ros.h>
#include <ros/package.h>

namespace xros {
    inline std::string StripNamespace(const std::string& text) {
        const std::size_t LEN_DOUBLE_COLON = std::string("::").size();
        std::size_t last_ns = text.rfind("::");
        if (last_ns == std::string::npos) {
            return text;
        } else {
            return text.substr(last_ns + LEN_DOUBLE_COLON);
        }
    }
}

#define XROS_CONSTRUCTOR_DEFAULT_PARAMETERS int argc, char* argv[], const std::string& node_name, double rate=0.0
#define XROS_CONSTRUCTOR_INIT_RUNNABLE RunnableNode(argc, argv, node_name, rate)
#define XROS_DECLARE_RUNNABLE_NODE_CONSTRUCTOR(clazz) inline clazz(XROS_CONSTRUCTOR_DEFAULT_PARAMETERS) : XROS_CONSTRUCTOR_INIT_RUNNABLE {}

#define XROS_RUNNABLE_NODE_MAIN_COMPLETE(clazz,name,rate) \
int main(int argc, char* argv[]) { \
  clazz node(argc, argv, xros::StripNamespace(name), rate); \
  return node.Run(); \
}

#define XROS_RUNNABLE_NODE_MAIN_NO_RATE(clazz,name) \
int main(int argc, char* argv[]) { \
  clazz node(argc, argv, xros::StripNamespace(name)); \
  return node.Run(); \
}

#define XROS_RUNNABLE_NODE_MAIN(clazz) XROS_RUNNABLE_NODE_MAIN_NO_RATE(clazz,#clazz)

#define XROS_RUNNABLE_NODE_MAIN_AT_RATE(clazz,rate) XROS_RUNNABLE_NODE_MAIN_COMPLETE(clazz,#clazz,rate)

namespace xros {
    
    class NodeBase {
    protected:
        inline NodeBase(int argc, char **argv, const std::string& node_name) {
            ros::init(argc, argv, node_name);
        }

        inline std::string GetPackageFileName(const std::string &package_url) {
            ROS_DEBUG_STREAM("Determining package url: " << package_url);
            std::size_t prefix_len = std::string("package://").length();
            std::size_t rest = package_url.find('/', prefix_len);

            std::string package(package_url.substr(prefix_len, rest - prefix_len));

            std::string pkg_path(ros::package::getPath(package));

            if (pkg_path.empty()) {
                ROS_WARN_STREAM("Unknown package: " << pkg_path << ", ignored.");
            } else {
                return pkg_path + package_url.substr(rest);
            }
        }

    };

    template <typename T>
    class RunnableNode : public NodeBase {
    protected:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        double rate_;

    protected:
        inline RunnableNode(int argc, char **argv, const std::string& node_name, double rate)
            : NodeBase(argc, argv, node_name),
              nh_(""),
              private_nh_("~"),
              rate_(rate) { }

        inline RunnableNode(int argc, char **argv, const std::string& node_name)
            : RunnableNode(argc, argv, node_name, 0.0) { }
            
        /**
         * Method called before entering node handle loop
         */
        virtual void Init() {}

        /**
         * Method called every time in the node handle loop. CRTP is used to avoid 
         * virtual function overhead since it is called repeatedly in the loop
         */
        inline void Execute() {}

        /**
         * Method called after node handle loop is exited, before the node end
         */
        virtual void Destroy() {}
    public:
        inline int Run() {
            try {
                this->Init();
                if (rate_ > 0) {
                    ros::Rate rate(rate_);
                    while (private_nh_.ok()) {
                        static_cast<T*>(this)->Execute();
                        ros::spinOnce();
                        rate.sleep();
                    }
                } else {
                    while (private_nh_.ok()) {
                        static_cast<T*>(this)->Execute();
                        ros::spin();
                    }
                }
                this->Destroy();
            } catch (const std::exception& e) {
                ROS_ERROR("Exception caught while running node.\n  what() => %s", e.what());
                return -1;
            }
        }
    };

}

#endif