//
// Created by phlf on 03/06/16.
//

#ifndef CAFER_CORE_DB_MANAGER_HPP
#define CAFER_CORE_DB_MANAGER_HPP

#include <ros/ros.h>

#include "cafer_core/cafer_core.hpp"
#include "cafer_core/WriteDataAction.h"
#include "actionlib/server/simple_action_server.h"

#include <condition_variable>
#include <thread>
#include <atomic>
#include <fstream>

#include <boost/filesystem.hpp>

/**
 * Facilities for meta-programming
 */
namespace mp_tools {

    template<std::size_t I = 0, typename FuncT, typename... Tp>
    inline typename std::enable_if<I == sizeof...(Tp), void>::type
    for_each(std::tuple<Tp...>&, FuncT)
    { }

    template<std::size_t I = 0, typename FuncT, typename... Tp>
    inline typename std::enable_if<I < sizeof...(Tp), void>::type
    for_each(std::tuple<Tp...>& t, FuncT f)
    {
        f(std::get<I>(t));
        for_each<I + 1, FuncT, Tp...>(t, f);
    }

    struct IsManagerEmpty {
        template<typename T>
        bool operator()(T& t) const
        {
            return t.data_size() == 0;
        }
    };

    struct RetrieveData {
        template<typename T>
        void operator()(T& t) const
        {
            /*

            dream_babbling::pose_goalActionFeedback trajectory_msg;
            if (_joints_value_manager->data_size() != 0) {
                trajectory_msg = _joints_value_manager->get();
                if (!trajectory_msg.feedback.joints_positions.empty()) {
                    fs_joints_data << "frame_" << trajectory_msg.header.stamp.sec + trajectory_msg.header.stamp.nsec <<
                    ":" << std::endl;
                    fs_joints_data << "  timestamp:" << std::endl;
                    fs_joints_data << "    sec:  " << trajectory_msg.header.stamp.sec << std::endl;
                    fs_joints_data << "    nsec:  " << trajectory_msg.header.stamp.nsec << std::endl;
                    fs_joints_data << "  joints_values:" << std::endl;
                    for (unsigned int i = 0; i < trajectory_msg.feedback.joints_positions.size(); i++) {
                        fs_joints_data << "    joint_" << i << ":  " << trajectory_msg.feedback.joints_positions[i] <<
                        std::endl;
                    }
                }
            }

            dream_babbling::rgbd_motion_data motion_msg;
            if (_rgbd_motion_data_manager->data_size() != 0) {
                motion_msg = _rgbd_motion_data_manager->get();

                //Necessary as saving 16bits png is unknown to OpenCV...
                depth = cv_bridge::toCvShare(motion_msg.depth, nullptr)->image;
                depth = cv::Mat(depth.rows, depth.cols, CV_8UC4, depth.data);

                if (!motion_msg.motion_rects.empty()) {
                    fs_motion_data << "frame_" << motion_msg.header.stamp.sec + motion_msg.header.stamp.nsec << ":" <<
                    std::endl;
                    fs_motion_data << "  timestamp:" << std::endl;
                    fs_motion_data << "    sec:  " << motion_msg.header.stamp.sec << std::endl;
                    fs_motion_data << "    nsec:  " << motion_msg.header.stamp.nsec << std::endl;
                    fs_motion_data << "  rects:" << std::endl;
                    for (unsigned int i = 0; i < motion_msg.motion_rects.size(); i++) {
                        fs_motion_data << "    rect_" << i << ":" << std::endl;
                        fs_motion_data << "      x:  " << motion_msg.motion_rects[i].x << std::endl;
                        fs_motion_data << "      y:  " << motion_msg.motion_rects[i].y << std::endl;
                        fs_motion_data << "      width:  " << motion_msg.motion_rects[i].width << std::endl;
                        fs_motion_data << "      height:  " << motion_msg.motion_rects[i].height << std::endl;
                    }
                }
            }

             imwrite(_fs_manager.iter_save_path() + "depth/" + std::to_string(motion_msg.header.stamp.sec) + "_" +
                        std::to_string(motion_msg.header.stamp.nsec) + ".png", depth);

             imwrite(_fs_manager.iter_save_path() + "rgb/" + std::to_string(motion_msg.header.stamp.sec) + "_" +
                        std::to_string(motion_msg.header.stamp.nsec) + ".jpeg",
                        cv_bridge::toCvShare(motion_msg.rgb, nullptr, "bgr8")->image);

        };
        */

        }
    };
}

namespace cafer_core {
    class filesystem_manager {
    public:
        filesystem_manager()
        {
            std::string ros_home;
            ros::get_environment_variable(ros_home, "ROS_HOME");
            _db_path = ros_home + "/" + ros::this_node::getNamespace() + "/db";
        }

        void add_new_iteration()
        {
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);

            if (_experiment_path.empty()) {
                std::stringstream ss;
                ss << std::put_time(std::localtime(&in_time_t), "%F-%T-%Z");
                _experiment_path = "experiment_" + ss.str();
            }
            if (_iter_path.empty()) {
                _counter++;
                _iter_path = "iteration_" + std::to_string(_counter);
            }

            while (boost::filesystem::exists(_db_path / _experiment_path / _iter_path)) {
                _counter++;
                _iter_path = "iteration_" + std::to_string(_counter);
            }
            boost::filesystem::create_directories(
                    _db_path / _experiment_path / _iter_path / _data_path / boost::filesystem::path("rgb"));
            boost::filesystem::create_directories(
                    _db_path / _experiment_path / _iter_path / _data_path / boost::filesystem::path("depth"));
        }

        const std::string iter_save_path() const
        {
            return (_db_path / _experiment_path / _iter_path / _data_path).string();
        }

    private:
        unsigned int _counter = 0;
        boost::filesystem::path _db_path;
        boost::filesystem::path _iter_path;
        boost::filesystem::path _experiment_path;
        boost::filesystem::path _data_path = "motion/scene/";

    };

    class DatabaseManager : public Component {
        using Component::Component;

    private:
        std::tuple<Ts ...> _objects;
        std::array<std::ofstream, sizeof...(Ts)> _ofstreams;

        std::vector<cafer_core::Manager> _managers;

        void write_data_cb()
        {
            //cafer_core::WriteDataFeedback feedback;
            cafer_core::WriteDataResult result;

            //Listen to the data topics
            client_connect_to_ros();

            //Try to acquire _cv_mutex: wait for the processing thread to finish previous task.
            _cv_mutex.lock();
            //Accept the new goal
            _write_data_action_server->acceptNewGoal();

            //Let the processing thread process data
            _is_active.store(true);
            _condition_variable.notify_one();
            _cv_mutex.unlock();

            ROS_INFO("New data recording request accepted.");
        }

        void preempt_cb()
        {
            //Stop listening to the data topics
            client_disconnect_from_ros();

            //Set the server status to preempted
            _is_active.store(false);
        }

        std::unique_ptr<actionlib::SimpleActionServer<cafer_core::WriteDataAction>> _write_data_action_server;

        std::unique_ptr<std::thread> _processing_thread;
        std::condition_variable _condition_variable;
        std::mutex _cv_mutex;
        std::atomic<bool> _is_active{false};

        filesystem_manager _fs_manager;

        void processing()
        {
            std::ofstream fs_motion_data, fs_joints_data;
            std::unique_lock<std::mutex> lock(_cv_mutex);
            bool are_managers_empty = true;

            //Processing loop
            for (; ;) {
                //Wait for signal to process data if server is inactive/preempted and there is no data.
                //The thread can be spuriously woken-up inconsequently.

                if (!_is_active) {
                    are_managers_empty &= mp_tools::for_each(_objects, mp_tools::IsManagerEmpty);
                    if (are_managers_empty) {

                        //Is server still active?
                        if (_write_data_action_server->isActive()) {
                            //Change server state and notify the client
                            _write_data_action_server->setSucceeded();
                        }

                        //Release last opened filestreams
                        for (auto& fs:_ofstreams) {
                            if (fs.is_open()) {
                                fs.close();
                            }
                        }

                        ROS_INFO("DB manager is waiting.");
                        //Release _cv_mutex and blocks the thread.
                        _condition_variable.wait(lock);
                        //Thread notified: acquires _cv_mutex and resume.
                        ROS_INFO("DB manager is recording data.");

                        _fs_manager.add_new_iteration();


                        for (auto fs:_ofstreams) {

                        }
                        //TODO for each fs, open corresponding file
                        /*
                        fs_motion_data.open(_fs_manager.iter_save_path() + "motion_rects.yml", std::ios::out);
                        fs_joints_data.open(_fs_manager.iter_save_path() + "joints_trajectories.yml", std::ios::out);
                         */
                    }
                }
                mp_tools::for_each(_objects, mp_tools::RetrieveData);

            }
        }

    public:

        DatabaseManager()
        { }


//        template<std::size_t I>
//        auto get_object() -> decltype(std::get<I>(_objects))
//        {
//            return std::get<I>(_objects);
//        }

        ~DatabaseManager()
        {
            client_disconnect_from_ros();
            _processing_thread->join();
            _processing_thread.reset();
            _rgbd_motion_data_manager.reset();
            _joints_value_manager.reset();
            _write_data_action_server.reset();
        }

        void init() override
        {
            _write_data_action_server.reset(
                    new actionlib::SimpleActionServer<cafer_core::WriteDataAction>(*cafer_core::ros_nh,
                                                                                   ros::this_node::getName(),
                                                                                   false));
            _write_data_action_server->registerGoalCallback(boost::bind(&DatabaseManager::write_data_cb, this));
            _write_data_action_server->registerPreemptCallback(boost::bind(&DatabaseManager::preempt_cb, this));

            _processing_thread.reset(new std::thread(std::bind(&DatabaseManager::processing, this)));

            _write_data_action_server->start();

            _is_init = true;
        }

        void client_connect_to_ros() override
        {
            std::string motion_detector_topic_name;
            std::string joints_topic_name;

            //TODO this level of knowledge should be handled by the managers
            cafer_core::ros_nh->getParam("/dream_babbling/motion_detector_topic", motion_detector_topic_name);
            cafer_core::ros_nh->getParam("/dream_babbling/robot_controller_feedback_topic", joints_topic_name);

            for(const auto& manager:_managers)
            {
                manager.listen_to();
            }
        }

        void client_disconnect_from_ros() override
        {
            for(const auto& manager:_managers)
            {
                manager.disconnect_from_ros();
            }
        }

        void update() override
        { };
    };

}

#endif //CAFER_CORE_DB_MANAGER_HPP
