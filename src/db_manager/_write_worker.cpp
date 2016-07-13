//
// Created by phlf on 13/07/16.
//

#include "db_manager.h"

DatabaseManager::_WriteWorker::_WriteWorker()
{
    _processing_thread.reset(new std::thread(&_WriteWorker::_processing, this));
}

void DatabaseManager::_WriteWorker::_processing()
{
    std::unique_lock<std::mutex> lock(_signal_process_mutex);
    dream_babbling::db_manager_status db_status_msg;

    db_status_msg.state = static_cast<uint8_t>(true);

    //Processing loop
    while (ros::ok()) {
        //Wait for signal to process data if server is inactive/preempted and there is no data.
        //The thread can be spuriously woken-up inconsequently.
        if (!_wave->is_active && _wave->no_data_left()) {

            _wave->fs_manager.close_records();

            _status_publisher->publish(db_status_msg);
            //Release _signal_process_mutex and blocks the thread.
            _signal_processing_thread.wait(lock);
            //Thread notified: acquires _signal_process_mutex and resume.
            ROS_INFO_STREAM("DB is now recording data from _wave " << _wave->name);
            _wave->fs_manager.new_records();

//            fs_motion_data.open(_fs_manager.iter_save_path() + "motion_rects.yml", std::ios::out);
//            fs_joints_data.open(_fs_manager.iter_save_path() + "joints_trajectories.yml", std::ios::out);
//            fs_soi_classifier.open(_fs_manager.iter_save_path() + "soi_classifier.yml", std::ios::out);
        }
        for (const auto& manager:_wave->managers) {
            //_wave->fs_manager.save_data(manager.get_data());
        }
    }
}

DatabaseManager::_WriteWorker::~_WriteWorker()
{
    _signal_process_mutex.lock();
    //Let the _processing thread go to the end of the loop.
    _wave->is_active.store(true);
    _signal_processing_thread.notify_one();
    _signal_process_mutex.unlock();
    _processing_thread->join();

    _processing_thread.reset();
}

void DatabaseManager::_WriteWorker::_awake_worker()
{
    //Try to acquire _signal_process_mutex: wait for the _processing thread to finish previous task.
    _signal_process_mutex.lock();
    //Let the _processing thread process data
    _wave->is_active.store(true);
    _signal_processing_thread.notify_one();
    _signal_process_mutex.unlock();
}

void DatabaseManager::_WriteWorker::_link_to_wave(_Wave& wave)
{
    //Try to acquire _signal_process_mutex: wait for the _processing thread to finish previous task.
    _signal_process_mutex.lock();
    _wave.reset(&wave);
    _signal_process_mutex.unlock();
}