//
// Created by phlf on 13/07/16.
//

#include "cafer_core/db_manager.hpp"

using namespace cafer_core;

DatabaseManager::_WriteWorker::_WriteWorker()
{}

DatabaseManager::_WriteWorker::_WriteWorker(_Wave* parent)
{
    link_to_wave(parent);
}

void DatabaseManager::_WriteWorker::_processing()
{
    std::unique_lock<std::mutex> lock(_signal_process_mutex);

    //Processing loop
    double last_time = 0.0;
    bool new_start = false;
    while (!_finish) {
        //Wait for signal to process data if server is inactive/preempted and there is no data.
        //The thread can be spuriously woken-up inconsequently.
        if ((!_is_active && _wave->no_data_left()) || (new_start && last_time > _wave->start_time)) {
            _wave->fs_manager.close_records();

            _wave->ready = true;

            //Release _signal_process_mutex and blocks the thread.
            _signal_processing_thread.wait(lock);
            //Thread notified: acquires _signal_process_mutex and resume.
            if (!_finish) {
                ROS_INFO_STREAM("DB is now recording data from " << _wave->name);
            }
            _wave->fs_manager.new_records();
            new_start = false;
        }

        for (auto& manager:_wave->managers) {
            if (manager->data_size() != 0) {

                last_time = manager->get_time();
                _wave->fs_manager.save_data(manager->get());
            }

        }
        new_start = last_time < _wave->start_time;
    }
}

DatabaseManager::_WriteWorker::~_WriteWorker()
{
    _signal_process_mutex.lock();
    //Let the _processing thread go to the end of the loop.
    _is_active.store(true);
    _signal_processing_thread.notify_one();
    _finish.store(true);
    _signal_process_mutex.unlock();
    _processing_thread->join();

    _processing_thread.reset();
}

void DatabaseManager::_WriteWorker::awake_worker()
{
    //Try to acquire _signal_process_mutex: wait for the _processing thread to finish previous task.
    _signal_process_mutex.lock();
    //Let the _processing thread process data
    _is_active.store(true);
    _signal_processing_thread.notify_one();
    _signal_process_mutex.unlock();
}

void DatabaseManager::_WriteWorker::pause_worker()
{
    _is_active.store(false);
}

void DatabaseManager::_WriteWorker::link_to_wave(_Wave* wave)
{
    _wave = wave;
    _processing_thread.reset(new std::thread(&_WriteWorker::_processing, this));
}
