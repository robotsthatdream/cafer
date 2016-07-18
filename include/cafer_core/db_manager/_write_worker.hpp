//
// Created by phlf on 03/06/16.
//

#ifndef CAFER_CORE_WRITE_WORKER_HPP
#define CAFER_CORE_WRITE_WORKER_HPP

#include <ros/ros.h>

#include <condition_variable>
#include <thread>
#include <atomic>

#include "cafer_core/cafer_core.hpp"

namespace cafer_core {
    namespace _details {
        class _WriteWorker {
        public:
            _WriteWorker();

            _WriteWorker(_Wave*);

            ~_WriteWorker();

            void awake_worker();

            void pause_worker();

            void link_to_wave(_Wave*);

        private:
            cafer_core::shared_ptr<_Wave> _wave;

            std::atomic<bool> _is_active{false};
            std::unique_ptr<std::thread> _processing_thread;
            std::condition_variable _signal_processing_thread;
            std::mutex _signal_process_mutex;

            void _processing();
        };
    }
}
#endif //CAFER_CORE_WRITE_WORKER_HPP
