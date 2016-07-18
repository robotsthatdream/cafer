//
// Created by phlf on 03/06/16.
//

#ifndef CAFER_CORE_WAVE_HPP
#define CAFER_CORE_WAVE_HPP

#include <ros/ros.h>

#include "cafer_core/cafer_core.hpp"

namespace cafer_core {
    namespace _details {
        class _Wave {
        public:
            std::string name;
            bool sequential;
            std::map<std::string, std::string> data_topics;
            std::map<std::string, std::string> data_structure;

            _FilesystemManager fs_manager;
            std::vector<std::unique_ptr<cafer_core::IManager>> managers;
            cafer_core::shared_ptr<cafer_core::Publisher> status_publisher;

            _Wave(std::string&, cafer_core::Publisher&);

            void add_manager(cafer_core::IManager&);

            bool no_data_left();

            void connect();

            void disconnect();

        private:
            _WriteWorker _write_worker;

        };
    }
}
#endif //CAFER_CORE_WAVE_HPP
