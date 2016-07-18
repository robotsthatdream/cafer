//
// Created by phlf on 03/06/16.
//

#ifndef CAFER_CORE_DB_MANAGER_HPP
#define CAFER_CORE_DB_MANAGER_HPP

#include <ros/ros.h>

#include <fstream>
#include <boost/filesystem.hpp>

#include "cafer_core/cafer_core.hpp"

namespace cafer_core {
    namespace _details {
        /**
            * Class manipulating the filesystem.
            */
        class _FilesystemManager {
        public:
            _FilesystemManager(_Wave*);

            ~_FilesystemManager();

            void new_records();

            void close_records();

            void save_data(cafer_core::Data&&);

        private:
            cafer_core::shared_ptr<_Wave> _wave;

            uint32_t _counter = 0;
            std::map<std::string, std::ofstream> _records;
            boost::filesystem::path _ros_home;


        };
    }
}
#endif //CAFER_CORE_DB_MANAGER_HPP
