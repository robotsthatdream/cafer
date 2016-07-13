//
// Created by phlf on 07/07/16.
//

#ifndef DREAM_BABBLING_FILESYSTEM_H
#define DREAM_BABBLING_FILESYSTEM_H

#include <ros/ros.h>
#include <chrono>
#include <fstream>
#include <boost/filesystem.hpp>
#include <cafer_core/cafer_core.hpp>

/**
 * Class manipulating the filesystem.
 */
class FilesystemManager {
public:
    FilesystemManager();
    ~FilesystemManager();

    void new_records();

    void close_records();

    void save_data();

private:
    boost::filesystem::path _db_path;
    std::vector<std::ofstream> _fstreams;
};


#endif //DREAM_BABBLING_FILESYSTEM_H
