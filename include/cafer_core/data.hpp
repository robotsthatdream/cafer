//
// Created by phlf on 03/06/16.
//

#ifndef CAFER_CORE_DATA_HPP
#define CAFER_CORE_DATA_HPP

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

namespace cafer_core {
    class Data {
    public:
        Data(topic_tools::ShapeShifter msg) : _stored_msg(msg)
        { }

        const topic_tools::ShapeShifter get_stored_msg() const
        {
            return _stored_msg;
        };

        virtual std::map<std::string, std::string> get_serialized_data() const = 0;

    protected:
        topic_tools::ShapeShifter _stored_msg;
    };
}
#endif //CAFER_CORE_DATA_HPP
