//
// Created by phlf on 03/06/16.
//

#ifndef CAFER_CORE_DATA_HPP
#define CAFER_CORE_DATA_HPP

#include <ros/ros.h>
#include <ros/serialization.h>
#include <topic_tools/shape_shifter.h>

namespace cafer_core {
    class Data {
    public:
        Data(const topic_tools::ShapeShifter& msg)
        {
            uint8_t* data = new uint8_t[msg.size()];
            ros::serialization::OStream ostream(data, msg.size());

            _stored_msg.morph(msg.getMD5Sum(), msg.getDataType(), msg.getMessageDefinition(), "latching=0");

            msg.write<ros::serialization::Stream>(ostream);

            ros::serialization::IStream istream(data, msg.size());

            _stored_msg.read<ros::serialization::Stream>(istream);

            delete[] data;
        }

        Data(const Data& data)
        {
            uint8_t* data_buff = new uint8_t[data._stored_msg.size()];

            ros::serialization::OStream ostream(data_buff, data._stored_msg.size());

            _stored_msg.morph(data._stored_msg.getMD5Sum(), data._stored_msg.getDataType(),
                              data._stored_msg.getMessageDefinition(), "latching=0");

            data._stored_msg.write<ros::serialization::Stream>(ostream);

            ros::serialization::IStream istream(data_buff, data._stored_msg.size());

            _stored_msg.read<ros::serialization::Stream>(istream);

            delete[] data_buff;
        }

        virtual ~Data()
        {}

        const topic_tools::ShapeShifter& get_stored_msg() const
        {
            return _stored_msg;
        };

        virtual std::map<std::string, std::string> get_serialized_data() const = 0;

    protected:
        topic_tools::ShapeShifter _stored_msg;
    };
}
#endif //CAFER_CORE_DATA_HPP
