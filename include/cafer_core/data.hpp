//
// Created by phlf on 03/06/16.
//

#ifndef CAFER_CORE_DATA_HPP
#define CAFER_CORE_DATA_HPP

#include <ros/ros.h>
#include <ros/serialization.h>
#include <topic_tools/shape_shifter.h>

namespace cafer_core {
    /**
     * This interface defines how a type of data coming from a ROS message will be stored in a DatabaseManager object.
     * It is a wrapper over a ROS message and must be implemented to be used as a Manager template parameter.
     */
    class Data {
    public:
        /**
         * Constructs a new Data object from a ShaperShifter (generic) message.
         * @param msg The ShapeShifter message to wrap over.
         */
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

        /**
         * Copy constructor for the Data object.
         * @param data The Data object to clone.
         */
        Data(const Data& data) : _stored_time(data._stored_time)
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

        /**
         * Returns the wrapped  ShapeShifter message.
         * @return The ShapeShifter message to return.
         */
        const topic_tools::ShapeShifter& get_stored_msg() const
        {
            return _stored_msg;
        }

        /**
         * @brief Returns the data from the wrapped message as a map<string,string>.
         * @details The map associates the name of the record where part of the data is stored
         * and the part of the data itself as a string in the format defined by the user.
         * @return The map associating record(s)'s name and piece(s) of data
         */
        virtual std::map<std::string, std::string> get_serialized_data() const = 0;
        void set_stored_time(double t){_stored_time = t;}
        double get_stored_time(){return _stored_time;}

    protected:
        topic_tools::ShapeShifter _stored_msg;
        double _stored_time = 0.0f;
    };
}
#endif //CAFER_CORE_DATA_HPP
