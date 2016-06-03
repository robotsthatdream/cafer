//
// Created by phlf on 03/06/16.
//

#ifndef CAFER_CORE_DATA_HPP
#define CAFER_CORE_DATA_HPP

#include <ros/ros.h>

template<typename Msg>
class Data {
public:
    const data_type;

    //TODO Complete this class
    Data() : data_type(ros::message_traits::DataType<Msg>::value())
    { }

    friend std::ostream& operator<<(std::ostream& os, const Data& obj)
    {
        obj.stream(os);
        return os;
    }

protected:
    virtual void stream(std::ostream& os) = 0;
};

#endif //CAFER_CORE_DATA_HPP
