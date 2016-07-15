//
// Created by phlf on 03/06/16.
//

#ifndef CAFER_CORE_DATA_HPP
#define CAFER_CORE_DATA_HPP

#include <ros/ros.h>

namespace cafer_core {
    template<typename Msg>
    class Data {
    public:
        using type = typename Msg;

        Data(Msg msg) : _stored_msg(msg)
        { }

        const Msg get_stored_msg const
        {
            return _stored_msg;
        };

    protected:
        Msg _stored_msg;

        virtual std::map<std::string, std::string> get_serialized_data() const = 0;
    };
}
#endif //CAFER_CORE_DATA_HPP
