//| This file is a part of the CAFER framework developped within
//| the DREAM project (http://www.robotsthatdream.eu/).
//| Copyright 2015, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s):
//|   * Stephane Doncieux, stephane.doncieux@isir.upmc.fr
//|   * Léni Le Goff, le_goff@isir.upmc.fr
//|
//| This experiment allows to generate neural networks for simple
//| navigation tasks (obstacle avoidance and maze navigation).
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

#ifndef _MANAGER_HPP
#define _MANAGER_HPP

#include <iostream>
#include <ctime>
#include <random>
#include <sstream>
#include <mutex>
#include <unordered_map>
#include <deque>

#include <std_msgs/Time.h>
#include <std_msgs/Header.h>

#include "cafer_core/component.hpp"

namespace cafer_core {

    /**
     * The details namespace shall not be accessed by users, it is meant to hide some implementation details.
     */
    namespace details {
        template<typename Msg>
        using Map=std::unordered_map<u_int32_t, Msg>;

        template<typename Msg>
        using Queue=std::deque<Msg>;
    }

    /**
     * class Manager<Msg, DataContainer, DerivedClass> \n
     * A data manager to handle ROS messages (like images, features or policies).
     */
    template<typename Msg, template<typename> class DataContainer,
            template<typename, template<typename> class> class DerivedClass>
    class ManagerBase {
    public:

        /**
         * @brief Manager constructor
         * @param type specify which of data manager (and not the type of data)
         * @param name of the manager
         * @param description a short description of the manager (optionnal)
         */
        ManagerBase(std::string data_topic_param, std::string type = "", std::string name = "",
                    std::string description = "")
                : _type(type), _name(name), _description(description)
        {
            //Retrieve data_topic from ROS parameter
            ros_nh->getParam(data_topic_param, _data_topic);
            //init random number generator for random access
            std::seed_seq seed = {std::time(0)};
            _gen.seed(seed);
        }

        ManagerBase(ManagerBase&& manager) = default;

        ManagerBase(const ManagerBase& manager) = delete;

        void disconnect_from_ros()
        {
            _subcriber.reset();
        }

        bool is_initialized()
        {
            return true;
        }

        void update()
        {

        }

        void operator<<(const std::string& topic)
        {
            _data_topic = topic;
            listen_to(topic);
        }

        /**
        * @brief Connects to the default topic and listen to it.
        */
        void listen_to()
        {
            listen_to(_data_topic);
        }

        /**
         * @brief Connects to a specific topic and listen to it.
         * @param The topic to listen to.
         */
        void listen_to(const std::string& topic)
        {
            auto add_callback = [this](const shared_ptr<Msg>& msg)
            {
                static_cast<DerivedClass<Msg, DataContainer> *>(this)->add(*msg);
            };

            _subcriber.reset(new Subscriber(ros_nh->subscribe<const shared_ptr<Msg>>(topic, 10, add_callback)));
        }

        /**
        * @brief Returns the number of elements in the data container.
        * @return The manager container's size.
        */
        size_t data_size()
        {
            size_t _container_size;
            _container_mutex.lock();
            _container_size = _data_set.size();
            _container_mutex.unlock();
            return _container_size;
        }

    protected:

        DataContainer<Msg> _data_set;

        long int _id;
        std::string _name;
        std::string _description;
        std::string _type;
        std::string _data_topic;

        std::unique_ptr<Subscriber> _subcriber;

        std::mt19937 _gen;

        //Mutex to protect the _data_set from concurrent access
        std::mutex _container_mutex;
    };

    //Declaring the Manager class, inheriting from ManagerBase.
    template<typename Msg, template<typename> class DataContainer>
    class Manager : public ManagerBase<Msg, DataContainer, Manager> {
    };

    //Partial template specialization of the Manager class using unordered_map as container.
    template<typename Msg>
    class Manager<Msg, details::Map> : public ManagerBase<Msg, details::Map, Manager> {
        //Defining Base as a private alias for the template pattern.
        //Here the namespace ::cafer_core:: should be specified, depending on the compiler to find the Manager template.
        using Base=ManagerBase<Msg, details::Map, Manager>;
        //Inheriting base class constructor
        using Base::Base;
    public:

        /**
         * @brief add a msg to the container of Manager
         * @param msg the message to add
         */
        void add(const Msg& msg)
        {
            Base::_container_mutex.lock();
            Base::_data_set.emplace(msg.header.seq, msg);
            Base::_container_mutex.unlock();
        }

        /**
        * @brief Get an element from the data container.
        * @return The returned message/element.
        */
        Msg get()
        {
            std::uniform_int_distribution<> dist(0., Base::_data_set.size() - 1);
            Base::_container_mutex.lock();
            auto random_it = std::next(std::begin(Base::_data_set), dist(Base::_gen));
            Base::_container_mutex.unlock();
            Msg res = random_it->second;
            return res;
        }

        /**
         * @brief remove a element of the container of the manager
         * @param id identifier of the msg to remove
         * @return should be 1 in success case and 0 otherwise.
         */
        size_t remove(const u_int32_t& h)
        {
            size_t return_val;
            Base::_container_mutex.lock();
            return_val = Base::_data_set.erase(h);
            Base::_container_mutex.unlock();
            return return_val;
        }

        /**
        * @brief search a precise msg by is identifier
        * @param id identifier of the searched msg
        */
        Msg search(const u_int32_t& id)
        {
            Msg return_msg;
            Base::_container_mutex.lock();
            return_msg = Base::_data_set.find(id)->second;
            Base::_container_mutex.unlock();
            return return_msg;
        }
    };

    //Partial template specialization of the Manager class using deque as container.
    template<typename Msg>
    class Manager<Msg, details::Queue> : public ManagerBase<Msg, details::Queue, Manager> {
        //Defining Base as a private alias for the template pattern.
        //Here the namespace ::cafer_core:: should be specified, depending on the compiler to find the Manager template.
        using Base=ManagerBase<Msg, details::Queue, Manager>;
        //Inheriting base class constructor
        using Base::Base;

    public:

        /**
         * @brief add a msg to the container of Manager
         * @param msg the message to add
         */
        void add(const Msg& msg)
        {
            Base::_container_mutex.lock();
            Base::_data_set.push_back(msg);
            Base::_container_mutex.unlock();
        }

        /**
        * @brief Get an element from the data container.
        * @return The returned message/element.
        */
        Msg get()
        {
            Msg msg;

            Base::_container_mutex.lock();
            msg = Base::_data_set.front();
            Base::_data_set.pop_front();
            Base::_container_mutex.unlock();

            return msg;
        }
    };

    //Namespace aliases to simplify template usage.
    template<typename Msg>
    using ManagerMap=Manager<Msg, details::Map>;

    template<typename Msg>
    using ManagerQueue=Manager<Msg, details::Queue>;
}

#endif //_MANAGER_HPP
