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
#include <type_traits>

#include <std_msgs/Time.h>
#include <std_msgs/Header.h>

#include "cafer_core/component.hpp"
#include "cafer_core/data.hpp"

namespace cafer_core {
    /**
     * This namespace shall not be accessed by users, it is meant to hide some implementation details.
     */
    namespace _details {
        template<typename TData>
        using Map=std::unordered_map<uint32_t, TData>;

        template<typename TData>
        using Queue=std::deque<TData>;

        template<typename TData>
        using MultiMap=std::multimap<std::string,TData>;
    }

    /**
     * class Manager<TData, DataContainer, DerivedClass> \n
     * A data manager to handle ROS messages (like images, features or policies).
     */
    class IManager {
    public:

        IManager() : _id(0), _name(""), _description(""), _type("")
        {
            //init random number generator for random access
            std::seed_seq seed = {std::time(0)};
            _gen.seed(seed);
        }

        /**
         * @brief Manager constructor
         * @param type specify which of data manager (and not the type of data)
         * @param name of the manager
         * @param description a short description of the manager (optionnal)
         */
        IManager(std::string data_topic_param, std::string type = "", std::string name = "",
                 std::string description = "")
                : _name(name), _description(description), _type(type)
        {
            //Retrieve data_topic from ROS parameter
            ros_nh->getParam(data_topic_param, _data_topic);
            //init random number generator for random access
            std::seed_seq seed = {std::time(0)};
            _gen.seed(seed);
        }

        IManager(IManager&& manager) = default;

        IManager(const IManager& manager) = delete;

        virtual ~IManager() = default;

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
        }

        /**
        * @brief Connects to the default topic and listen to it.
        */
        void listen_to()
        {
            if (!_data_topic.empty()) {
                listen_to(_data_topic);
            }
        }

        /**
         * @brief Connects to a specific topic and listen to it.
         * @param The topic to listen to.
         */
        virtual void listen_to(const std::string& topic) = 0;

        /**
        * @brief Returns the number of elements in the data container.
        * @return The manager container's size.
        */
        virtual size_t data_size() = 0;

        virtual std::unique_ptr<cafer_core::Data> get() = 0;

    protected:

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

    /**
     * class Manager<TData, DataContainer, DerivedClass> \n
     * A data manager to handle ROS messages (like images, features or policies).
     */
    template<typename TData, template<typename> class DataContainer,
            template<typename, template<typename> class> class DerivedClass>
    class ManagerBase : public IManager {
        static_assert(std::is_base_of<cafer_core::Data, TData>::value,
                      "The Data template argument should be derived from Data");
    public:
        using IManager::IManager;

        virtual ~ManagerBase() = default;

        /**
         * @brief Connects to a specific topic and listen to it.
         * @param The topic to listen to.
         */
        void listen_to(const std::string& topic) override
        {
            auto add_callback = [this](const shared_ptr<const topic_tools::ShapeShifter>& msg)
            {
                static_cast<DerivedClass<TData, DataContainer>*>(this)->add(*msg);
            };

            _subcriber.reset(new Subscriber(ros_nh->subscribe<topic_tools::ShapeShifter>(topic, 1000, add_callback)));
        }

        /**
        * @brief Returns the number of elements in the data container.
        * @return The manager container's size.
        */
        size_t data_size() override
        {
            size_t _container_size;
            _container_mutex.lock();
            _container_size = _data_set.size();
            _container_mutex.unlock();
            return _container_size;
        }

        std::unique_ptr<cafer_core::Data> get() override
        {
            return static_cast<DerivedClass<TData, DataContainer>*>(this)->get();
        }

    protected:

        DataContainer<TData> _data_set;
    };

    //Declaring the Manager class, inheriting from ManagerBase.
    template<typename TData, template<typename> class DataContainer>
    class Manager : public ManagerBase<TData, DataContainer, Manager> {
    };

    //TODO: fix this class to use Data interface and ShapeShifter messages
    //Partial template specialization of the Manager class using unordered_map as container.
    template<typename TData>
    class Manager<TData, _details::Map> : public ManagerBase<TData, _details::Map, Manager> {
        //Defining Base as a private alias for the template pattern.
        //Here the namespace ::cafer_core:: should be specified, depending on the compiler to find the Manager template.
        using Base=ManagerBase<TData, _details::Map, ::cafer_core::Manager>;
        //Inheriting base class constructor
        using Base::Base;
    public:

        /**
         * @brief add a data object to the container of Manager
         * @param data the message to add
         */
        void add(const topic_tools::ShapeShifter& msg)
        {
            Base::_container_mutex.lock();
            Base::_data_set.emplace(ros::Time::now().nsec, TData(msg));
            Base::_container_mutex.unlock();
        }

        /**
        * @brief Random access to an element from the data container.
        * @return The returned message/element.
        */
        std::unique_ptr<cafer_core::Data> get()
        {
            std::uniform_int_distribution<> dist(0., Base::_data_set.size() - 1);
            Base::_container_mutex.lock();
            auto random_it = std::next(std::begin(Base::_data_set), dist(Base::_gen));
            Base::_container_mutex.unlock();
            std::unique_ptr<cafer_core::Data> data(&random_it->second);

            return data;
        }

        /**
         * @brief remove a element of the container of the manager
         * @param id identifier of the data to remove
         * @return should be 1 in success case and 0 otherwise.
         */
        size_t remove(const uint32_t& h)
        {
            size_t return_val;
            Base::_container_mutex.lock();
            return_val = Base::_data_set.erase(h);
            Base::_container_mutex.unlock();
            return return_val;
        }

        /**
        * @brief search specific data by its identifier
        * @param id identifier of the searched data object
        */
        bool search(const u_int32_t& id, cafer_core::Data& data)
        {
            bool res;
            Base::_container_mutex.lock();
            auto it = Base::_data_set.find(id);
            if (it == Base::_data_set.end()) {
                res = false;
            }
            else {
                res = true;
                data = Base::_data_set.find(id)->second;
            }
            Base::_container_mutex.unlock();
            return res;
        }
    };

    //Partial template specialization of the Manager class using deque as container.
    template<typename TData>
    class Manager<TData, _details::Queue> : public ManagerBase<TData, _details::Queue, Manager> {
        //Defining Base as a private alias for the template pattern.
        //Here the namespace ::cafer_core:: should be specified, depending on the compiler to find the Manager template.
        using Base=ManagerBase<TData, _details::Queue, ::cafer_core::Manager>;
        //Inheriting base class constructor
        using Base::Base;

    public:

        /**
         * @brief Adds a data object to the container of Manager
         * @param data the data object to add
         */
        void add(const topic_tools::ShapeShifter& msg)
        {
            Base::_container_mutex.lock();
            Base::_data_set.emplace_back(TData(msg));
            Base::_container_mutex.unlock();
        }

        /**
        * @brief Get an element from the data container.
        * Get return the element in the front of the queue and suppress it, i.e. return the first element added.
        * @return The returned message/element.
        */
        std::unique_ptr<cafer_core::Data> get()
        {
            std::unique_ptr<cafer_core::Data> data;

            Base::_container_mutex.lock();
            data.reset(new TData(Base::_data_set.front()));
            Base::_data_set.pop_front();
            Base::_container_mutex.unlock();

            return data;
        }
    };

    //TODO: fix this class to use Data interface and ShapeShifter messages
    //Partial template specialization of the Manager class using deque as container.
    template<typename TData>
    class Manager<TData, _details::MultiMap> : public ManagerBase<TData, _details::MultiMap, Manager> {
        //Defining Base as an alias for the template pattern.
        using Base=ManagerBase<TData, _details::MultiMap, ::cafer_core::Manager>;
        //Inheriting base class constructor
        using Base::Base;

    public:

        /**
         * @brief add a data object to the container of Manager
         * @param data the data object to add
         */
        void add(const topic_tools::ShapeShifter& msg)
        {
            Base::_container_mutex.lock();
            Base::_data_set.emplace_back(TData(msg));
            Base::_container_mutex.unlock();
        }

        /**
        * @brief search a precise msg by is identifier
        * @param id identifier of the searched msg
        */
        bool search(const std::string& tag, std::vector<TData>& vect_msg)
        {
            bool res;
            Base::_container_mutex.lock();
            auto it_pair = Base::_data_set.equal_range(tag);
            if(it_pair.first == Base::_data_set.end() && it_pair.second == Base::_data_set.end())
                res = false;
            else{
                res = true;
                for(auto it = it_pair.first; it != it_pair.second; it++)
                    vect_msg.push_back(it->second);
            }
            Base::_container_mutex.unlock();
            return res;
        }

    };

    /**
     * @brief Manager component which use std::map as container.
     */
    template<typename TData>
    using ManagerMap=Manager<TData, _details::Map>;

    /**
     * @brief Manager component which use std::multimap as container with std::string as key.
     */
    template<typename TData>
    using ManagerTagMap=Manager<TData, _details::MultiMap>;

    /**
     * @brief Manager component which use std::deque as container.
     */
    template<typename TData>
    using ManagerQueue=Manager<TData, _details::Queue>;
}

#endif //_MANAGER_HPP
