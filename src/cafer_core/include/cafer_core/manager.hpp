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

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>

#include <boost/functional/hash.hpp>
#include <boost/random.hpp>
#include <unordered_map>

#include <cafer_core/component.hpp>

namespace cafer_core {


/**
 *@brief class Manager<Msg>
 * A data manager for handle the messages (like images, features or policies) used in cafer.
 */
template<typename Msg>
class Manager{
private:


    typedef std::unordered_map<u_int32_t, Msg> data_t;
    data_t _data_set;

    long int _id;
    std::string _name;
    std::string _description;
    std::string _type;

    std::mt19937 _gen;

    boost::shared_ptr<ros::Publisher> _publisher;
//    boost::shared_ptr<ros::ServiceServer> _start_to_publish;
//    boost::shared_ptr<ros::ServiceServer> _search_service;
    boost::shared_ptr<ros::Subscriber> _subcriber;

public:

    /**
     * @brief The io enum type of traitement to do with incoming data.
     */
    enum io{
        ADD
    };

    /**
     * @brief Manager constructor
     * @param type specify which of data manager (and not the type of data)
     * @param name of the manager
     * @param description a short description of the manager (optionnal)
     */
    Manager(std::string type, std::string name, std::string description = "")
        : _type(type), _name(name), _description(description)
    {
        //init random number generator for random access
        std::seed_seq seed = {std::time(0)};
        _gen.seed(seed);


//        //init services
//        std::stringstream sstream;
//        sstream << _name << "_start_to_publish";
//        _start_to_publish->reset(new ros::ServiceServer(ros_nh->advertiseService(sstream.str(),start_to_publish)));

//        std::stringstream sstream2;
//        sstream2 << _name << "_search_service";
//        _search_service->reset(new ros::ServiceServer(ros_nh->advertiseService(sstream2.str(),search_service)));

    }

    void disconnect_from_ros(){
//        _publisher.reset();
//        _server.reset();
    }

    bool is_initialized(){
        return true;
    }

    void update(){

    }

    /**
     * @brief add a msg to the container of Manager
     * @param msg the message to add
     */
    void add(const Msg& msg){
        _data_set.emplace(msg.header.seq,msg);
    }

    /**
     * @brief add_cb callback function for ListenTo
     * @param msg
     */
    void add_cb(const boost::shared_ptr<Msg>& msg){
        add(*msg);
    }

    /**
     * @brief get random access to any data
     * @return a message
     */
    Msg get() {
        std::uniform_int_distribution<> dist(0.,_data_set.size()-1);
        auto random_it = std::next(std::begin(_data_set),dist(_gen));
        Msg res = random_it->second;
        return res;
    }

    /**
     * @brief remove a element of the container of the manager
     * @param id identifier of the msg to remove
     * @return should be 1 in success case and 0 otherwise.
     */
    size_t remove(const u_int32_t& h){
        return _data_set.erase(h);
    }

    /**
     * @brief search a precise msg by is identifier
     * @param id identifier of the searched msg
     */
    Msg search(const u_int32_t& id){
        return _data_set.find(id)->second;
    }

//    void search_service()

    /**
     * @brief data_size
     * @return the number of data.
     */
    size_t data_size(){
        return _data_set.size();
    }

//    /**
//     * @brief AskTo Call a specific service
//     * @param md
//     */
//    void ask_to(const std::string& name){

//        std::stringstream sstream;
//        sstream << name << "_search_service";
//        ros::ServiceClient client = ros_nh->serviceClient<>(sstream.str());


//        while(!client.call(srv)){std::cout << "Wait for service " << service_name << std::endl;}
//        //DO SOMETHING !!!!
//    }

    /**
     * @brief ListenTo connect to a specific topic and listen to him
     * @param topic
     */
    void ListenTo(const std::string& topic, io type_io = ADD){
        if(type_io == ADD)
             _subcriber.reset(new ros::Subscriber(ros_nh->subscribe(topic,10,&Manager::add_cb,this)));
    }

//    void start_to_publish(cafer_core::start_to_talk::Request& req,
//                          cafer_core::start_to_talk::Response& res){

//        if(!req.want_to_listen)
//            return;

//        std::stringstream sstream;
//        sstream << _name << "_talker";
//        _publisher.reset(ros_nh->advertise(sstream.str(),talk_cb));
//        res.start_to_talk = true;
//        //do parallisation
//    }


};

}

#endif //_MANAGER_HPP
