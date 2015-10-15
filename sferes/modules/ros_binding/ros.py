#! /usr/bin/env python
# encoding: utf-8

# This file is a part of the CAFER framework developped within
# the DREAM project (http://www.robotsthatdream.eu/).
# Copyright 2015, ISIR / Universite Pierre et Marie Curie (UPMC)
# Main contributor(s): 
#   * Antoins Cully, antoine.cully@isir.upmc.fr
#   * Stephane Doncieux, stephane.doncieux@isir.upmc.fr
#
#
# This experiment allows to generate neural networks for simple
# navigation tasks (obstacle avoidance and maze navigation).
#
# This software is governed by the CeCILL license under French law
# and abiding by the rules of distribution of free software.  You
# can use, modify and/ or redistribute the software under the terms
# of the CeCILL license as circulated by CEA, CNRS and INRIA at the
# following URL "http://www.cecill.info".
# 
# As a counterpart to the access to the source code and rights to
# copy, modify and redistribute granted by the license, users are
# provided only with a limited warranty and the software's author,
# the holder of the economic rights, and the successive licensors
# have only limited liability.
#
# In this respect, the user's attention is drawn to the risks
# associated with loading, using, modifying and/or developing or
# reproducing the software by the user in light of its specific
# status of free software, that may mean that it is complicated to
# manipulate, and that also therefore means that it is reserved for
# developers and experienced professionals having in-depth computer
# knowledge. Users are therefore encouraged to load and test the
# software's suitability as regards their requirements in conditions
# enabling the security of their systems and/or data to be ensured
# and, more generally, to use and operate it in the same conditions
# as regards security.
#
# The fact that you are presently reading this means that you have
# had knowledge of the CeCILL license and that you accept its terms.


import os, glob, types
import Options, Configure, config_c
import commands
def detect_ros(conf,pack):
    env = conf.env
    opt = Options.options
    ret = conf.find_program('rospack')
    conf.check_message_1('Checking for ROS')
    if not ret:
        conf.check_message_2('not found', 'RED')
        return 0
    rosversion = commands.getoutput('rosversion -d')
    home_directory = commands.getoutput('echo ~')
    conf.check_message_2('ok distribution='+rosversion)
    config_c.parse_flags('-I/opt/ros/'+rosversion+'/include -L/opt/ros/'+rosversion+'/lib -lroscpp -lrosconsole -lxmlrpcpp -lroscpp_serialization -lrostime -lpthread -lcpp_common -L'+home_directory+'/git/cafer/devel/lib -lcafer_client ', 'ROS', env)
    for obj in pack:
        res = commands.getoutput('rospack depends '+ obj)    	
        config_c.parse_flags(res, 'ROS', env)
        print res

 
    return 1

def detect(conf):
    return detect_ros(conf,['roscpp'])

def set_options(opt):
    pass
