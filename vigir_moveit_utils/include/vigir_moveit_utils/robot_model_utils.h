//=================================================================================================
// Copyright (c) 2013, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef ROBOT_MODEL_UTILS_H__
#define ROBOT_MODEL_UTILS_H__

#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/robot_model.h>

namespace robot_model_utils{

  void addChildrenToStringVector(const moveit::core::JointModel* joint_model, std::vector<std::string>& link_strings)
  {
    const moveit::core::LinkModel* link_model = joint_model->getChildLinkModel();

    if (link_model->getShapes().size() >0){
      link_strings.push_back(link_model->getName());
    }

    const std::vector<const moveit::core::JointModel*> joint_models = link_model->getChildJointModels();

    for (size_t i = 0; i < joint_models.size(); ++i){
      addChildrenToStringVector(joint_models[i], link_strings);
    }
  }


  std::vector<std::string> getSubLinks(const moveit::core::RobotModel& robot_model , const std::string& link_name, bool debug = false){
    const moveit::core::LinkModel* root_model = robot_model.getLinkModel(link_name);

    std::vector<std::string> link_vector;
    if (root_model->getChildJointModels().size() > 0){
      const std::vector<const moveit::core::JointModel*> joint_models = root_model->getChildJointModels();

      for (size_t i = 0; i < joint_models.size(); ++i){
        addChildrenToStringVector(joint_models[i], link_vector);
      }

    }

    if (debug){
      for (int i = 0; i < link_vector.size(); ++i){
        std::cout << link_vector[i] << "\n";
      }
    }


    return link_vector;
  }
    

 

}

#endif
