#ifndef MITSUBAGENERATOR_H
#define MITSUBAGENERATOR_H

#include <fstream>
#include <string>
#include <vector>
#include <utility>
#include <sstream>
#include "core/ChMath.h"
#include "xml/rapidxml.hpp"
#include "xml/rapidxml_print.hpp"


struct xml_option {
  xml_option(std::string type_, std::string parameter_, std::string value_)
      : type(type_), parameter(parameter_), value(value_) {}
  std::string type;
  std::string parameter;
  std::string value;
};

class MitsubaGenerator {
 public:
  MitsubaGenerator();
  ~MitsubaGenerator() {}

  void ExportDriver(const std::string& filename);
  // Function creates a node of a specific type
  rapidxml::xml_node<>* CreateNewNode(const std::string& type);
  void AddAttribute(const std::string& name, const std::string& value, rapidxml::xml_node<>* node);
  void AddAttributes(const std::vector<xml_option>& options, rapidxml::xml_node<>* root);
  rapidxml::xml_node<>* CreatePlugin(const std::string& name,
                                     const std::string& type,
                                     const std::vector<xml_option>& options);
  rapidxml::xml_node<>* CreateTransform(const std::string& name);
  rapidxml::xml_node<>* CreateTransform(const ChVector<>& scale,
                                        const ChVector<>& position,
                                        const ChQuaternion<>& rotation);
  rapidxml::xml_node<>* CreateSky(const std::string& scale,
                                  const std::string& hour,
                                  const std::string& turbidity,
                                  const std::string& albedo);
  std::string CreateTriplet(const ChVector<>& v);
  void CreateXYZTriplet(const ChVector<>& v, rapidxml::xml_node<>* node);
  void Translate(const ChVector<>& vector, rapidxml::xml_node<>* root);
  void Scale(const ChVector<>& vector, rapidxml::xml_node<>* root);
  void Rotate(const double angle, const ChVector<>& axis, rapidxml::xml_node<>* root);
  void LookAt(const ChVector<>& origin, const ChVector<>& look_at, const ChVector<>& up, rapidxml::xml_node<>* root);

  void CreateScene(bool add_integrator = true, bool add_sensor = true, bool add_sky = true);
  void Write(const std::string& filename);
  void SetDataFolder(const std::string& folder) { data_folder = folder; }
  void SetRenderFolder(const std::string& folder) { render_folder = folder; }

  void AddShape(const std::string& id,
                const ChVector<>& scale,
                const ChVector<>& position,
                const ChQuaternion<>& rotation);

  void AddSensor(ChVector<> origin, ChVector<> target, ChVector<> up);

  //=========================================================================================================
  //=========================================================================================================
  // Sensor Settings
  ChVector<> camera_target, camera_origin, camera_up;
  float scale, hour, turbidity, albedo;

  //=========================================================================================================
  // Film settings
  unsigned int height, width;
  //=========================================================================================================
  // Output settings
  std::string data_folder, render_folder;
  //=========================================================================================================
  // Options
  //=========================================================================================================

  rapidxml::xml_document<> scene_doc;
  rapidxml::xml_node<>* root_node;
};

#endif
