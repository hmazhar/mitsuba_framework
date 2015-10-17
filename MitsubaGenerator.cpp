#include "MitsubaGenerator.h"
#include <chrono/collision/ChCModelBullet.h>

void MitsubaGenerator::CreateNewNode(const std::string& type) {
  xmlTextWriterStartElement(writer, BAD_CAST type.c_str());
}
void MitsubaGenerator::CloseNode() { xmlTextWriterEndElement(writer); }

void MitsubaGenerator::AddAttribute(const std::string& name,
                                    const std::string& value) {
  xmlTextWriterWriteAttribute(writer, BAD_CAST name.c_str(),
                              BAD_CAST value.c_str());
}
void MitsubaGenerator::AddAttributes(const std::vector<xml_option>& options) {
  for (int i = 0; i < options.size(); i++) {
    CreateNewNode(options[i].type.c_str());
    AddAttribute("name", options[i].parameter.c_str());
    AddAttribute("value", options[i].value.c_str());
    CloseNode();
  }
}
void MitsubaGenerator::CreatePlugin(const std::string& name,
                                    const std::string& type,
                                    const std::vector<xml_option>& options) {
  CreateNewNode(name.c_str());
  AddAttribute("type", type.c_str());
  AddAttributes(options);
}
void MitsubaGenerator::CreateTransform(const std::string& name) {
  CreateNewNode("transform");
  AddAttribute("name", name.c_str());
}

void MitsubaGenerator::CreateTransform(const chrono::ChVector<>& scale,
                                       const chrono::ChVector<>& position,
                                       const chrono::ChQuaternion<>& rotation) {
  CreateTransform("toWorld");

  Scale(scale);

  double angle;
  chrono::ChVector<> axis;
  rotation.Q_to_AngAxis(angle, axis);
  angle = angle * 180.0 / chrono::CH_C_PI;
  Rotate(angle, axis);
  Translate(position);
}

std::string MitsubaGenerator::CreateTriplet(const chrono::ChVector<>& v) {
  std::stringstream ss;
  ss << float(v.x) << ", " << float(v.y) << ", " << float(v.z);
  return ss.str();
}
void MitsubaGenerator::CreateXYZTriplet(const chrono::ChVector<>& v) {
  AddAttribute("x", std::to_string(float(v.x)));
  AddAttribute("y", std::to_string(float(v.y)));
  AddAttribute("z", std::to_string(float(v.z)));
}

void MitsubaGenerator::Translate(const chrono::ChVector<>& vector) {
  CreateNewNode("translate");
  CreateXYZTriplet(vector);

  CloseNode();
}
void MitsubaGenerator::Scale(const chrono::ChVector<>& vector) {
  CreateNewNode("scale");
  CreateXYZTriplet(vector);
  CloseNode();
}
void MitsubaGenerator::Rotate(const double angle,
                              const chrono::ChVector<>& axis) {
  CreateNewNode("rotate");

  CreateXYZTriplet(axis);
  AddAttribute("angle", std::to_string(angle));
  CloseNode();
}

void MitsubaGenerator::LookAt(const chrono::ChVector<>& origin,
                              const chrono::ChVector<>& look_at,
                              const chrono::ChVector<>& up) {
  CreateNewNode("lookAt");

  AddAttribute("origin", CreateTriplet(origin).c_str());
  AddAttribute("target", CreateTriplet(look_at).c_str());
  AddAttribute("up", CreateTriplet(up).c_str());

  CloseNode();
}

void MitsubaGenerator::CreateSky(const std::string& scale,
                                 const std::string& hour,
                                 const std::string& turbidity,
                                 const std::string& albedo) {
  std::vector<xml_option> sky_options;
  sky_options.push_back(xml_option("float", "scale", scale));
  sky_options.push_back(xml_option("float", "hour", hour));
  sky_options.push_back(xml_option("float", "turbidity", turbidity));
  sky_options.push_back(xml_option("spectrum", "albedo", albedo));
  CreatePlugin("emitter", "sky", sky_options);
}

void MitsubaGenerator::CreateScene(bool add_integrator, bool add_sensor,
                                   bool add_sky) {
  CreateNewNode("include");
  AddAttribute("filename", "geometry.xml");
  CloseNode();

  if (add_integrator) {
    /////Integrator
    std::vector<xml_option> integrator_options;
    integrator_options.push_back(xml_option("boolean", "hideEmitters", "true"));
    integrator_options.push_back(xml_option("integer", "maxDepth", "10"));
    integrator_options.push_back(xml_option("integer", "rrDepth", "10"));
    CreatePlugin("integrator", "path", integrator_options);
    CloseNode();
  }
  if (add_sensor) {
    AddSensor(camera_origin, camera_target, camera_up);
  }
  if (add_sky) {
    // Add the Sky emitter

    CreateSky(std::to_string(scale), std::to_string(hour),
              std::to_string(turbidity), std::to_string(albedo));
    CloseNode();
  }
  CreateNewNode("include");
  AddAttribute("filename", "$frame.xml");
  CloseNode();
}

MitsubaGenerator::MitsubaGenerator(const std::string& filename) {
  // create xml header
  writer = xmlNewTextWriterFilename(filename.c_str(), 0);
  xmlTextWriterStartDocument(writer, NULL, "utf-8", NULL);

  xmlTextWriterStartElement(writer, BAD_CAST "scene");
  xmlTextWriterWriteAttribute(writer, BAD_CAST "version", BAD_CAST "0.5.0");

  width = 1920;
  height = 1080;

  camera_target = chrono::ChVector<>(0, -3, 0);
  camera_origin = chrono::ChVector<>(0, -2, 20);
  camera_up = chrono::ChVector<>(0, 1, 0);

  scale = 3;
  hour = 12;
  turbidity = 10;
  albedo = .15;
}

void MitsubaGenerator::AddShape(const std::string& id,
                                const chrono::ChVector<>& scale,
                                const chrono::ChVector<>& position,
                                const chrono::ChQuaternion<>& rotation) {
  CreateNewNode("shape");  // Create the root integrator node <integrator ...
  AddAttribute("type", "instance");
  CreateNewNode("ref");
  AddAttribute("id", id);
  CloseNode();
  CreateTransform(scale, position, rotation);
  CloseNode();

  CloseNode();
}

void MitsubaGenerator::AddCompleteShape(
    const std::string& id, const std::string& material_type,
    const chrono::ChVector<>& color, const chrono::ChVector<>& scale,
    const chrono::ChVector<>& position,
    const chrono::ChQuaternion<>& rotation) {
  CreateNewNode("shape");
  AddAttribute("type", id);
  CreateTransform(scale, position, rotation);
  CloseNode();

  CreateNewNode("bsdf");
  AddAttribute("type", "diffuse");
  CreateNewNode("srgb");
  AddAttribute("name", "reflectance");
  AddAttribute("value", CreateTriplet(color).c_str());
  CloseNode();
  CloseNode();
  CloseNode();
}

void MitsubaGenerator::AddSimpleShape(const int type,
                                      const chrono::ChVector<>& scale,
                                      const chrono::ChVector<>& position,
                                      const chrono::ChQuaternion<>& rotation) {
  switch (type) {
    case chrono::collision::SPHERE:
      AddShape("sphere", scale, position, rotation);
      break;
    case chrono::collision::ELLIPSOID:
      AddShape("sphere", scale, position, rotation);
      break;
    case chrono::collision::BOX:
      AddShape("cube", scale, position, rotation);
      break;
    case chrono::collision::CYLINDER:
      AddShape("cylinder", scale, position, rotation);
      break;
    case chrono::collision::CONE:
      AddShape("cone", scale, position, rotation);
      break;
    case chrono::collision::CAPSULE:
      AddShape("sphere", chrono::ChVector<>(scale.x),
               position + rotation.Rotate(chrono::ChVector<>(0, scale.y, 0)),
               chrono::QUNIT);
      AddShape("cylinder", scale, position, rotation);
      AddShape("sphere", chrono::ChVector<>(scale.x),
               position + rotation.Rotate(chrono::ChVector<>(0, -scale.y, 0)),
               chrono::QUNIT);
      break;
    default:
      // type is -1 (triangle mesh)
      break;
  }
}

void MitsubaGenerator::AddSensor(chrono::ChVector<> origin,
                                 chrono::ChVector<> target,
                                 chrono::ChVector<> up) {
  /////Sensor
  std::vector<xml_option> sensor_options;
  sensor_options.push_back(xml_option("string", "fovAxis", "smaller"));
  sensor_options.push_back(xml_option("float", "fov", "45"));
  CreatePlugin("sensor", "perspective", sensor_options);
  CreateTransform("toWorld");
  LookAt(origin, target, up);
  CloseNode();

  /////Sampler
  std::vector<xml_option> sampler_options;
  sampler_options.push_back(xml_option("integer", "sampleCount", "256"));
  sampler_options.push_back(xml_option("integer", "scramble", "-1"));
  CreatePlugin("sampler", "sobol", sampler_options);
  CloseNode();

  /////Film
  std::vector<xml_option> film_options;
  film_options.push_back(xml_option("string", "pixelFormat", "rgba"));
  film_options.push_back(xml_option("integer", "width", std::to_string(width)));
  film_options.push_back(
      xml_option("integer", "height", std::to_string(height)));
  film_options.push_back(xml_option("boolean", "banner", "false"));
  CreatePlugin("film", "ldrfilm", film_options);
  CloseNode();
  CloseNode();
}

void MitsubaGenerator::Write() {
  xmlTextWriterEndDocument(writer);
  xmlFreeTextWriter(writer);
}

void MitsubaGenerator::ExportDriver(const std::string& filename) {}
