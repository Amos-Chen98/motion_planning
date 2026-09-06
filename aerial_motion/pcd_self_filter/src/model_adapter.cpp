#include <pcd_self_filter/model_adapter.h>

#include <tinyxml2.h>
#include <urdf/model.h>

#include <map>
#include <stdexcept>

namespace pcd_self_filter
{
std::string normalizedFrame(const std::string& frame)
{
  const size_t first = frame.find_first_not_of('/');
  return first == std::string::npos ? std::string() : frame.substr(first);
}

FilterModel makeFilterModel(const std::string& source, const std::string& tf_prefix)
{
  tinyxml2::XMLDocument document;
  if (document.Parse(source.c_str()) != tinyxml2::XML_SUCCESS)
    throw std::invalid_argument("robot_description is not valid XML");
  auto* robot = document.FirstChildElement("robot");
  if (!robot)
    throw std::invalid_argument("robot_description has no robot element");

  std::string prefix = normalizedFrame(tf_prefix);
  while (!prefix.empty() && prefix.back() == '/')
    prefix.pop_back();
  if (!prefix.empty())
    prefix += '/';

  FilterModel result;
  std::map<std::string, std::string> names;
  for (auto* link = robot->FirstChildElement("link"); link;
       link = link->NextSiblingElement("link"))
  {
    const char* name = link->Attribute("name");
    if (!name || !*name)
      throw std::invalid_argument("URDF link has no name");
    const std::string original(name);
    std::string frame = normalizedFrame(original);
    if (frame.compare(0, prefix.size(), prefix) != 0)
      frame = prefix + frame;
    if (!names.emplace(original, frame).second)
      throw std::invalid_argument("Duplicate URDF link: " + original);
    link->SetAttribute("name", frame.c_str());

    // Only existing collision geometry participates in filtering. Keep links
    // without collisions in the URDF tree, but exclude them from body TF checks.
    if (link->FirstChildElement("collision"))
      result.collision_frames.push_back(frame);
    for (auto* collision = link->FirstChildElement("collision"); collision;
         collision = collision->NextSiblingElement("collision"))
      ++result.collision_count;
  }
  for (auto* joint = robot->FirstChildElement("joint"); joint;
       joint = joint->NextSiblingElement("joint"))
  {
    for (const char* role : {"parent", "child"})
    {
      auto* element = joint->FirstChildElement(role);
      const char* name = element ? element->Attribute("link") : nullptr;
      if (!name || names.find(name) == names.end())
        throw std::invalid_argument("URDF joint references a missing link");
      element->SetAttribute("link", names.at(name).c_str());
    }
  }
  tinyxml2::XMLPrinter printer;
  document.Print(&printer);
  result.xml = printer.CStr();
  urdf::Model checked;
  if (!checked.initString(result.xml))
    throw std::invalid_argument("Adapted robot_description is not a valid URDF tree");
  if (result.collision_count == 0)
    throw std::invalid_argument("Robot model has no collision geometry for filtering");
  return result;
}
}  // namespace pcd_self_filter
