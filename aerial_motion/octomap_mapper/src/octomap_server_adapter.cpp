#include <octomap_server/OctomapServer.h>

namespace
{
class QuietOcTree : public octomap::OcTree
{
public:
  explicit QuietOcTree(const octomap::OcTree& tree) : octomap::OcTree(tree) {}

  std::ostream& writeBinaryData(std::ostream& stream) const override
  {
    // Use OctoMap's native encoder without writeBinaryData's per-scan debug
    // print. That print can be compiled into liboctomap and bypasses rosconsole.
    if (root)
      writeBinaryNode(stream, root);
    return stream;
  }
};
}

// Upstream 0.6.8 skips full-map publication for an empty tree, including reset.
// Keep its mapping, services and visualization while completing the full-map
// topic and silencing the binary encoder's debug output.
class OctomapServerAdapter : public octomap_server::OctomapServer
{
public:
  OctomapServerAdapter()
  {
    // Preserve the sensor model and resolution configured by the base server.
    auto* tree = new QuietOcTree(*m_octree);
    delete m_octree;
    m_octree = tree;
  }

protected:
  void publishAll(const ros::Time& stamp = ros::Time::now()) override
  {
    if (m_octree->size() <= 1)
      publishFullOctoMap(stamp);
    else
      OctomapServer::publishAll(stamp);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_server");
  OctomapServerAdapter server;
  std_srvs::Empty::Request request;
  std_srvs::Empty::Response response;
  server.resetSrv(request, response);
  ROS_INFO("Official OctoMap server ready; full empty maps are published on startup/reset. MapBound limits planning only.");
  ros::spin();
  return 0;
}
