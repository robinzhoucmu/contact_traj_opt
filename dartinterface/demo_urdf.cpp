#include "dart-interface.h"

#include <string>
using namespace dart::dynamics;
using namespace dart::simulation;

SkeletonPtr loadUrdf(std::string urdf_file_name) {
  dart::utils::DartLoader loader;
  dart::common::Uri uri = dart::common::Uri::createFromString(urdf_file_name);
  std::cout << uri.toString() << std::endl;
  //SkeletonPtr sk = loader.parseSkeleton(urdf_file_name);
  SkeletonPtr sk = loader.parseSkeleton(uri);
  return sk;
}

int main(int argc, char* argv[]) {
  WorldPtr world = std::make_shared<World>();
  
  std::string ground_urdf = "../testdata/ground.urdf"; 
  SkeletonPtr sk_ground = loadUrdf(ground_urdf);
  world->addSkeleton(sk_ground);
  
  std::string block_urdf = "../testdata/block.urdf";
  SkeletonPtr sk_block = loadUrdf(block_urdf);
  world->addSkeleton(sk_block);
  
}
