#include <iostream>
#include <fstream>
#include <assert.h>
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
using namespace std;
bool isCopy=false;
// void OctomapCallBack(const octomap_msgs::Octomap &octo_map){
//     if (!isCopy){
//         octomap::OcTree* sub_octree = new octomap::OcTree(octo_map.resolution);    
//         octomap_msgs::readTree(sub_octree, octo_map);
//         sub_octree->writeBinary( "data/sample.bt" );
//         cout<<"done."<<endl;
//         isCopy=true;
//     }
    
    
// }
int main( int argc, char** argv )
{
    // ros::init(argc,argv,"OctomapGene");
    // ros::NodeHandle nh;
    // ros::Subscriber octoSub = nh.subscribe<octomap_msgs::Octomap>("/octomap_full",10,OctomapCallBack);
    // ros::spin();
    octomap::OcTree tree( 0.1 );
    for(int i=0;i<10;i++){
        for(int j=0;j<10;j++){
            for(int k=0;k<10;k++){
                float x=i*0.1+0.3;
                float y=j*0.1+0.3;
                float z=-0.8+k*0.1;
                tree.updateNode(octomap::point3d(x,y,z),true);
            }
        }
    }
    
    // 更新octomap
    string outputFile="data/sample.bt";
    fstream outfile;
    outfile.open(outputFile,ios::out);
    tree.updateInnerOccupancy();
    // 存储octomap
    tree.writeBinary(outputFile);
    cout<<"done."<<endl;
    outfile.close();
    return 0;
}