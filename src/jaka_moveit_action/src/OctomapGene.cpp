#include <iostream>
#include <fstream>
#include <assert.h>
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
using namespace std;
// bool isCopy=false;
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
    ros::init(argc,argv,"OctomapGene");
    ros::NodeHandle nh;
    ros::Publisher oc_pub= nh.advertise<octomap_msgs::Octomap>("/octomap_full",10);
    ros::Rate looprate(1);
    octomap::OcTree tree( 0.05 );
    for(int i=0;i<4;i++){
        for(int j=0;j<2;j++){
            for(int k=0;k<4;k++){
                float x=-i*0.025-0.4;
                float z=-0.45+k*0.025;
                float y=-0.2+j*0.025;
                tree.updateNode(octomap::point3d(x,y,z),true);
            }

        }
            
    }
    // octomap::OcTreeNode* obNode=tree.search(octomap::point3d(0.41,-0.22,-0.4));
    // if(obNode){std::cout<<"prob"<<obNode->getOccupancy()<<std::endl;}
    // else{
    //             std::cout<<"Not find"<<std::endl;
    //         }
    tree.updateInnerOccupancy();
    //octomap::OcTree::tree_iterator it = tree.begin_tree();

    octomap_msgs::Octomap msg;

    octomap_msgs::fullMapToMsg<octomap::OcTree>(tree,msg);
    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();
    // octomap::AbstractOcTree* sTree=octomap_msgs::fullMsgToMap(msg);
    // octomap::OcTree* sub_octree = new octomap::OcTree(msg.resolution);    
    // // std::stringstream datastream;
    // // datastream.write((const char*) &msg.data[0], msg.data.size());
    // // sub_octree->readBinary(datastream);
    // sub_octree = dynamic_cast<octomap::OcTree*>(sTree);
    // cout<<sub_octree->size()<<endl;

    // //octomap_msgs::readTree<octomap::OcTree>(sub_octree, msg);
    // for (octomap::OcTree::leaf_iterator it = sub_octree->begin_leafs(),end = sub_octree->end_leafs(); it != end; ++it) {
    //         octomap::point3d center= it.getCoordinate();   
    //         double size=it.getSize();
    //         cout<<center.x()<<' '<<center.y()<<' '<<center.z()<<' '<<size<<endl;
    // }
    while(ros::ok()){
        oc_pub.publish(msg);
        
        looprate.sleep();
    }
        

    
    // 更新octomap

    return 0;
}