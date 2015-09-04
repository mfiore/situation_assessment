
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/ogre_helpers/arrow.h>
#include "visualize_situation/area_visual.h"

namespace visualize_areas
{
// BEGIN_TUTORIAL
AreaVisual::AreaVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
	scene_manager_ = scene_manager;
	// Ogre::SceneNode s form a tree, with each node storing the
	// transform (position and orientation) of itself relative to its
	// parent. Ogre does the math of combining those transforms when it
	// is time to render.
	//
	// Here we create a node to store the pose of the Imu's header frame
	// relative to the RViz fixed frame.
	frame_node_ = parent_node->createChildSceneNode();
	// We create the arrow object within the frame node so that we can
	// set its position and direction relative to its header frame.
	
	area_.reset(scene_manager_->createManualObject());
	area_->setDynamic( true );
	frame_node_->attachObject(area_.get());
}
AreaVisual::~AreaVisual()
{
	// Destroy the frame node since we don't need it anymore.
	scene_manager_->destroySceneNode(frame_node_ );
}
void AreaVisual::setMessage(geometry_msgs::Polygon msg, Ogre::ColourValue color, float alpha)
{
	uint32_t num_points = msg.points.size();
	if( num_points > 0 )
	{
	  area_->estimateVertexCount( num_points );
	  area_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );
	  for( uint32_t i=0; i < num_points + 1; ++i )
	  {
        const geometry_msgs::Point32& msg_point = msg.points[ i % num_points ];
	    area_->position( msg_point.x, msg_point.y, msg_point.z );
	    area_->colour(color);
	    // area_->alpha=alpha;
	  }
	  area_->end();
	}

}
// Position and orientation are passed through to the SceneNode.
void AreaVisual::setFramePosition( const Ogre::Vector3& position )
{
	frame_node_->setPosition( position );
}
void AreaVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
	frame_node_->setOrientation( orientation );
}
// END_TUTORIAL
} // end namespace rviz_plugin_tutorials