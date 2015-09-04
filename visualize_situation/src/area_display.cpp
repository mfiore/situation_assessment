
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include "visualize_situation/area_visual.h"
#include "visualize_situation/area_display.h"

namespace visualize_areas
{
// BEGIN_TUTORIAL
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
AreaDisplay::AreaDisplay()
{
	color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
	"Color to draw the acceleration arrows.",
	this, SLOT( updateColorAndAlpha() ));
	alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
	"0 is fully transparent, 1.0 is fully opaque.",
	this, SLOT( updateColorAndAlpha() ));
}
// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function. This is where we
// instantiate all the workings of the class. We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
// Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void AreaDisplay::onInitialize()
{
	MFDClass::onInitialize();
}
AreaDisplay::~AreaDisplay()
{
}
// Clear the visuals by deleting their objects.
void AreaDisplay::reset()
{
	MFDClass::reset();
	visuals_.clear();
}
// Set the current color and alpha values for each visual.
void AreaDisplay::updateColorAndAlpha()
{
	// float alpha = alpha_property_->getFloat();
	// Ogre::ColourValue color = color_property_->getOgreColor();
	// for( size_t i = 0; i < visuals_.size(); i++ )
	// {
	// visuals_[ i ]->setColor( color.r, color.g, color.b, alpha );
	// }
}
// This is our callback to handle an incoming message.
void AreaDisplay::processMessage( const situation_assessment_msgs::AreaList::ConstPtr& msg )
{
// Here we call the rviz::FrameManager to get the transform from the
// fixed frame to the frame in the header of this Imu message. If
// it fails, we can't do anything else so we return.
	// for (int i=0; i<visuals_.size();i++){
		// visuals_[i].clear();
	// }
	visuals_.erase(visuals_.begin(),visuals_.end());

	Ogre::Quaternion orientation;
	Ogre::Vector3 position;
	if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
	msg->header.stamp,
	position, orientation ))
	{
		ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
		msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
		return;
	}
	for (int i=0; i<msg->areas.size();i++) {
		boost::shared_ptr<AreaVisual> visual;
		visual.reset(new AreaVisual(context_->getSceneManager(), scene_node_ ));
		// Now set or update the contents of the chosen visual.
		Ogre::ColourValue color = color_property_->getOgreColor();
		float alpha = alpha_property_->getFloat();

		geometry_msgs::Polygon poly_msg=msg->areas[i];
		visual->setMessage( poly_msg,color,alpha );
		visual->setFramePosition( position );
		visual->setFrameOrientation( orientation );
		// visual->setColor( color.r, color.g, color.b, alpha );
		visuals_.push_back(visual);
	}
}

} // end namespace rviz_plugin_tutorials
// Tell pluginlib about this class. It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(visualize_areas::AreaDisplay,rviz::Display )
// END_TUTORIAL


