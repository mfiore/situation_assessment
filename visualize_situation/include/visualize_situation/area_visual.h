#ifndef AREA_VISUAL_H
#define AREA_VISUAL_H
#include <geometry_msgs/Polygon.h>

namespace Ogre
{
	class Vector3;
	class Quaternion;
	class ManualObject;
}
namespace rviz { 
 class ColorProperty;
 class FloatProperty;
}

namespace visualize_areas
{
// BEGIN_TUTORIAL
// Declare the visual class for this display.
//
// Each instance of ImuVisual represents the visualization of a single
// sensor_msgs::Imu message. Currently it just shows an arrow with
// the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.
	class AreaVisual
	{
	public:
		// Constructor. Creates the visual stuff and puts it into the
		// scene, but in an unconfigured state.
		AreaVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
		// Destructor. Removes the visual stuff from the scene.
		virtual ~AreaVisual();
		// Configure the visual to show the data in the message.
		void setMessage(geometry_msgs::Polygon msg, Ogre::ColourValue color, float alpha );
		// Set the pose of the coordinate frame the message refers to.
		// These could be done inside setMessage(), but that would require
		// calls to FrameManager and error handling inside setMessage(),
		// which doesn't seem as clean. This way ImuVisual is only
		// responsible for visualization.
		void setFramePosition( const Ogre::Vector3& position );
		void setFrameOrientation( const Ogre::Quaternion& orientation );
		// Set the color and alpha of the visual, which are user-editable
		// parameters and therefore don't come from the Imu message.
	private:
		// The object implementing the actual arrow shape
		boost::shared_ptr<Ogre::ManualObject> area_;
		// A SceneNode whose pose is set to match the coordinate frame of
		// the Imu message header.
		Ogre::SceneNode* frame_node_;
		// The SceneManager, kept here only so the destructor can ask it to
		// destroy the ``frame_node_``.
		Ogre::SceneManager* scene_manager_;
	};
	// END_TUTORIAL
	} // end namespace rviz_plugin_tutorials
#endif // AREA_VISUAL_H