#ifndef AREA_DISPLAY_H
#define AREA_DISPLAY_H

#include <situation_assessment_msgs/AreaList.h>
#include <rviz/message_filter_display.h>
#include <vector>

namespace Ogre
{
	class SceneNode;
}
namespace rviz
{
	class ColorProperty;
	class FloatProperty;
}
// All the source in this plugin is in its own namespace. This is not
// required but is good practice.
namespace visualize_areas
{
	class AreaVisual;
	// BEGIN_TUTORIAL
	// Here we declare our new subclass of rviz::Display. Every display
	// which can be listed in the "Displays" panel is a subclass of
	// rviz::Display.
	//
	class AreaDisplay: public rviz::MessageFilterDisplay<situation_assessment_msgs::AreaList>
	{
	Q_OBJECT
	public:
		// Constructor. pluginlib::ClassLoader creates instances by calling
		// the default constructor, so make sure you have one.
		AreaDisplay();
		virtual ~AreaDisplay();
		// Overrides of protected virtual functions from Display. As much
		// as possible, when Displays are not enabled, they should not be
		// subscribed to incoming data and should not show anything in the
		// 3D view. These functions are where these connections are made
		// and broken.
	protected:
		virtual void onInitialize();
		// A helper to clear this display back to the initial state.
		virtual void reset();
		// These Qt slots get connected to signals indicating changes in the user-editable properties.
		private Q_SLOTS:
			void updateColorAndAlpha();
	private:
		// Function to handle an incoming ROS message.
		void processMessage( const situation_assessment_msgs::AreaList::ConstPtr& msg );
		// Storage for the list of visuals. It is a circular buffer where
		// data gets popped from the front (oldest) and pushed to the back (newest)
		std::vector<boost::shared_ptr<AreaVisual> > visuals_;
		// User-editable property variables.
		rviz::ColorProperty* color_property_;
		rviz::FloatProperty* alpha_property_;
	};
} // end namespace rviz_plugin_tutorials
#endif // AREA_DISPLAY_H
// %EndTag(FULL_SOURCE)%