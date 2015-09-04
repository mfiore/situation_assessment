 #ifndef RVIZ_VISUALIZE_AREAS_H
 #define RVIZ_VISUALIZE_AREAS_H
 
 #include <situation_assessment_msgs/AreaList.h>
 #include <vector>

 #include "rviz/message_filter_display.h"
 

 namespace Ogre
 {
 class ManualObject;
 }
 
namespace rviz { 
 class ColorProperty;
 class FloatProperty;
}

 namespace visualize_areas {
 
 class VisualizeAreas: public rviz::MessageFilterDisplay<situation_assessment_msgs::AreaList>
 {
 Q_OBJECT
 public:
   VisualizeAreas();
   virtual ~VisualizeAreas();
 
   virtual void onInitialize();
 
   virtual void reset();
 
 protected:
   virtual void processMessage( const situation_assessment_msgs::AreaList::ConstPtr& msg );
 
   std::vector<Ogre::ManualObject*> manual_object_;
 
   rviz::ColorProperty* color_property_;
   rviz::FloatProperty* alpha_property_;
 };
 
 }
 #endif /* RVIZ_VISUALIZE_AREAS_H */
 