#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreBillboardSet.h>
 
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/parse_color.h"
#include "rviz/validate_floats.h"
 
#include "visualize_situation/visualize_areas.h"
 

namespace visualize_areas { 
 
 VisualizeAreas::VisualizeAreas()
 {
   color_property_ = new rviz::ColorProperty( "Color", QColor( 25, 255, 0 ),
                                        "Color to draw the polygons.", this, SLOT( queueRender() ));
   alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                        "Amount of transparency to apply to the polygons.", this, SLOT( queueRender() ));
   alpha_property_->setMin( 0 );
   alpha_property_->setMax( 1 );
 }
 
 VisualizeAreas::~VisualizeAreas()
 {
   if ( initialized() )
   {
   	 for (int i=0; i< manual_object_.size(); i++) {
    	 scene_manager_->destroyManualObject(manual_object_[i]);
 	}
   }
 }
 
 void VisualizeAreas::onInitialize()
 {
   MFDClass::onInitialize();
   Ogre::ManualObject* an_object;
   an_object = scene_manager_->createManualObject();
   an_object->setDynamic( true );
   scene_node_->attachObject( an_object );
   manual_object_.push_back(an_object);
 }
 
 void VisualizeAreas::reset()
 {
   MFDClass::reset();
   for (int i=0; i<manual_object_.size();i++) {
   	 manual_object_[i]->clear();
	}
	manual_object_.clear();
 }
 // bool validateFloats( const geometry_msgs::PolygonStamped& msg )
 // {
 //   return validateFloats(msg.polygon.points);
 // }
 
 void VisualizeAreas::processMessage(const situation_assessment_msgs::AreaList::ConstPtr& msg)
 {
   // if( !validateFloats( *msg ))
   // {
   //   setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
   //   return;
   // }
 
   Ogre::Vector3 position;
   Ogre::Quaternion orientation;
   if( !context_->getFrameManager()->getTransform( msg->header, position, orientation))
   {
     ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
   }
 
   scene_node_->setPosition( position );
   scene_node_->setOrientation( orientation );
 
   for (int i=0; i<manual_object_.size(); i++) {
   		manual_object_[i]->clear();
   }
   manual_object_.clear();
 
   Ogre::ColourValue color = rviz::qtToOgre( color_property_->getColor() );
   color.a = alpha_property_->getFloat();
   // TODO: this does not actually support alpha as-is.  The
   // "BaseWhiteNoLighting" material ends up ignoring the alpha
   // component of the color values we set at each point.  Need to make
   // a material and do the whole setSceneBlending() rigamarole.
   for (int i=0; i<msg->areas.size(); i++) {
   	uint32_t num_points = msg->areas[i].points.size();
   	if( num_points > 0 )
   	{
   	  Ogre::ManualObject *an_object=scene_manager_->createManualObject();
        an_object->setDynamic( true );
        scene_node_->attachObject( an_object );
   	  an_object->estimateVertexCount( num_points );
   	  an_object->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP );
        for( uint32_t i=0; i < num_points + 1; ++i )
        {
          const geometry_msgs::Point32& msg_point = msg->areas[i].points[ i % num_points ];
   	    an_object->position( msg_point.x, msg_point.y, msg_point.z );
   	    an_object->colour( color );
   	  }
   	  an_object->end();
   	  manual_object_.push_back(an_object);
   	}
   }

 }
} 
 
 #include <pluginlib/class_list_macros.h>
 PLUGINLIB_EXPORT_CLASS( visualize_areas::VisualizeAreas, rviz::Display )
