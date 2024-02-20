
#include <iostream>

#include "scenario.h"
#include "Classproject/mycircle.h"
#include "Classproject/mypline.h"
#include "parametrics/curves/gmpline.h"
#include "Classproject/mysubdiv.h"
#include "Classproject/blendingSpline.h"
#include "Classproject/myBlendingSurface.h"
#include "Classproject/MyModelCurve.h"
#include "parametrics/surfaces/gmpplane.h"
#include "parametrics/surfaces/gmpcylinder.h"

#include "Classproject/curve-test.h"



// hidmanager
#include "hidmanager/defaulthidmanager.h"

// gmlib
#include <scene/light/gmpointlight.h>
#include <scene/sceneobjects/gmpathtrack.h>
#include <scene/sceneobjects/gmpathtrackarrows.h>

// qt
#include <QQuickItem>


template <typename T>
inline
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v) {
  out << v.size() << std::endl;
  for(uint i=0; i<v.size(); i++) out << " " << v[i];
  out << std::endl;
  return out;
}




void Scenario::initializeScenario() {

  // Insert a light
  GMlib::Point<GLfloat,3> init_light_pos( 2.0, 4.0, 10 );
  GMlib::PointLight *light = new GMlib::PointLight(  GMlib::GMcolor::white(), GMlib::GMcolor::white(),
                                                     GMlib::GMcolor::white(), init_light_pos );
  light->setAttenuation(0.8f, 0.002f, 0.0008f);
  this->scene()->insertLight( light, false );

  // Insert Sun
  this->scene()->insertSun();

  // Default camera parameters
  int init_viewport_size = 600;
  GMlib::Point<float,3>  init_cam_pos( 0.0f, 0.0f, 0.0f );
  GMlib::Vector<float,3> init_cam_dir( 0.0f, 1.0f, 0.0f );
  GMlib::Vector<float,3> init_cam_up(  1.0f, 0.0f, 0.0f );

  // Projection cam
  auto proj_rcpair = createRCPair("Projection");
  proj_rcpair.camera->set(init_cam_pos,init_cam_dir,init_cam_up);
  proj_rcpair.camera->setCuttingPlanes( 1.0f, 8000.0f );
  proj_rcpair.camera->rotateGlobal( GMlib::Angle(-45), GMlib::Vector<float,3>( 1.0f, 0.0f, 0.0f ) );
  proj_rcpair.camera->translateGlobal( GMlib::Vector<float,3>( 0.0f, -20.0f, 20.0f ) );
  scene()->insertCamera( proj_rcpair.camera.get() );
  proj_rcpair.renderer->reshape( GMlib::Vector<int,2>(init_viewport_size, init_viewport_size) );


  /***************************************************************************
   *                                                                         *
   * Standar example, including path track and path track arrows             *
   *                                                                         *
   ***************************************************************************/

  GMlib::DVector<GMlib::Vector<float,3>> c_points(10);
  c_points[0] =GMlib::Vector<float,3>(0,0,0);
  c_points[1] =GMlib::Vector<float,3>(1,1,0);
  c_points[2] =GMlib::Vector<float,3>(2,0,0);
  c_points[3] =GMlib::Vector<float,3>(3,1,0);
  c_points[4] =GMlib::Vector<float,3>(4,0,0);
  c_points[5] =GMlib::Vector<float,3>(5,1,0);
  c_points[6] =GMlib::Vector<float,3>(6,0,0);
  c_points[7] =GMlib::Vector<float,3>(7,1,0);
  c_points[8] =GMlib::Vector<float,3>(8,0,0);
  c_points[9] =GMlib::Vector<float,3>(9,1,0);





 //------------------------------B-splines  First Constructor-------------------------------------------------------------

//  auto mybspl = new GMlib::MYPLine<float>(c_points);
//  mybspl->toggleDefaultVisualizer();
//  mybspl->sample(60,0);
//  mybspl->setColor(GMlib::GMcolor::white());
//  this->scene()->insert(mybspl);

//  for(int i=0; i<9; i++){
//      // we use vector instead of point so we can get an


//      auto line = new GMlib::PLine<float>(GMlib::Point<float,3>(c_points[i]),GMlib::Point<float,3>(c_points[i+1]));
//      line->toggleDefaultVisualizer();
//      line->sample(4,0);
//      line->setColor(GMlib::GMcolor::green());
//      this->scene()->insert(line);
//  }





 //----------------- B-spline Second Constructor:-------------------------



//  auto mybspl = new GMlib::MYPLine<float>(c_points , 10);
//  mybspl->toggleDefaultVisualizer();
//  mybspl->sample(60,0);
//  mybspl->setColor(GMlib::GMcolor::green());
//  this->scene()->insert(mybspl);


//  GMlib::DVector<GMlib::Vector<float, 3>> P(50);
//  auto cir = new GMlib::PCircle<float>(10);
//  //GMlib::DVector<GMlib::Vector<float,3>> c_points(50);
//  for (int i = 0; i < 50; ++i) {
//      P[i]= cir->getPosition(cir->getParStart()+i* cir->getParDelta()/49);
//  }

//  auto mybspline = new GMlib::MYPLine<float>(P,8);
//  mybspline->toggleDefaultVisualizer();
//  mybspline->sample(60,0);
//  mybspline->setColor(GMlib::GMcolor::white());
//  mybspline->setLineWidth(5);
//  this->scene()->insert(mybspline);



  GMlib::Material mm(GMlib::GMmaterial::polishedBronze());
  mm.set(45.0);

//  auto ptom = new TestTorus(1.0f, 0.4f, 0.6f);
//  ptom->toggleDefaultVisualizer();
//  ptom->sample(60,60,1,1);
//  this->scene()->insert(ptom);
//  auto ptrack = new GMlib::PathTrack();
//  ptrack->setLineWidth(2);
//  ptom->insert(ptrack);
//  auto ptrack2 = new GMlib::PathTrackArrows();
//  ptrack2->setArrowLength(2);
//  ptom->insert(ptrack2);





//--------------------------Mycircle half----------------------------------


//  auto circle = new GMlib::MYPCircle<float>(10.0f); //this is constructor
//  circle->toggleDefaultVisualizer();
//  circle->sample(60,0);
  //this->scene()->insert(circle);



//--------------------------Mycircle complete----------------------------------


  auto circle2 = new GMlib::MYPCircle<float>(10.0f, 2.0f); //this is constructor
  circle2->toggleDefaultVisualizer();
  circle2->sample(60,0);
 // this->scene()->insert(circle2);







  //---------------MYSUBDIV---------------//



//  GMlib::DVector<GMlib::Vector<float, 3>> c_points(6);
//  c_points[0] = GMlib::Vector<float,3>(0,0,0);
//  c_points[1] = GMlib::Vector<float,3>(0,2,0);
//  c_points[2] = GMlib::Vector<float,3>(1,3,0);
//  c_points[3] = GMlib::Vector<float,3>(2,2,0);
//  c_points[4] = GMlib::Vector<float,3>(2,0,0);
//  c_points[5] = GMlib::Vector<float,3>(1,1,0);
//  auto ptom = new GMlib::MYsubdiv<float>(c_points);
//  ptom->toggleDefaultVisualizer();
//  ptom->sample(3,2);
//  ptom->setColor(GMlib::GMcolor::azure());
//  ptom->setLineWidth(5);
  //this->scene()->insert(ptom);







//-------------------MY_BLENDING_SPLINE------------------------//


//  GMlib::DVector<GMlib::Vector<float, 3>> P(50);
//    auto cir = new GMlib::PCircle<float>(1);
//    //GMlib::DVector<GMlib::Vector<float,3>> c_points(50);
//    for (int i = 0; i < 50; ++i) {
//        P[i]= cir->getPosition(cir->getParStart()+i* cir->getParDelta()/49);
//    }
//  auto Blendsp = new GMlib::MyBlendingSpline<float>(cir,7);
//  Blendsp->toggleDefaultVisualizer();
//  Blendsp->sample(60,0);
//  Blendsp->setColor(GMlib::GMcolor::white());
//  Blendsp->setLineWidth(5);
//  this->scene()->insert(Blendsp);







  //-------------------------MY_BLENDING_SURFACE----------------------//



    GMlib::Point<float,3> p(0,0,0);
    GMlib::Vector<float, 3> V1(1,0,0);
    GMlib::Vector<float, 3> V2(0,1,0);


    auto plane1 = new GMlib::PPlane<float>(p,V1,V2);
    plane1->toggleDefaultVisualizer();
    plane1->sample(40,40,1,1);
    this->scene()->insert(plane1);

    auto myBsurf = new GMlib::myBlendingSurface<float>(plane1,3,3);
    myBsurf->toggleDefaultVisualizer();
    myBsurf->sample(40,40,1,1);
    this->scene()->insert(myBsurf);


//    auto cylindar = new GMlib::PCylinder<float>(2, 2, 4);
//    auto myBlendingSurface = new GMlib::myBlendingSurface<float>(cylindar, 3, 3);
//    myBlendingSurface->toggleDefaultVisualizer();
//    myBlendingSurface->sample(20, 20, 1, 1);
//    this->scene()->insert(myBlendingSurface);






//----------------------------Butterfly model curve---------

//  auto test_new = new GMlib::Curve_test<float>(3);
//  //fish one call with Blending spline curve
//  auto bspline_model = new GMlib::ModelCurve<float>(test_new,8);
//  bspline_model-> toggleDefaultVisualizer();
//  bspline_model-> sample(200,0);
//  bspline_model->setColor(GMlib::GMcolor::white());
//  this->scene()-> insert(bspline_model);
//  bspline_model->setLineWidth(5);




}







void Scenario::cleanupScenario() {

}




void Scenario::callDefferedGL() {

  GMlib::Array< const GMlib::SceneObject*> e_obj;
  this->scene()->getEditedObjects(e_obj);

  for(int i=0; i < e_obj.getSize(); i++)
    if(e_obj(i)->isVisible()) e_obj[i]->replot();
}

