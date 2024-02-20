/**********************************************************************************
**
** Copyright (C) 1994 Narvik University College
** Contact: GMlib Online Portal at http://episteme.hin.no
**
** This file is part of the Geometric Modeling Library, GMlib.
**
** GMlib is free software: you can redistribute it and/or modify
** it under the terms of the GNU Lesser General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** GMlib is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU Lesser General Public License for more details.
**
** You should have received a copy of the GNU Lesser General Public License
** along with GMlib.  If not, see <http://www.gnu.org/licenses/>.
**
**********************************************************************************/




#ifndef Classproject_MYPCircle_H
#define Classproject_MYPCircle_H


#include "../../gmlib/modules/parametrics/curves/gmpcircle.h"


namespace GMlib {


  template <typename T>
  class MYPCircle : public GMlib::PCurve<T,3> {
    GM_SCENEOBJECT(MYPCircle)

  public:
    MYPCircle( T radius = T(20) );
    MYPCircle( T radius, T circle_size );
//    MYPCircle( T radius = T(20), float circle_size = (1.0f) );


    MYPCircle( const MYPCircle<T>& copy );
    virtual ~MYPCircle();

    // Public local functions
    T               getRadius(int i=1) const;
    void            setRadius( T radius = T(20) );


    //****************************************
    //****** Virtual public functions   ******
    //****************************************

    // from PCurve
    bool            isClosed() const override;

  protected:
    // Virtual functions from PCurve, which have to be implemented locally
    void            eval(T t, int d, bool l) const override;
    T               getStartP() const override;
    T               getEndP()   const override;

    void            computeSurroundingSphere( const std::vector<DVector<Vector<T,3>>>& /*p*/, Sphere<T,3>& s ) const override;

    // Protected data for the curve
    T               _rx;
    T               _ry;
    T               _size;

  }; // END class MYPCircle

} // END namepace GMlib

// Include MYPCircle class function implementations
/**********************************************************************************
**
** Copyright (C) 1994 - 2016 University of Tromsø - The Arctic University of Norway
** Contact: GMlib Online Portal at https://source.uit.no/gmlib/gmlib/wikis/home
**
** This file is part of the Geometric Modeling Library, GMlib.
**
** GMlib is free software: you can redistribute it and/or modify
** it under the terms of the GNU Lesser General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** GMlib is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU Lesser General Public License for more details.
**
** You should have received a copy of the GNU Lesser General Public License
** along with GMlib.  If not, see <http://www.gnu.org/licenses/>.
**
**********************************************************************************/





namespace GMlib {


//*****************************************
// Constructors and destructor           **
//*****************************************

/*! MYPCircle<T>::MYPCircle( T radius )
 *  Default constructor, to make a circle with the radius = radius.
 *
 *  \param[in] radius      (default 20) The radius of the circle
 */
  template <typename T>
  inline
  MYPCircle<T>::MYPCircle( T radius ) : PCurve<T,3>(20, 0, 7) {
      // Note that the last parameter in the PCurve constructor is 7,
      // this because 7 derivatives in eval() is implemented!!!!
    _rx = radius;
    _ry = radius;
    _size = 1.0f;
  }


  template <typename T>
  inline
  MYPCircle<T>::MYPCircle( T radius, T circle_size ) : PCurve<T,3>(20, 0, 7) {
      // Note that the last parameter in the PCurve constructor is 7,
      // this because 7 derivatives in eval() is implemented!!!!
    _rx = radius;
    _ry = radius;
    _size = circle_size;

  }







  /*! MYPCircle<T>::MYPCircle(const MYPCircle<T>& copy )
   *  A copy constructor
   *  Making a copy of the curve (circle)
   *
   *  \param[in] copy The curve to copy
   */
  template <typename T>
  inline
  MYPCircle<T>::MYPCircle( const MYPCircle<T>& copy ) : PCurve<T,3>(copy) {}



  /*! MYPCircle<T>::~MYPCircle()
   *  The destructor
   *  clean up and destroy all private data
   */
  template <typename T>
  MYPCircle<T>::~MYPCircle() {}


  //**************************************
  //        Public local functons       **
  //**************************************

  /*! T MYPCircle<T>::getRadius() const
   *  Give you the radius of the circle (curve)
   *
   *  \return  The radius of the circle
   */
  template <typename T>
  inline
  T MYPCircle<T>::getRadius(int i) const {
    if(i==1)
      return _rx;
    else {
      return _ry;
    }
  }



  /*! void MYPCircle<T>::setRadius( T radius )
   *  Will change the radius of the circle
   *
   *  \param[in] radius   (default 20) The new radius of the circle
   */
  template <typename T>
  inline
  void MYPCircle<T>::setRadius( T radius ) {
      _rx = _ry = radius;
  }






  //***************************************************
  // Overrided (public) virtual functons from PCurve **
  //***************************************************

  /*! bool MYPCircle<T>::isClosed() const
   *  To tell that this curve (circle) is closed.
   */
  template <typename T>
  bool MYPCircle<T>::isClosed() const {
    return true;
  }



  //******************************************************
  // Overrided (protected) virtual functons from PCurve **
  //******************************************************


  template <typename T>
  void MYPCircle<T>::eval( T t, int d, bool /*l*/ ) const {

    this->_p.setDim( d + 1 );

    const T ct_x = _rx * cos(t);
    const T st_y = _ry * sin(t);

    this->_p[0][0] = ct_x;
    this->_p[0][1] = st_y;
    this->_p[0][2] = T(0);

    if( this->_dm == GM_DERIVATION_EXPLICIT ) {
      const T st_x = _rx * sin(t);
      const T ct_y = _ry * cos(t);
      if( d > 0 ) {
        this->_p[1][0] = -st_x;
        this->_p[1][1] =  ct_y;
        this->_p[1][2] =  T(0);
      }
      if( d > 1 ) this->_p[2] = -this->_p[0];
      if( d > 2 ) this->_p[3] = -this->_p[1];
      if( d > 3 ) this->_p[4] =  this->_p[0];
      if( d > 4 ) this->_p[5] =  this->_p[1];
      if( d > 5 ) this->_p[6] =  this->_p[2];
      if( d > 6 ) this->_p[7] =  this->_p[3];
    }
  }




  template <typename T>
  T MYPCircle<T>::getStartP() const {
    return T(T(0));
  }



  /*! T MYPCircle<T>::getEndP() const
   *  Provides the end parameter value associated with
   *  the eval() function implemented above.
   *  (the end parameter value = 2*Pi).
   *
   *  \return The parametervalue at end of the internal domain
   */
//  template <typename T>
//  T MYPCircle<T>::getEndP()const {
//    return T( M_2PI);
//  }

  template <typename T>
  T MYPCircle<T>::getEndP()const {
    return T( M_2PI/_size );
  }



  /*! void MYPCircle<T>::computeSurroundingSphere( const std::vector<DVector<Vector<T,3>>>&, Sphere<T,3>& s ) const
   *  Will set the surrounding sphere of the circle (equal the circle)
   *
   *  \param[in]  p   dummy!!!!, not used
   *  \param[out] s  The surrounding sphere to be computed
   */
  template <typename T>
  void MYPCircle<T>::computeSurroundingSphere( const std::vector<DVector<Vector<T,3>>>& /*p*/, Sphere<T,3>& s ) const {

     s.resetPos(Point<T,3>(T(0)));
     if(_rx > _ry)
       s.resetRadius(_rx);
     else
       s.resetRadius(_ry);
  }

} // END namespace GMlib



#endif // Classproject_MYPCircle_H
