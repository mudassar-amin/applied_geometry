#ifndef Curve_test_H
#define Curve_test_H





#include "parametrics/gmpcurve.h"
#include "scene/gmsceneobject.h"
namespace GMlib {


  template <typename T>
  class Curve_test : public PCurve<T,3> {
    GM_SCENEOBJECT(Curve_test);

  public:
   Curve_test(  T Ky = 3, T Kx = 4 , T a = 2, T b = 3,T c=4,T d=5 );
   Curve_test( const Curve_test<T>& copy );
    virtual ~Curve_test();


    //****************************************
    //****** Virtual public functions   ******
    //****************************************

    // from PCurve
    bool            isClosed() const ;

  protected:
    // Virtual functions from PCurve, which have to be implemented locally
    void            eval(T t, int d,  bool l) const override;
    T               getStartP() const override;
    T               getEndP()   const override;

    T a, b,c, d, Kx, Ky;


  }; // END class Curve_test

} // END namepace GMlib

// Include Curve_test class function implementations

//*****************************************
// Constructors and destructor           **
//*****************************************

/*! Curve_test<T>::Curve_test( T radius )
 *  Default constructor, to make a circle with the radius = radius.
 *
 *  \param[in] radius      (default 20) The radius of the circle
 */
template <typename T>
inline
GMlib::Curve_test<T>::Curve_test(  T Kx, T Ky, T a, T b,T c,T d) : PCurve<T,3>(20, 0, 7) {
    // Note that the last parameter in the PCurve constructor is 7,
    // this because 7 derivatives in eval() is implemented!!!!
    this->Kx = Kx;
    this->Ky = Ky;
    this->a = a;
    this->b = b;
    this->c = c;
    this->d = d;


  }


  /*! Curve_test<T>::Curve_test(const Curve_test<T>& copy )
   *  A copy constructor
   *  Making a copy of the curve (circle)
   *
   *  \param[in] copy The curve to copy
   */
  template <typename T>
  inline
  GMlib::Curve_test<T>::Curve_test( const Curve_test<T>& copy ) : PCurve<T,3>(copy) {}



  /*! Curve_test<T>::~Curve_test()
   *  The destructor
   *  clean up and destroy all private data
   */
  template <typename T>
  GMlib::Curve_test<T>::~Curve_test() {}








  //***************************************************
  // Overrided (public) virtual functons from PCurve **
  //***************************************************

  /*! bool Curve_test<T>::isClosed() const
   *  To tell that this curve (circle) is closed.
   */
  template <typename T>
  bool GMlib::Curve_test<T>::isClosed() const {
    return true;
  }



  //******************************************************
  // Overrided (protected) virtual functons from PCurve **
  //******************************************************

  /*! void Curve_test<T>::eval( T t, int d, bool l ) const
   *  Evaluation of the curve at a given parameter value
   *  To compute position and d derivatives at parameter value t on the curve.
   *  7 derivatives are implemented
   *
   *  \param  t[in]  The parameter value to evaluate at
   *  \param  d[in]  The number of derivatives to compute
   *  \param  l[in]  (dummy) because left and right are always equal
   */
  template <typename T>
  void GMlib::Curve_test<T>::eval( T t, int d, bool /*l*/ ) const {

    this->_p.setDim( d + 1 );


    int x =1;


    //x = a* cos(kx * t);
   // y = a* sin (ky * t)/2; sin(t) * (exp(cos(t)) - 2*cos(4*t) - pow(sin(t/12.0), 5));
    //this->_p[0][0] = a * cos(t) - (a * pow(sin(t),2))/sqrt(2);
   // this->_p[0][1] = a *  cos(t) * sin( t);

    this->_p[0][0] = a*sin(t) * (exp(cos(t)) - a*2*cos(4*t) - a*pow(sin(t/12.0), 5));
    this->_p[0][1] = a*cos(t) * (exp(cos(t)) - a*2*cos(4*t) - a*pow(sin(t/12.0), 5));
    //this->_p[0][0] = a * t - b * sin(t);
   // this->_p[0][1] = c * log(abs(tan(t/2))) + d * sin(t);
    this->_p[0][2] = T(0);


//   this->_p[0][2] = T(0);

  }



  /*! T Curve_test<T>::getStartP() const
   *  Provides the start parameter value associated with
   *  the eval() function implemented above.
   *  (the start parameter value = 0).
   *
   *  \return The parametervalue at start of the internal domain
   */
  template <typename T>
  T GMlib::Curve_test<T>::getStartP() const {
    return T(0);
  }



  /*! T Curve_test<T>::getEndP() const
   *  Provides the end parameter value associated with
   *  the eval() function implemented above.
   *  (the end parameter value = 2*Pi).
   *
   *  \return The parametervalue at end of the internal domain
   */
  template <typename T>
  T GMlib::Curve_test<T>::getEndP()const {
    return T(M_2PI);
  }






#endif // TEST_H
