#ifndef GM_PARAMETRICS_CURVES_ModelCurve_H
#define GM_PARAMETRICS_CURVES_ModelCurve_H

//#include<QtWidgets>
#include "parametrics/curves/gmpcircle.h"
#include "parametrics/curves/gmpsubcurve.h"
#include "parametrics/gmpcurve.h"


namespace GMlib {


  template <typename T>
  class ModelCurve : public PCurve<T,3> {
    GM_SCENEOBJECT(ModelCurve)
  public:
    ModelCurve( PCurve <T ,3 >* model_curve, int n);
    ModelCurve( const ModelCurve<T>& copy );
    virtual ~ModelCurve();

    //****************************************
    //****** Virtual public functions   ******
    //****************************************

    // from PCurve
    bool                isClosed() const override;

  protected:
    // Virtual functions from PCurve, which have to be implemented locally
    void                eval(T t, int d, bool l) const override;
    T                   getStartP() const override;
    T                   getEndP()   const override;

    // Protected data for the curve
    int                   _d;
    int                   _n;
    int                   _k;
    std::vector<T>        _t;
//    DVector<Vector<T,3>>  _c;
    PCurve <T ,3 >*          _modelCurve ;
    DVector<PCurve <T ,3 >*> _c; // local curves

    void        makeKnotsOpen(int n,  T s, T e);
    void        makeKnotsClosed(int n, T s, T e);
    void        findControlPoints( const DVector<Vector<T,3>>& p,int m, int n);

    void        localSimulate(double dt) override;

    int         findI(T t) const;
    T           getW(int d, int i, T t) const;
    Vector<T,2> getB(int i, T t) const;

    T           getBlend(T t) const;
    void        localcurves(int n, bool closed) ;


  }; // END class MyB-spline


// Include MyB-spline class function implementations
//*****************************************
// Constructors and destructor           **
//*****************************************


  template <typename T>
  inline
  ModelCurve<T>::ModelCurve(PCurve <T ,3 >* model_curve, int n):PCurve<T,3>(20, 0, 0), _d(1), _k(2) {

    _modelCurve = model_curve;
    _n=n;

    if(isClosed())
    {
        _c.setDim(n+1);
        makeKnotsClosed(n, _modelCurve->getParStart(),_modelCurve->getParEnd());
        for(int i=0;i<n; i++)
        {
            _c[i]= new PSubCurve<T>(_modelCurve,_t[i],_t[i+2],_t[i+1]);
            _c[i]->toggleDefaultVisualizer();
            _c[i]->sample(10,0);
            this->insert(_c[i]);
            _c[i]->setCollapsed(true);
        }
        _c[n]=_c[0];
    }
    else
    {
        _c.setDim(n);
        makeKnotsOpen(n, _modelCurve->getParStart(),_modelCurve->getParEnd());
        for(int i=0;i<n; i++)
        {
            _c[i]= new PSubCurve<T>(_modelCurve,_t[i],_t[i+2],_t[i+1]);
            _c[i]->toggleDefaultVisualizer();
            _c[i]->sample(10,0);
            this->insert(_c[i]);
            _c[i]->setCollapsed(true);
        }

    }

  }

  template <typename T>
  inline
  ModelCurve<T>::ModelCurve( const ModelCurve<T>& copy ) : PCurve<T,3>(copy) {
    _d = copy._d;
    _k = copy._k;
    _t = copy._t;
    _c = copy._c;
  }



  /*! MyB-spline<T>::~MyB-spline()
   *  The destructor
   *  clean up and destroy all private data
   */
  template <typename T>
  ModelCurve<T>::~ModelCurve() {}


  //***************************************************
  // Overrided (public) virtual functons from PCurve **
  //***************************************************

  /*! bool MyB-spline<T>::isClosed() const
   *  To tell that this curve (line) is always open.
   *
   */
  template <typename T>
  bool ModelCurve<T>::isClosed() const {
    return _modelCurve->isClosed();
  }



  //******************************************************
  // Overrided (protected) virtual functons from PCurve **
  //******************************************************


  template <typename T>
  T ModelCurve<T>::getBlend(T t)const{
    //return 3*t*t - 2*t*t*t;
    return t*t / (t*t +(1-t*t));
  }

  template <typename T>
  void ModelCurve<T>::localSimulate( double dt ) {
      static int count = 0 ;
      static double in  = 0.2;
      int x = 10;
      if(count == 300)
      {
        in = -in;

        int scale =  count/10;
        count = 0;

        GMlib::Color scarlet = scale * GMlib::GMcolor::red();
        this->setColor(scarlet);
        //this->rotateParent(dt, Vector<float,3>(0,1,0));// changes here
      }
      if(_modelCurve) {
          std::vector<Vector<T,3>> v;
          for(int i=0; i<_n; i++){
              v.push_back(UnitVector<T, 3>(i, _n-i , i-5)); //generates the path for translate
          }
          for(int i=0; i<_n; i++){
              _c[i]->rotateParent(dt, Vector<float,3>(1,0,0));
            //  _c[i]->translateParent(dt*in*v[i], true);
              _c[i]->translateParent(dt*in* cos(x), true);
            //  _c[i]->turn(30,true);
          }
          this->setEditDone(true);
          PCurve<T,3>::sample(200,0);
          count++;
    }

  }
  template <typename T>
     void ModelCurve<T>::localcurves(int n, bool closed){
         for( int i=0; i<n; i++){
             PSubCurve<T>* ps = new PSubCurve<T>(_modelCurve, _t[i],_t[i+2],_t[i+1]);
             _c[i]=ps;
             ps->toggleDefaultVisualizer();
             ps->sample(10,0);
             this->insert(ps);
             ps->setCollapsed(true);
         }
              _c[0]->rotateParent(30,Vector<float,3>(1,0,0));
         if (_modelCurve->isClosed()){
             _c[n]=_c[0];
         }
     }

  template <typename T>
  void ModelCurve<T>::findControlPoints( const DVector<Vector<T,3>>& p, int m, int n )  {
      T dt =  getParDelta()/(m-1);
      DMatrix<T>A(m,n,T(0));

        for(int j=0;j<m;j++)
        {
            T t = getParStart()+ j * dt;
            int i = findI(t);
            Vector<T,2> b= getB(i,t);
            A[j][i-2] = b[0];
            A[j][i-1] = b[1];
            A[j][i] = b[2];
        }

       DMatrix<T>AT = A;
       AT.transpose();
       DMatrix<T> B = AT * A;
       DVector<Vector<T,3>> q = AT * p;
       B.invert();
       _c = B * q;

  }


  template <typename T>
  void ModelCurve<T>::eval( T t, int d, bool /*l*/ ) const {

    this->_p.setDim( d + 1 );

    int i = findI(t);

    Vector<T,2> b = getB(i, t);

    DVector<Vector<T,3>> c1=_c[i-1]->evaluateParent(t, 0);
    DVector<Vector<T,3>> c2=_c[i]->evaluateParent(t, 0);

    this->_p[0] = b[0]*c1[0] + b[1]*c2[0] ;

  }




  // Protected help functions


  template <typename T>
  void ModelCurve<T>::makeKnotsOpen(int n, T s, T e) {

      _t.resize( n + _k );

    //  std::cout<<_k<<std::endl;
      for(int i=0 ; i < _d; i++ )              // Set the start knots
          _t[i] = s;
      T dt = (e-s)/n-1 ;
      for( int i = _k; i < n; i++ )  // Set the ""-knots
          _t[i] = s + (i-1)*dt;
      for(int i= n; i < _d; i++ )     // Set the end knots
          _t[i] = e;


       //std::cout<<_t<<std::endl;

  }

  // Protected help functions


  template <typename T>
  void ModelCurve<T>::makeKnotsClosed(int n, T s, T e) {

     _t.resize(n  + _k);
     T dt = (e-s)/n;

     for(int i = 0; i < n+_k; i++ )
          _t[i] = s+(i-1)*dt;


  }


  template <typename T>
  int ModelCurve<T>::findI(T t) const{
    for(int i =_d; i<_c.getDim(); i++)
      if(t>=_t[i] && t < _t[i+1])
        return i;
    return _c.getDim()-1;
  }

  template <typename T>
  T ModelCurve<T>::getW(int d, int i, T t) const{

    return (t-_t[i])/(_t[i+d]-_t[i]);
  }


  template <typename T>
  Vector<T,2> ModelCurve<T>::getB(int i, T t) const{

    T w1i = getBlend(getW(1,i,t));

    Vector<T,2> b;
    b[0] = (1-w1i);
    b[1] = w1i;

    return b;
  }


  template <typename T>
  T ModelCurve<T>::getStartP() const {
    return _t[_d];
  }



  template <typename T>
  T ModelCurve<T>::getEndP() const {
    return _t[_c.getDim()];
  }

} // END namepace GMlib


#endif // GM_PARAMETRICS_CURVES_MyB-spline_H
