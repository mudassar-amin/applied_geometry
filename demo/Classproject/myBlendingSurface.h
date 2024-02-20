#ifndef MYBLENDINGSPLINESURFACE_H
#define MYBLENDINGSPLINESURFACE_H




#include "parametrics/curves/gmpsubcurve.h"
#include "parametrics/gmpsurf.h"
#include "scene/gmsceneobject.h"
#include "mysurface.h"
#include "simplesubsurf-1.h"





namespace GMlib {

//Blending Spline = special B-Spline
// Use sub curve construction 8.3
//Implement function
//Constructor points to model curves, and int local curves
//Open/Closed based on model curve
//Use subcurve

//Blending spline = b spline 1st degree seond order
//We are to make a copy of a different curve wth a different domain
//Tensorprodukt kapittel 12.2


template <typename T>
class myBlendingSurface : public PSurf<T,3> {
    GM_SCENEOBJECT(myBlendingSurface);

public:

    myBlendingSurface(  PSurf<T,3>* p, int nu, int nv);
    myBlendingSurface( const myBlendingSurface<T>& copy );

    virtual ~myBlendingSurface();


    //****************************************
    //****** Virtual public functions   ******
    //****************************************

    // from PCurve
    bool            isClosedV() const override ;
    bool            isClosedU() const override ;
    void            sample(int mu, int mv, int du =0, int dv = 0) override;

protected:
    // Virtual functions from PCurve, which have to be implemented locally
    void            eval(T u, T v, int d1, int d2, bool lu = true, bool lv = true ) const override;
    T               getStartPU() const override;
    T               getStartPV() const override;
    T               getEndPU()   const override;
    T               getEndPV()   const override;

   // void            localcurves(int n, bool closed);
    /*
     * For B_Spline we need Degree, Coordinates, Order
     */
    int                   _du;
    int                   _ku;
    int                   _dv;
    int                   _kv;

    /*
     * Big T = typename, decides if it wnat to be double or float or anything else
     */

    std::vector<T>        _u;
    std::vector<T>        _v;


    /*
     * Controll curve
     */

    PSurf<T,3>*            model_surf;
    DMatrix<PSurf<T, 3>*> _c;
    /*
     * Number of controll points
     */
    int                    _nu;
    int                    _nv;
    int                    _sampleU;
    int                    _sampleV;


    void            localSimulate(double dt) override;





    /*
     * Help functions
     */
    //_t = knotvektor
    void             makeKnotVectors( std::vector<T>& _t, int n, T s, T e);
    void             makeKnotVectorsClosed(std::vector<T>& _t, int n, T s, T e);
    T                getW(const std::vector<T>& _t, int d, int i, T t) const;
    T                getWDer(const std::vector<T>& _t, int d, int i, T t) const;
    int              findI( const std::vector<T>& _t, T t, bool cl)const;
    //Create a getBlend(t){ return b} Function and use getBlend(getW(d, i, t)) instead og getW
    //getB computes
    T                getBlend(T t) const;
    T                getBlendDer(T t) const;
    Vector<T, 2>     getB(const std::vector<T>& _t, int i, T t) const;
    Vector<T, 2>     getBDer(const std::vector<T>& _t, int i, T t) const;





}; // END class myBlendingSurface

} // END namepace GMlib

// Include myBlendingSurface class function implementations

//*****************************************
// Constructors and destructor           **
//*****************************************



template <typename T>
inline
//_d = degree     _k order
GMlib::myBlendingSurface<T>::myBlendingSurface( PSurf<T,3>* p, int nu, int nv )   {

    model_surf = p;
    _nu = nu;
    _nv = nv;
    _du = _dv = 1;
    _ku = _kv = 2;


    if (isClosedU())
        makeKnotVectorsClosed(_u, _nu, p->getParStartU(), p->getParEndU() );
    else
         makeKnotVectors(_u, _nu, p->getParStartU(), p->getParEndU() );



    if (isClosedV())
        makeKnotVectorsClosed(_v, _nv, p->getParStartV(), p->getParEndV() );
    else
        makeKnotVectors(_v, _nv, p->getParStartV(), p->getParEndV() );

    //Nu + 0 om åpen, 1 om lukket
     if (isClosedU())
         nu++;

     if(isClosedV())
         nv++;

    _c.setDim(nu , nv);

    for (int j =0; j<_nv; j++){
        for (int i =0; i<_nu; i++){
            _c[i][j] = new PSimpleSubSurf<T>( model_surf, _u[i],  _u[i+2], _u[i+1], _v[j],  _v[j+2], _v[j+1]);
            _c[i][j]->toggleDefaultVisualizer();
            _c[i][j]->sample(10,10,1,1);
            this->insert(_c[i][j]);
            _c[i][j]->setCollapsed(true);
        }
        if (isClosedU())
            _c[_nu][j] = _c[0][j];
    }
    if (isClosedV())
        for (int i =0; i<_nu; i++)
            _c[i][_nv] = _c[i][0];

    if (isClosedV() && isClosedU())
        _c[_nu][_nv] = _c[0][0];









}

template <typename T>
inline
GMlib::myBlendingSurface<T>::myBlendingSurface( const myBlendingSurface<T>& copy)   {




    _du = copy._du;
    _ku = copy._ku;
    _c  = copy._c;
    _nu = copy._nu;
    _nv = copy._nv;
    _v  = copy._v;
    _u  = copy._u;
    _dv = copy._dv;
    _kv = copy._kv;
}


template <typename T>
GMlib::myBlendingSurface<T>::~myBlendingSurface() {}


//***************************************************
// Overrided (public) virtual functons from PCurve **
//***************************************************

template <typename T>
bool GMlib::myBlendingSurface<T>::isClosedU() const {
    return model_surf->isClosedU();
}

template <typename T>
bool GMlib::myBlendingSurface<T>::isClosedV() const {
    return model_surf->isClosedV();
}

//******************************************************
// Overrided (protected) virtual functons from PCurve **
//******************************************************


template <typename T>
void GMlib::myBlendingSurface<T>::eval( T u, T v, int d1, int d2, bool /*lu*/ , bool /*lv*/  ) const {

    this->_p.setDim( d1 + 1, d2 +1 );

    int i = findI(_u, u, isClosedU());
    int j = findI(_v, v, isClosedV());

   //4 basis funtions. 2 in u direction, 2 in v direction;
    Vector<T, 2> bu = getB(_u, i, u);
    Vector<T, 2> bv = getB(_v, j, v);

    Vector<T, 2> buDer = getBDer(_u, i, u);
    Vector<T, 2> bvDer = getBDer(_v, j, v);




   //Evaluateparent creates a Dmatrix
    DMatrix<Vector<T,3>> c1 = _c(i-1)(j-1)->evaluateParent(u, v,d1, d2);
    DMatrix<Vector<T,3>> c2 = _c(i)(j-1)->evaluateParent(u, v,d1, d2);
    DMatrix<Vector<T,3>> c3 = _c(i-1)(j)->evaluateParent(u, v,d1, d2);
    DMatrix<Vector<T,3>> c4 = _c(i)(j)->evaluateParent(u, v,d1, d2);


    //Du = C[o][1]

    //(Dv = C[1][0]
    //This is to get the position
    this->_p[0][0] = bv[0]*(bu[0]* c1[0][0] + bu[1]* c2[0][0]) + bv[1] *( bu[0]* c3[0][0] + bu[1]* c4[0][0]) ;

//    Thses 2 are to get the normal in order to get the shading on the surface
    this->_p[0][1] = bv[0] * (buDer[0] * c1[0][0] + bu[0] *c1[0][1] + buDer[1] * c2[0][0] +bu[1] *c2[0][1]) +
            bv[0] * (buDer[0] * c3[0][0] + bu[0] *c3[0][1] + buDer[1] * c4[0][0] +bu[1] *c4[0][1]) ;

    this->_p[1][0] = bvDer[0] * (bu[0]* c1[0][0] + bu[1]* c2[0][0]) + bv[0] * (bu[0] * c1[1][0] + bv[1]* c2[1][0]) +
            bvDer[1] * (bu[0]* c3[0][0] + bu[1]* c4[0][0]) + bv[0] * (bu[0] * c3[1][0] + bv[1]* c4[1][0]) ;




}

template <typename T>
T GMlib::myBlendingSurface<T>::getStartPU() const {

    return _u[_du];
}

template <typename T>
T GMlib::myBlendingSurface<T>::getStartPV() const {

    return _v[_dv];
}

template <typename T>
T GMlib::myBlendingSurface<T>::getEndPU()const {

    return _u[_c.getDim1()];
}

template <typename T>
T GMlib::myBlendingSurface<T>::getEndPV()const {

    return _v[_c.getDim2()];
}

template <typename T>
void GMlib::myBlendingSurface<T>::makeKnotVectors(std::vector<T>& _t, int n, T s, T e){


    for( int i = 0; i < 2; i++)
        _t.push_back(s);

    //Ask model_curve if open /n-1 if closed /n


    T dt = (e-s)/(n-1);
    for( int i = 2; i < n; i++)
        _t.push_back(s + (i-1)* dt);


    for( int i = 0; i < 2; i++)
        _t.push_back(e);



}


template <typename T>
void GMlib::myBlendingSurface<T>::makeKnotVectorsClosed(std::vector<T>& _t, int n, T s, T e){




    T dt = (e-s)/(n);
    _t.push_back(s-dt);

    for( int i = 1; i < n+_ku; i++)
        _t.push_back(s + (i-1)* dt);



}





template <typename T>

T GMlib::myBlendingSurface<T>::getW(const std::vector<T>& _t, int d, int i, T t)const{
    return (t-_t[i])/(_t[i+d]-_t[i]);
}
template <typename T>
T GMlib::myBlendingSurface<T>::getWDer(const std::vector<T>& _t, int d, int i, T t)const{
    return 1/(_t[i+d]-_t[i]);
}


template <typename T>
int GMlib::myBlendingSurface<T>::findI(const std::vector<T>& _t, T t, bool cl)const{
    int r = cl? 2:3;
    for(int i = _du; i <_t.size()-r; i++)
        if (t >=_t[i] && t < _t[i+1]){
            return i;
        }
    return _t.size()-r;


}

template <typename T>
GMlib::Vector<T, 2>  GMlib::myBlendingSurface<T>::getB(const std::vector<T>& _t, int i, T t)const{
    T w1 = getBlend(getW(_t, 1, i, t));
    Vector<T, 2> b;
    b[0] = (1-w1);
    b[1] = w1;

    return b;
}


template <typename T>
GMlib::Vector<T, 2>  GMlib::myBlendingSurface<T>::getBDer(const std::vector<T>& _t, int i, T t)const{
    T w1 = getBlendDer(getW(_t, 1, i, t)*getWDer(_t, 1, i, t));
    Vector<T, 2> bd;
    bd[0] = -w1; // derivates of get w and getbled
    bd[1] = w1;

    return bd;
}

template <typename T>
T  GMlib::myBlendingSurface<T>::getBlend(T t)const{
   T b = 3*pow(t, 2) - 2*pow(t, 3);
   return b;
}

template <typename T>
T  GMlib::myBlendingSurface<T>::getBlendDer(T t)const{
   T b = 6*t - 6*pow(t, 2);
   return b;
}


 template <typename T>
 void GMlib::myBlendingSurface<T>::sample(int mu,int mv, int du, int dv){

     _sampleU = mu;
     _sampleV = mv;
     PSurf<T,3>::sample(mu, mv, du, dv);
 }


//template <typename T>
//void GMlib::myBlendingSurface_Surface<T>::localcurves(int n, bool closed){
//    for( int i=0; i<n; i++){
//        PSubCurve<T>* ps = new PSubCurve<T>(_modelcurve, _t[i],_t[i+2],_t[i+1]);
//        _c[i]=ps;
//        ps->toggleDefaultVisualizer();
//        ps->sample(10,0);
//        this->insert(ps);
//        ps->setCollapsed(true);

//    }

//    if (_modelcurve->isClosed()){
//        _c[n]=_c[0];
//    }

//}


 template <typename T>
 void GMlib::myBlendingSurface<T>::localSimulate(double dt){

       PSurf<T,3>::sample(_sampleU, _sampleV, 1, 1);
     }






#endif // MYBLENDINGSPLINESURFACE_H
