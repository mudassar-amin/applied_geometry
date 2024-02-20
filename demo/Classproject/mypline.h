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

#ifndef Classproject_MYPLine_H
#define Classproject_MYPLine_H


#include "parametrics/gmpcurve.h"



namespace GMlib {


template <typename T>
class MYPLine : public PCurve<T,3> {
    GM_SCENEOBJECT(MYPLine)
public:
    MYPLine( const DVector<Vector<T,3>>& c);
    MYPLine( const DVector<Vector<T,3>>& p, int n);
    MYPLine( const MYPLine<T>& copy );
    virtual ~MYPLine();

    //****************************************
    //****** Virtual public functions   ******
    //****************************************

    // from PCurve
    bool
    isClosed() const override;

protected:
    // Virtual functions from PCurve, which have to be implemented locally
    void                eval(T t, int d, bool l) const override;
    T                   getStartP() const override;
    T                   getEndP()   const override;

    // Protected data for the curve
    int                   _d ; //degree
    int                   _k; // order
    bool                  _cl;         //!< closed (or open) curve?
    std:: vector<T>       _t;
    DVector<Vector<T,3>>  _c; // control points


    void            makeKnots(int n, int k);
    int             findI(T t) const ;
    T               getW(int d, int i, T t) const;
    Vector<T,3>     getB(int i, T t) const;



}; // END class MYPLine







//*****************************************
// Constructors and destructor           **
//*****************************************


template <typename T>
inline
MYPLine<T>::MYPLine( const DVector<Vector<T,3>>& c ) : PCurve<T,3>(20, 0, 0), _d(2), _k(3) {

    _c = c;
    makeKnots (c.getDim(), _k);
}


template <typename T>
inline
MYPLine<T>::MYPLine( const DVector<Vector<T,3>>& p, int n ) : PCurve<T,3>(20, 0, 0), _d(2), _k(3) {

    _c.setDim(n);
    makeKnots (n, _k);


    // FInd Controls points with least square method..


         //std::cout<<_t<<std::endl;
         int m = p.getDim();
         DMatrix<T>A(m,n,T(0));
         for(int j=0; j<m;j++){
             T t =getParStart()+j*getParDelta()/(m-1);
             int i = findI(t);
             //std::cout<<t<< " "<<i<<std::endl;
             GMlib::Vector<T,3> b=getB(i,t);
             A[j][i-2]=b[0];
             A[j][i-1]=b[1];
             A[j][i]=b[2];
         }
         //std::cout<<A<<std::endl; Bc=y where B=A^T
         GMlib::DMatrix<T>             AT=A;
         AT.transpose();
         GMlib::DMatrix<T>             B=AT*A;
         GMlib::DVector<Vector<T,3>>   q=AT*p;
         B.invert();
         _c=B*q;





}



template <typename T>
inline
MYPLine<T>::MYPLine( const MYPLine<T>& copy ) : PCurve<T,3>(copy) {

    _d = copy._d ; //degree
    _k = copy._k; // order
    _t=copy._t;
    _c=copy._c; // control points
}


template <typename T>
MYPLine<T>::~MYPLine() {}




//***************************************************
// Overrided (public) virtual functons from PCurve **
//***************************************************

/*! bool MYPLine<T>::isClosed() const
   *  To tell that this curve (line) is always open.
   *
   */
template <typename T>
bool MYPLine<T>::isClosed() const {
    return false;
}



//******************************************************
// Overrided (protected) virtual functons from PCurve **
//******************************************************

template <typename T>
void MYPLine<T>::eval( T t, int d, bool /*l*/ ) const {

    this->_p.setDim( d + 1 );

    int i = findI(t);

    Vector<T,3> b = getB(i , t);
    this->_p[0] = b[0]*_c[i-2]+ b[1]*_c[i-1]+ b[2]*_c[i];

}

template <typename T>
T MYPLine<T>::getStartP() const {
    return _t[_d];
}




template <typename T>
T MYPLine<T>::getEndP() const {
    return _t[_c.getDim()];
}


template <typename T>
void MYPLine<T>::makeKnots(int n, int k){
    for(int i=0; i<_k; i++)
        _t.push_back(0);

    for(int i=_k; i<_c.getDim(); i++)
        _t.push_back(i-_d);

    for(int i=0; i<_k; i++)
        _t.push_back(_t[_c.getDim()-1]+1);
    //std::cout << _t << std::endl;

}


template <typename T>
int MYPLine<T>::findI(T t) const {

    for (int i = _d; i<_c.getDim(); i++)
        if (t>=_t[i] && t < _t[i+1])
            return i;

    return _c.getDim()-1;
}


template <typename T>

Vector<T,3> MYPLine<T>::getB(int i, T t) const{

    T w1i = getW(1,i,t);
    T w2im1 = getW(2, i-1,t);
    T w2i = getW(2, i, t);
    Vector<T,3> b;
    b[0] = (1-w1i)*(1-w2im1);
    b[1] = (1-w1i)* w2im1 + w1i*(1-w2i);
    b[2] = w1i*w2i;
    return b;
}


template <typename T>
T  MYPLine<T>::getW(int d, int i, T t) const{

    return (t-_t[i])/(_t[i+d]-_t[i]);
}







} // END namespace GMlib



#endif // GM_PARAMETRICS_CURVES_MYPLine_H
