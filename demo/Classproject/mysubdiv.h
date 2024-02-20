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

#ifndef Classproject_MYsubdiv_H
#define Classproject_MYsubdiv_H


#include "parametrics/gmpcurve.h"



namespace GMlib {


template <typename T>
class MYsubdiv : public PCurve<T,3> {
    GM_SCENEOBJECT(MYsubdiv)
public:
    MYsubdiv( const DVector<Vector<T,3>>& c);
    MYsubdiv( const MYsubdiv<T>& copy );
    virtual ~MYsubdiv();

    //****************************************
    //****** Virtual public functions   ******
    //****************************************

    // from PCurve
    bool
    isClosed() const override;
    void  sample(int k, int d=0)override;

protected:
    // Virtual functions from PCurve, which have to be implemented locally
    void                eval(T t, int d, bool l) const override;
    T                   getStartP() const override;
    T                   getEndP()   const override;

    DVector<Vector<T,3>>  _c; // control points


// Virtual functions from Pcurve whaich have to be implemented locally:
    void laneRF(const DVector<Vector<T,3>>& p, int k, int d);
    int doublepart (std::vector<DVector<Vector<T,3>>>&p, int n);
    void smoothPCL (std::vector<DVector<Vector<T,3>>>&p, int n , int d);


}; // END class MYsubdiv







//*****************************************
// Constructors and destructor           **
//*****************************************


template <typename T>
inline
MYsubdiv<T>::MYsubdiv( const DVector<Vector<T,3>>& c ) : PCurve<T,3>(20, 0, 0) {

    _c = c;
    _d = 2;
    //makeKnots (c.getDim(), _k);
}


template <typename T>
inline
MYsubdiv<T>::MYsubdiv( const MYsubdiv<T>& copy ) : PCurve<T,3>(copy) {
    _c=copy._c; // control points
}


template <typename T>
MYsubdiv<T>::~MYsubdiv() {}




//***************************************************
// Overrided (public) virtual functons from PCurve **
//***************************************************

/*! bool MYsubdiv<T>::isClosed() const
   *  To tell that this curve (line) is always open.
   *
   */
template <typename T>
bool MYsubdiv<T>::isClosed() const {
    return true;
}



//******************************************************
// Overrided (protected) virtual functons from PCurve **
//******************************************************

template <typename T>
void MYsubdiv<T>::eval( T t, int d, bool /*l*/ ) const {

    this->_p.setDim( d + 1 );
}




template <typename T>
T MYsubdiv<T>::getStartP() const {
    return 0;
}




template <typename T>
T MYsubdiv<T>::getEndP() const {
    return 1;
}




template <typename T>
void MYsubdiv<T>::laneRF(const DVector<Vector<T,3>>& p, int k, int d){
    int n = p.getDim();
    int m = pow(2,k)*n +1;
   // std::cout << "n = " << n << std::endl;
    std::vector<DVector<Vector<T,3>>>& g= this->_visu[0].sample_val;
    g.resize(m);
    for(int i=0; i<n; i++){
        g[i][0]=p[i];
    }
    g[n++][0]=p[0][0];


    for(int i=0; i<k; i++){
        n=doublepart(g,n);
        smoothPCL(g,n,d);
        //std::cout << "n = " << n << std::endl;

    }
    Sphere<T, 3>& s=_visu[0].sur_sphere;
    s.reset();
    computeSurroundingSphere(g,s);

    //      for(int i=0; i<m; i++)
    //          s += g[i][0];
    //      _visu[0].sur_sphere = s;
    //SceneObject::setSurroundingSphere(_visu[0].sur_sphere);

}

//std::cout<< g << std::endl;



}


template <typename T>
int GMlib::MYsubdiv<T>::doublepart (std::vector<DVector<Vector<T,3>>>&p, int n) {

    for(int i=n-1; i>0; i--){
              p[2*i][0]=p[i][0];
              p[2*i-1][0] = (p[i][0]+p[i-1][0])/2;
          }
          return 2*n-1;
}




template <typename T>
void GMlib::MYsubdiv<T>::smoothPCL (std::vector<DVector<Vector<T,3>>>&p, int n , int d){

    for(int j=1; j<d; j++){
        for(int i=0; i<n-1; i++){
            p[i][0]= (p[i][0] + p[i+1][0])/2;

        }
        p[n-1][0]=p[0][0];

    }
}




template <typename T>
void GMlib::MYsubdiv<T>::sample(int k, int d=0){



    this-> _visu.resize(1);
   // _checkSampleVal(k,d);

    laneRF(_c, k, d);
    this->setEditDone();

}

// END namespace GMlib


#endif // GM_PARAMETRICS_CURVES_MYsubdiv_H
