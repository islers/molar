/*  Copyright (c) 2011, 2014, Stefan Isler, islerstefan@bluewin.ch
 *
    This file is part of MOLAR (Multiple Object Localization And Recognition),
    which was originally developed as part of a Bachelor thesis at the
    Institute of Robotics and Intelligent Systems (IRIS) of ETH Zurich.

    MOLAR is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MOLAR is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with MOLAR.  If not, see <http://www.gnu.org/licenses/>.

*/


#include "generictype.hpp"

namespace st_is{

  GenericType::GenericType( const GenericType& _toCopy )
  {
    if( _toCopy.Content!=NULL ) Content=_toCopy.Content->newCopyOnHeap();
    else Content=NULL;
  }



  GenericType::~GenericType()
  { 
    if( Content!=NULL ) delete Content;
  }



  GenericType& GenericType::operator=( const GenericType& _toEqual )
  {
    if( Content!=NULL ) delete Content;
    
    if( _toEqual.Content!=NULL ) Content=_toEqual.Content->newCopyOnHeap();
    else Content=NULL;

	return *this;
  }



   const std::type_info* GenericType::typeInfo() const
   {
	   return Content->getTypeInfo();
   }


   const std::type_info& GenericType::getTypeId() const
   {
        return Content->getTypeId();
   }



  void GenericType::print() const
  {
    if(Content!=NULL) Content->print();
  }



  template<>
  bool GenericType::sameType<GenericType>( const GenericType& _toCompare ) const
  {
    if( Content==NULL && _toCompare.Content==NULL ) return true;
    if( Content==NULL || _toCompare.Content==NULL ) return false;
    ;
    return Content->getTypeId()==_toCompare.Content->getTypeId();
  }


  
}
