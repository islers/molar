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

#pragma once

#ifndef		GENERICTYPE_H_
#define		GENERICTYPE_H_

#include <typeinfo>
#include <stdexcept>
#include <iostream>

/** 

Wrapper class using type erasure to hold a variable of any arbitrary data type.

- Construct objects through GenericType( someobject)
- Access its members through as<datatype>() : this method throws a std::logic expression if the datatype is wrong

*/

namespace st_is{

  class GenericType
  {
    protected:
      class TypeErasedVariable
      {
		public:
          virtual ~TypeErasedVariable(){};
		  /** returns the type_info for the stored variable */
		  virtual const std::type_info& getTypeId() const =0;
		  virtual const std::type_info* getTypeInfo() const =0;
	  
		  /** used to get a copy of the class because new isnt possible: just as when using new, deleting has to be done seperately */
		  virtual TypeErasedVariable* newCopyOnHeap() const =0; 
	  
		  /**print content on standard output */
		  virtual void print() const =0;
	  
      };
      
      
      
      template<class T>
      class GenericVariable:public TypeErasedVariable
      {
		public:
		  GenericVariable( T _myT ):Value(_myT),TypeInfo( typeid(_myT) ){};
	  
		  T Value;
	  
		  TypeErasedVariable* newCopyOnHeap() const;
	  
		  const std::type_info& getTypeId() const;
		  const std::type_info* getTypeInfo() const;
	  
		  void print() const;
	  
		private:
		  const std::type_info& TypeInfo;
      };
    
    public:
      
      GenericType():Content( NULL ){};
      
      /**	Constructor function
      *	@param	_value	variable of arbitrary type T that is to be stored in the container
      */
      template<class T>
      explicit GenericType( T _value );
      
      GenericType( const GenericType& _toCopy );
      
      ~GenericType();
      
      /**	Template function to access the stored variable
      *
      *	@return		a reference to the stored variable
      *	@exception	std::logic_error if accessing through wrong datatype T
      */
      
      template<class T>
      T& as();
      
      /**	Template function to access the stored variable
      *
      *	@return		a reference to the stored variable
      *	@exception	std::logic_error if accessing through wrong datatype T
      *			std::logic_error if trying to read uninitialized member
      *	@see		T& as()
      */
      template<class T>
      const T& as() const;
      
      
      GenericType& operator=( const GenericType& );
      
      /** returns true if the generic variable is of type T
      *
      *	@param		variable	variable of type T whose type will be compared to
      *	@return		bool		true if variable has the same type than the generictype
      */
      
      template<class T>
      bool sameType( const T& variable ) const;

	  /** returns a pointer to the type_info element of the encapsulated variable
      */
	  const std::type_info* typeInfo() const;

      /** returns a reference to the type_info element of the encapsulated variable */
      const std::type_info& getTypeId() const;

      
      /** returns true if the generic variable is of type T
      *
      *	@return		bool		true if the generictype variable is of type T
      */
      
      template<class T>
      bool is() const;
      
      
      /**prints the content of the generictype variable to the standard output
      */
      void print() const;
      
    protected:
      TypeErasedVariable* Content;
    
  };


  // template function definitions





  template<class T>
  const std::type_info& GenericType::GenericVariable<T>::getTypeId() const
  { 
    return TypeInfo;
  }


  template<class T>
  const std::type_info* GenericType::GenericVariable<T>::getTypeInfo() const
  {
	  return &TypeInfo;
  }


  template<class T>
  void GenericType::GenericVariable<T>::print() const
  {
    std::cout<<Value;
  }


  template<class T>
  GenericType::TypeErasedVariable* GenericType::GenericVariable<T>::newCopyOnHeap() const
  {
    GenericVariable* newCopy=new GenericVariable(*this);
    return static_cast<TypeErasedVariable*>(newCopy);
  }



  template<class T>
  GenericType::GenericType( T _value )
  {
    Content=new GenericVariable<T>(_value);
  }



  template<class T>
  T& GenericType::as()
  {
   
    if( Content==NULL ) //accessing uninitialized generic type variable that does not have content or encapsulated type. this is only possible for encapsulated types that do support an empty initializer
    {
      Content=new GenericVariable<T>( T() );
    }
    else if( false && typeid(T)!=Content->getTypeId() ) throw std::logic_error("In template function T& GenericType::as<T>(): Trying to access generic type variable as wrong type.");
    
    return static_cast< GenericVariable<T>* >(Content)->Value;
    
  }

  template<class T>
  const T& GenericType::as() const
  {
    
    if( Content==NULL ) //accessing uninitialized generic type variable that does not have content or encapsulated type. this is only possible for encapsulated types that do support an empty initializer
    {
      throw std::logic_error("In const T& GenericType::as() const occured an error: Trying to read from uninitialized member.");
    }
    else if( typeid(T)!=Content->getTypeId() ) throw std::logic_error("In template function T& GenericType::as<T>(): Trying to access generic type variable as wrong type.");
    
    return static_cast< GenericVariable<T>* >(Content)->Value;
    
  }

  template<class T>
  bool GenericType::sameType( const T& variable ) const
  {
    try{ as<T>(); }
    catch( std::logic_error ){ return false; }
    return true;
  }

  template<class T>
  bool GenericType::is() const
  {
    try{ as<T>(); }
    catch( std::logic_error ){ return false; }
    return true;
  }
  
}
    
#endif
