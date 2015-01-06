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

#ifndef		GENERICMULTILEVELMAP_H_
#define		GENERICMULTILEVELMAP_H_

#include <cstring>
#include <map>
#include "generictype.hpp"
#include "ticpp.h"
#include <sstream>
#include <fstream>
#include <iostream>

/** template wrapper class for std::map, adding some additional functionality
This class easily provides a multilevel, generic type functionality, where data of arbitrary
type x can be stored at arbitrary level of depth (arbitrary at each level!) through as<x>().
Accessing through wrong types throws logic_errors.

additionally, the GMLM can be loaded directly from xml files. check the corresponding functions
for details.

can store an arbitrary number of values of each type
*/

namespace st_is{


  template<class KEY_TYPE=int>
  class GenericMultiLevelMap{
    public:
      
      /** empty constructor */
      GenericMultiLevelMap():ContentAccessed(false){};


	  std::type_info& levelContentType() const;
      
      /**	tests whether there is an entry with key _key in this level of the map
      *
      *	@param		KEY_TYPE		the key which should be looked for
      *	@return		bool			true if the key is found, false otherwhise
      */
      bool hasKey( KEY_TYPE _key ) const;
      
      
      /** Counts how many times the key _key exists on this GMLM level. As each key is unique
      ** on a specific level, this will return 0 if it isn't found and 1 if it is.
      **
      ** @param		KEY_TYPE		the key to be found
      */
      int count( KEY_TYPE _key ) const;
      
      
      /**	erases the entry with key _key
      *
      *	@param		KEY_TYPE		the key of the entry that should be erased
      */
      void eraseKey( KEY_TYPE _key );
      
      
      /** swaps the whole content of that level */
      void clear();
      
      
      /** returns how many entries the GenericMultiLevelMap has at each level */
      int size() const;
      
      
      /** tests whether the GenericMultiLevelMap is empty or not on all levels and 
      * if the Content has not been accessed
      */
      bool empty() const;
      
      
      /** returns a reference to the content of respective level
      *
      * @exception:	std::logic_error	if accessing through wrong datatype T
      */
      template<class T>
      T& as();
      
      
       /** returns a const reference to the content of respective level
      *
      * @exception:	std::logic_error	if accessing through wrong datatype T
      */
      template<class T>
      const T& as() const;


	  /** checks if the content at the given level is of the given type
	  */
	  template<class T>
	  bool is() const;


      /** initializes the map from an xml file, using the ticpp class //only makes sense for KEY_TYPE=string
      **
      ** DETAILED DESCRIPTION:
      ** ------------------------------------------------------------------------------------------
      ** xml file structure:
      ** example: <entity attributename="value"><membertype value="some_value" /></entity>
      **
      ** for each entity a new path [entity] is built in the map, the membertype value some_value
      ** is added as the value of the node, accessible through [entity].as<membertype>()
      **
      ** member types:_____________________________________________________________________________
      ** supported types are: string,bool,int,unsigned int,double,float,char
      ** such entities will always be treated as membertypes.
      **
      ** text is treated as member type string,
      ** thus <entity>text text text</entity> is equivalent to
      ** <entity><string value="text text text"/></entity>
      ** also <membertype value="some_value"/> is equivalent to <membertype>some_value</membertype>
      **
      ** if you declare <membertype><entityname>something</entityname></membertype>, then the
      ** function will try to save the whole string "<entityname>something</entityname>" as type
      ** membertype.
      **
      ** if your member is of type "string" it can be added to an entity also directly through
      ** <entity value="my_member" />
      **
      ** note:
      ** - if the string value of a membertype can not be casted to the membertype, an exception is
      ** thrown.
      ** - other attributes than "value" of member types are ignored
      **
      ** attributes:_______________________________________________________________________________
      ** The attributes of an entity are saved under [entity][ATTRIBUTE][attributename].as<string>() as
      ** std::string type. The function does not test for multiple declaration with the same name.
      ** In such a case, only the last declaration will be saved in the map,older declarations get
      ** overwritten. The same does apply if a node has more than one membertype.
      ** e.g. <window width="100px"></window> will be accessible through
      **      gmlElement["window"]["ATTRIBUTE"]["width"].as<string>()
      ** Note that the attribute name "value" is reserved. Such attirbute will be added as values to
      ** a node as<std::string>()
      **
      **
      ** The keyword "ATTRIBUTE" is reserved for the gmlm. If it is used as a entity name, this
      ** entity will be ignored.
      **
      ** Comments and unknown tags are ignored.
      **
      ** @param		std::string		the path to the xml file
      ** @exception	ticpp::Exception	if ticpp isn't able to load the file for any reason
      ** @exception	std::logic_error	if casting a membertype value to the membertype fails
      */
      void initFromXML( std::string _docPath );


	  bool saveToXML( std::string _docPath );
      
	  /** writes the GMLM to an xml file  //only makes sense for KEY_TYPE=string
	  *
	  */
	  void writeToXML( std::ofstream& _file, GenericMultiLevelMap<KEY_TYPE>* _map, int _level );

	  std::string levelIndent( int _level ) const;


	  /** writes the member type string for the content of _map on first level of _map */
	  std::string writeMemberType( GenericMultiLevelMap<KEY_TYPE>& _map );


	  /** detects the type of the content variable and writes it to the output string
	  *   Works currently only for the built in types string, bool, int, unsigned int, double, float and char
	  *
	  */
	  std::string toString( GenericMultiLevelMap<KEY_TYPE>* _map );
      
      /** returns the GenericMultiLevelMap with key _key and constructs it if it doesnt exist yet */
      GenericMultiLevelMap& operator[]( KEY_TYPE _key );
      
      
      /** const operator[] function
      **
      ** @param	KEY_TYPE			the key
      ** @return	const GenericMultiLevelMap&	if _key is valid, then a const reference
      ** 						to the GMLM that belongs to _key
      ** 						if _key is invalid, then a const reference
      ** 						to the parent GMLM
      */
      const GenericMultiLevelMap& operator[]( KEY_TYPE _key ) const;
      
      
      /** you can use this static method to register your own membertypes with corresponding
      ** cast functions
      **
      ** what the cast function should do simply is to cast the std::string object to your target
      ** type and then save this in the GMLM element through GMLM.as<target_type>()=the casted
      ** variable.
      ** existing casts with the same identifier name will be overridden.
      **
      ** @param		std::string							the name of your membertype. if the name does
      ** 										already exist, then this new declaration will
      ** 										override the old declaration
      ** @param		void(*)( std::string,GenericMultiLevelMap<KEY_TYPE>& )		the cast function
      */
      static void registerCast( std::string _memberTypeName,void (*_castFunction)( std::string,GenericMultiLevelMap<KEY_TYPE>& ) );
      
      class iterator; // bidirectional iterator
      class const_iterator; // bidirectional iterator
      
      iterator begin();
      const_iterator begin() const;
      iterator end();
      const_iterator end() const;
      
      /**flags for the following function*/
      enum
      {
		TYPESECURE=1,
		THROWEXCEP=2,
		IGNOREEMPTYSTRING=4
      };
      
      /** This function does copy all values of those keys from _toMirror in the actual GMLM, that
      ** do also exist in the GMLM. The very first node (root) gets copied anyway.
      **
      ** @param		const GenericMultiLevelMap<KEY_TYPE>&		the GMLM which values shall be mirrored
      ** @param		int _flagSetting			bitmask with the following options:
      ** 		TYPESECURE					if set then the function will not mirror values that are not
      ** 								of the same type than the type in the target gmlm. The values
      ** 								are either ignored, or an exception is thrown. (see THROWEXCEP flag)
      ** 		THROWEXCEP					only applies if TYPESECURE is set. if set: exceptions are thrown if values
      ** 								do not have the same type, if not set: no exception is thrown and the value
      ** 								simply does not get copied.
      ** 		IGNOREEMPTYSTRING				set: empty (string) fields are ignored and not mirrored
      ** @exception	std::logic_error				thrown if THROWEXCEP is set and the content of a twin pair has not the same type
      */
      void mirrorTwins( const GenericMultiLevelMap<KEY_TYPE>& _toMirror, int _flagSetting=0 );
      
      
      /** prints the gmlm with all its sublevels to the standard output
      */
      void print() const;
      
    protected:
    private:
      void loadXMLLayer( ticpp::Node const* _actualElement, GenericMultiLevelMap<KEY_TYPE>& _actualPosition );
      void loadAttributes( ticpp::Node const* _element, GenericMultiLevelMap<KEY_TYPE>& _actualMapPosition );
      bool isMemberType( ticpp::Element const* _element );
      bool nameIsAttribute( ticpp::Element const* _element );
      
      /** Helper function for mirrorTwins: Mirrors the values of keys in _sourceMap that exist in _targetMap
      ** as well to the _targetMap.
      **
      ** @param		GenericMultiLevelMap<KEY_TYPE>&		where the values will be saved in
      ** @param		const GenericMultiLevelMap<KEY_TYPE>&	where the values come from
      ** @see		void mirrorTwins( const GenericMultiLevelMap<KEY_TYPE>& )
      */
      void mirrorTwinsSub( GenericMultiLevelMap<KEY_TYPE>& _targetMap, const GenericMultiLevelMap<KEY_TYPE>& _sourceMap, int _flagSetting=0 );
      
      void printHelper( const GenericMultiLevelMap<KEY_TYPE>& _toPrint, unsigned int _levelIndicator ) const;
      
      static std::map<std::string,void(*)( std::string,GenericMultiLevelMap<KEY_TYPE>& )>* stm_CastTable;
      
      static std::map<std::string,void(*)( std::string,GenericMultiLevelMap<KEY_TYPE>& )>& getCastTable();
      
      /** initializes the built in casts */
      static std::map<std::string,void(*)( std::string,GenericMultiLevelMap<KEY_TYPE>& )>* initCastTable();
      
      /** cast function, saving the casted string to _target
      **
      ** @param		std::string				the input
      ** @param		GenericMultiLevelMap<KEY_TYPE>&		the target
      ** @exception	std::logic_error			if casting fails
      */
      static void stringCast( std::string _input, GenericMultiLevelMap<KEY_TYPE>& _target );
      /** cast function, saving the casted string to _target
      **
      ** @param		std::string				the input
      ** @param		GenericMultiLevelMap<KEY_TYPE>&		the target
      ** @exception	std::logic_error			if casting fails
      */
      static void boolCast( std::string _input, GenericMultiLevelMap<KEY_TYPE>& _target );
      /** cast function, saving the casted string to _target
      **
      ** @param		std::string				the input
      ** @param		GenericMultiLevelMap<KEY_TYPE>&		the target
      ** @exception	std::logic_error			if casting fails
      */
      static void intCast( std::string _input, GenericMultiLevelMap<KEY_TYPE>& _target );
      /** cast function, saving the casted string to _target
      **
      ** @param		std::string				the input
      ** @param		GenericMultiLevelMap<KEY_TYPE>&		the target
      ** @exception	std::logic_error			if casting fails
      */
      static void unsignedIntCast( std::string _input, GenericMultiLevelMap<KEY_TYPE>& _target );
      /** cast function, saving the casted string to _target
      **
      ** @param		std::string				the input
      ** @param		GenericMultiLevelMap<KEY_TYPE>&		the target
      ** @exception	std::logic_error			if casting fails
      */
      static void doubleCast( std::string _input, GenericMultiLevelMap<KEY_TYPE>& _target );
      /** cast function, saving the casted string to _target
      **
      ** @param		std::string				the input
      ** @param		GenericMultiLevelMap<KEY_TYPE>&		the target
      ** @exception	std::logic_error			if casting fails
      */
      static void floatCast( std::string _input, GenericMultiLevelMap<KEY_TYPE>& _target );
      /** cast function, saving the casted string to _target
      **
      ** @param		std::string				the input
      ** @param		GenericMultiLevelMap<KEY_TYPE>&		the target
      ** @exception	std::logic_error			if casting fails
      */
      static void charCast( std::string _input, GenericMultiLevelMap<KEY_TYPE>& _target );
      
      bool ContentAccessed; // used to indicate whether Content has been accessed or not
      GenericType Content; // used to store an actual value at this level
      mutable std::map< KEY_TYPE,GenericMultiLevelMap<KEY_TYPE> > NextLevel; // used to store other levels
      
  };

  /** two wrapper classes that mirror the std::map::iterators to the generic multilevel map
  */

  template<class KEY_TYPE=int>
  class GenericMultiLevelMap<KEY_TYPE>::iterator:public std::iterator<std::bidirectional_iterator_tag,GenericMultiLevelMap<KEY_TYPE> >
  {
    public:
      iterator():m_iterator(){};
      iterator( typename std::map< KEY_TYPE,GenericMultiLevelMap<KEY_TYPE> >::iterator _iterator ):m_iterator(_iterator){};
      
      iterator& operator=( const iterator& _toCopy ){ m_iterator=_toCopy.m_iterator; return *this; };
      iterator( const iterator& _toCopy ):m_iterator(_toCopy.m_iterator){};
      
      bool operator==( const iterator& _toCompare ) const { return m_iterator==_toCompare.m_iterator; };
      bool operator!=( const iterator& _toCompare ) const { return m_iterator!=_toCompare.m_iterator; };
      
      std::pair< const KEY_TYPE, GenericMultiLevelMap<KEY_TYPE> >& operator*() { return (*m_iterator); };
      std::pair< const KEY_TYPE, GenericMultiLevelMap<KEY_TYPE> >* operator->() { return m_iterator.operator->(); };
      
      iterator& operator++() { ++m_iterator; return (*this); };
      iterator operator++(int) { iterator copy=(*this); ++m_iterator; return copy; };
      
      iterator& operator--() { --m_iterator; return (*this); };
      iterator operator--(int) { iterator copy=(*this); --m_iterator; return copy; };
      
    private:
      typename std::map< KEY_TYPE,GenericMultiLevelMap<KEY_TYPE> >::iterator m_iterator;
  };



  template<class KEY_TYPE=int>
  class GenericMultiLevelMap<KEY_TYPE>::const_iterator:public std::iterator< std::bidirectional_iterator_tag,GenericMultiLevelMap<KEY_TYPE> >
  {
    public:
      const_iterator(){};
      const_iterator( typename std::map< KEY_TYPE,GenericMultiLevelMap<KEY_TYPE> >::const_iterator _iterator ):m_iterator(_iterator){};
      
      const_iterator& operator=( const const_iterator& _toCopy ){ m_iterator=_toCopy.m_iterator; return *this; }
      const_iterator( const const_iterator& _toCopy ):m_iterator(_toCopy.m_iterator){};
      
      bool operator==( const const_iterator& _toCompare ) const { return m_iterator==_toCompare.m_iterator; }
      bool operator!=( const const_iterator& _toCompare ) const { return m_iterator!=_toCompare.m_iterator; }
      
      const std::pair< const KEY_TYPE, GenericMultiLevelMap<KEY_TYPE> >& operator*() { return (*m_iterator); }
      const std::pair< const KEY_TYPE, GenericMultiLevelMap<KEY_TYPE> >* operator->() { return m_iterator.operator->(); }
      
      const_iterator& operator++() { ++m_iterator; return (*this); }
      const_iterator operator++(int) { iterator copy=*this; --m_iterator; return copy; }
      
      const_iterator& operator--() { --m_iterator; return *this; }
      const_iterator operator--(int) { iterator copy=*this; --m_iterator; return copy; }
	  
    private:
      typename std::map< KEY_TYPE,GenericMultiLevelMap<KEY_TYPE> >::const_iterator m_iterator;
  };



  // (author) Stefan Isler, ETH Zürich, Jan 2011, v1.1 July 2014

  #define		TEMPT		template <class KEY_TYPE>

  //main class function template definitions///////////////////////////////////////////////////////
  TEMPT
  std::map<std::string,void(*)( std::string,GenericMultiLevelMap<KEY_TYPE>& )>* GenericMultiLevelMap<KEY_TYPE>::stm_CastTable;



  TEMPT
  std::type_info& GenericMultiLevelMap<KEY_TYPE>::levelContentType() const
  {
      return Content.getTypeId();
  }

  TEMPT
  bool GenericMultiLevelMap<KEY_TYPE>::hasKey( KEY_TYPE _key ) const
  {
    return NextLevel.count( _key )>=1;
  }



  TEMPT
  int GenericMultiLevelMap<KEY_TYPE>::count( KEY_TYPE _key ) const
  {
    return NextLevel.count( _key );
  }



  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::eraseKey( KEY_TYPE _key )
  {
    NextLevel.erase( _key );
  }



  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::clear()
  {
    NextLevel.clear();
  }



  TEMPT
  int GenericMultiLevelMap<KEY_TYPE>::size() const
  {
    return NextLevel.size();
  }



  TEMPT
  bool GenericMultiLevelMap<KEY_TYPE>::empty() const
  {
    return NextLevel.empty()&&!ContentAccessed;
  }



  TEMPT
  template<class T>
  T& GenericMultiLevelMap<KEY_TYPE>::as()
  {
    ContentAccessed=true;
    return Content.as<T>();
  }



  TEMPT
  template<class T>
  const T& GenericMultiLevelMap<KEY_TYPE>::as() const
  {
    return Content.as<T>();
  }



  TEMPT
  template<class T>
  bool GenericMultiLevelMap<KEY_TYPE>::is() const
  {
    return Content.is<T>();
  }



  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::initFromXML( std::string _docPath )
  {
    ticpp::Document myDoc;
    try{
        myDoc.LoadFile( _docPath ); //throws a ticpp::Exception exception if not successful

        ticpp::Node* actualElement=&myDoc;

        loadXMLLayer( actualElement, *this );
        }
    catch(ticpp::Exception e)
    {
        std::cerr<<e.m_details<<std::endl;
        std::cerr<<std::endl<<"File "<<_docPath<<" couldn't be loaded."<<std::endl;
    }
    return;
  }

  TEMPT
  bool GenericMultiLevelMap<KEY_TYPE>::saveToXML( std::string _docPath )
  {
      try{
          std::ofstream file;

          file.open(_docPath.c_str(),std::ios_base::trunc); //existing file gets overwritten

          writeToXML(file,this,0);

          file.close();
      }
      catch(...)
      {
          return true; //in C return value 1 usually means that an error occured...
      }

      return false;
  }

  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::writeToXML( std::ofstream& _file, GenericMultiLevelMap<KEY_TYPE>* _map, int _level )
  {
      for( GenericMultiLevelMap<KEY_TYPE>::iterator it=_map->begin(); it!=_map->end(); it++ )
      {
          if( (*it).first=="ATTRIBUTE" ) continue;

          _file << levelIndent(_level) << "<" << (*it).first; //open level
          //build level
          if( (*it).second.ContentAccessed && (*it).second.is<std::string>() )
          {
              _file<<" value=\""<<toString( &(*it).second )<<"\"";
          }
          if( (*it).second.hasKey("ATTRIBUTE") )
          {
              GenericMultiLevelMap<KEY_TYPE> attributes=(*it).second["ATTRIBUTE"];
              for( GenericMultiLevelMap<KEY_TYPE>::iterator att=attributes.begin();att!=attributes.end();att++)
              {
                  _file<<" "<<(*att).first<<"=\""<< toString( &(*att).second ) <<"\"";
              }
          }
          if( !( (*it).second.ContentAccessed && !(*it).second.is<std::string>() ) && (*it).second.size()-(int)(*it).second.hasKey("ATTRIBUTE") <= 0 )
          {
              _file<<"/>"<<std::endl;
              continue;
          }

          _file<<">"<<std::endl;
          //member types
          if( (*it).second.ContentAccessed && !(*it).second.is<std::string>() ) _file<<levelIndent(_level+1)<<writeMemberType( (*it).second )<<std::endl;

          //sublevels
          writeToXML( _file, &(*it).second, _level+1 );

          _file<<levelIndent(_level)<<"</"<<(*it).first<<">"<<std::endl; //close level
      }
  }

  TEMPT
  std::string GenericMultiLevelMap<KEY_TYPE>::levelIndent( int _level ) const
  {
      std::string indent="";
      //level indent
      for( int i=0;i<_level;i++)
      {
          indent+="	";
      }
      return indent;
  }

  TEMPT
  std::string GenericMultiLevelMap<KEY_TYPE>::writeMemberType( GenericMultiLevelMap<KEY_TYPE>& _map )
  {
      std::string output="<";
      //check every type
      if( _map.is<bool>() ) output+="bool value=\""+toString(&_map)+"\" />";
      else if( _map.is<int>() ) output+="int value=\""+toString(&_map)+"\" />";
      else if( _map.is<double>() ) output+="double value=\""+toString(&_map)+"\" />";
      else if( _map.is<float>() ) output+="float value=\""+toString(&_map)+"\" />";
      else if( _map.is<char>() ) output+="char value=\""+toString(&_map)+"\" />";
      else if( _map.is<unsigned int>() ) output+="unsignedint value=\""+toString(&_map)+"\" />";
      else output+="string value=\""+toString(&_map)+"\" />";

      return output;
  }

  TEMPT
  std::string GenericMultiLevelMap<KEY_TYPE>::toString( GenericMultiLevelMap<KEY_TYPE>* _map )
  {
      std::string output;
      std::stringstream buffer;

      std::string caseSwitch = (*_map->Content.typeInfo()).name();

      if( caseSwitch == typeid(std::string).name() ) buffer << _map->as<std::string>();
      else if( caseSwitch == typeid(bool).name() ) buffer << _map->as<bool>();
      else if( caseSwitch == typeid(int).name() ) buffer << _map->as<int>();
      else if( caseSwitch == typeid(double).name() ) buffer << _map->as<double>();
      else if( caseSwitch == typeid(float).name() ) buffer << _map->as<float>();
      else if( caseSwitch == typeid(char).name() ) buffer << _map->as<char>();
      else if( caseSwitch == typeid(unsigned int).name() ) buffer << _map->as<unsigned int>();
      else buffer << _map->as<std::string>();

      std::string temp;
      while( getline(buffer,temp) ) output+=temp;

      return output;
  }

  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::loadXMLLayer( ticpp::Node const* _actualElement, GenericMultiLevelMap<KEY_TYPE>& _actualMapPosition )
  {
    //iterate through attributes if node is an element
    if( _actualElement->Type()==TiXmlNode::ELEMENT )
    {
      ticpp::Element const* asElement=static_cast<ticpp::Element const*>(_actualElement);
      ticpp::Iterator<ticpp::Attribute> attribute_it;
      for( attribute_it=attribute_it.begin( asElement ); attribute_it!=attribute_it.end(); ++attribute_it )
      {
        if( attribute_it->Name()=="value" )
        {
            getCastTable()["string"]( attribute_it->Value(),_actualMapPosition );
        }
        else _actualMapPosition["ATTRIBUTE"][attribute_it->Name()].as<std::string>()=attribute_it->Value();
      }
    }

    //iterate through child nodes
    ticpp::Iterator<ticpp::Node> it;
    for( it=it.begin( _actualElement ); it!=it.end(); ++it )
    {
      if( it->Type()==TiXmlNode::TEXT ) //if the actual node is of type TEXT ->see tinyxml:NodeType definition
      {
        getCastTable()["string"]( it->Value(),_actualMapPosition ); //casts and writes the string
      }
      else if( it->Type()!=TiXmlNode::ELEMENT ) continue; //if the actual node is not of type ELEMENT
      else
      {
        ticpp::Element const* _myElement=static_cast<ticpp::Element const*>( it.Get() );

        if( nameIsAttribute( _myElement ) ){} // ignore the node if its name is "ATTRIBUTE"
        else if( isMemberType( _myElement ) ) // if the node name is a member type, save as the value of the node
        {
              if( _myElement->HasAttribute("value") )
              {
                std::string valueContent=_myElement->GetAttribute("value");
                getCastTable()[ _myElement->Value() ]( valueContent, _actualMapPosition ); //write it into the map
              }
        }
        else //routine for normal entities
        {
          loadXMLLayer( it.Get(), _actualMapPosition[ it->Value() ] ); //recursive call for the next sub layer
        }
      }
    }

    return;
  }



  TEMPT
  bool GenericMultiLevelMap<KEY_TYPE>::isMemberType( ticpp::Element const* _element )
  {
    if( _element==NULL ) return false;
    if( getCastTable().count( _element->Value() )==1 ) return true;
    return false;
  }



  TEMPT
  bool GenericMultiLevelMap<KEY_TYPE>::nameIsAttribute( ticpp::Element const* _element )
  {
    if( _element== NULL ) return false;
    if( _element->Value()=="ATTRIBUTE" ) return true;
    return false;
  }



  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::registerCast( std::string _memberTypeName,void (*_castFunction)( std::string,GenericMultiLevelMap<KEY_TYPE>& ) )
  {
    getCastTable()[_memberTypeName]=_castFunction;
    return;
  }



  TEMPT
  std::map<std::string,void(*)( std::string,GenericMultiLevelMap<KEY_TYPE>& )>& GenericMultiLevelMap<KEY_TYPE>::getCastTable()
  {
    if( stm_CastTable==NULL ) stm_CastTable=initCastTable();
    return *stm_CastTable;
  }



  TEMPT
  std::map<std::string,void(*)( std::string,GenericMultiLevelMap<KEY_TYPE>& )>* GenericMultiLevelMap<KEY_TYPE>::initCastTable()
  {
    std::map<std::string,void(*)( std::string,GenericMultiLevelMap<KEY_TYPE>& )>* myCastTable=new std::map<std::string,void(*)( std::string,GenericMultiLevelMap<KEY_TYPE>& )>;
    (*myCastTable)["string"]=&stringCast;
    (*myCastTable)["bool"]=&boolCast;
    (*myCastTable)["int"]=&intCast;
    (*myCastTable)["unsignedint"]=&unsignedIntCast;
    (*myCastTable)["double"]=&doubleCast;
    (*myCastTable)["float"]=&floatCast;
    (*myCastTable)["char"]=&charCast;
    return myCastTable;
  }



  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::stringCast( std::string _input, GenericMultiLevelMap<KEY_TYPE>& _target )
  {
    _target.as<std::string>()=_input;
    return;
  }



  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::boolCast( std::string _input, GenericMultiLevelMap<KEY_TYPE>& _target )
  {
    bool value;
    if( _input=="false" || _input=="0" ) value=false;
    else if( _input=="true" || _input=="1" ) value=true;
    else
    {
      std::stringstream message;
      message<<"Couldn't cast std::string <<"<<_input<<">> to bool.";
      throw std::logic_error( message.str() );
    }

    _target.as<bool>()=value;
    return;
  }



  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::intCast( std::string _input, GenericMultiLevelMap<KEY_TYPE>& _target )
  {
    std::stringstream converter;
    int value;

    converter<<_input;
    converter>>value;

    if( converter.fail() )
    {
      std::stringstream message;
      message<<"Couldn't cast std::string <<"<<_input<<">> to int.";
      throw std::logic_error( message.str() );
    }
    _target.as<int>()=value;
    return;
  }



  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::unsignedIntCast( std::string _input, GenericMultiLevelMap<KEY_TYPE>& _target )
  {
    std::stringstream converter;
    unsigned int value;

    converter<<_input;
    converter>>value;

    if( converter.fail() )
    {
      std::stringstream message;
      message<<"Couldn't cast std::string <<"<<_input<<">> to unsigned int.";
      throw std::logic_error( message.str() );
    }
    _target.as<unsigned int>()=value;
    return;
  }



  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::doubleCast( std::string _input, GenericMultiLevelMap<KEY_TYPE>& _target )
  {
    std::stringstream converter;
    double value;

    converter<<_input;
    converter>>value;

    if( converter.fail() )
    {
      std::stringstream message;
      message<<"Couldn't cast std::string <<"<<_input<<">> to double.";
      throw std::logic_error( message.str() );
    }

    _target.as<double>()=value;
    return;
  }



  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::floatCast( std::string _input, GenericMultiLevelMap<KEY_TYPE>& _target )
  {
    std::stringstream converter;
    float value;

    converter<<_input;
    converter>>value;

    if( converter.fail() )
    {
      std::stringstream message;
      message<<"Couldn't cast std::string <<"<<_input<<">> to float.";
      throw std::logic_error( message.str() );
    }

    _target.as<float>()=value;
    return;
  }



  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::charCast( std::string _input, GenericMultiLevelMap<KEY_TYPE>& _target )
  {
    if( _input.size()>1 )
    {
      std::stringstream message;
      message<<"Couldn't cast std::string <<"<<_input<<">> to one char.";
      throw std::logic_error( message.str() );
    }

    _target.as<char>()=_input[0];
    return;
  }



  TEMPT
  GenericMultiLevelMap<KEY_TYPE>& GenericMultiLevelMap<KEY_TYPE>::operator[]( KEY_TYPE _key )
  {
    return NextLevel[_key];
  }



  TEMPT
  const GenericMultiLevelMap<KEY_TYPE>& GenericMultiLevelMap<KEY_TYPE>::operator[]( KEY_TYPE _key ) const
  {
    if( !hasKey(_key) ) return *this;
    return NextLevel[_key];
  }



  TEMPT
  typename GenericMultiLevelMap<KEY_TYPE>::iterator GenericMultiLevelMap<KEY_TYPE>::begin()
  {
    typename std::map< KEY_TYPE,GenericMultiLevelMap<KEY_TYPE> >::iterator it;
    it=NextLevel.begin();
    return iterator( it );
  }



  TEMPT
  typename GenericMultiLevelMap<KEY_TYPE>::const_iterator GenericMultiLevelMap<KEY_TYPE>::begin() const
  {
    typename std::map< KEY_TYPE,GenericMultiLevelMap<KEY_TYPE> >::const_iterator it;
    it=NextLevel.begin();
    return const_iterator( it );
  }


  TEMPT
  typename GenericMultiLevelMap<KEY_TYPE>::iterator GenericMultiLevelMap<KEY_TYPE>::end()
  {
    typename std::map< KEY_TYPE,GenericMultiLevelMap<KEY_TYPE> >::iterator it;
    it=NextLevel.end();
    return iterator(it);
  }


  TEMPT
  typename GenericMultiLevelMap<KEY_TYPE>::const_iterator GenericMultiLevelMap<KEY_TYPE>::end() const
  {
    typename std::map< KEY_TYPE,GenericMultiLevelMap<KEY_TYPE> >::const_iterator it;
    it=NextLevel.end();
    return const_iterator( it );
  }


  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::mirrorTwins( const GenericMultiLevelMap<KEY_TYPE>& _toMirror, int _flagSetting )
  {
    mirrorTwinsSub( *this, _toMirror, _flagSetting ); //if not type secure

    return;
  }


  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::mirrorTwinsSub( GenericMultiLevelMap<KEY_TYPE>& _targetMap, const GenericMultiLevelMap<KEY_TYPE>& _sourceMap, int _flagSetting )
  {
    bool skip=false;

    bool ignoreEmptyString=( (_flagSetting&IGNOREEMPTYSTRING)!=0 );
    bool typeSecure=( (_flagSetting&TYPESECURE)!=0 );
    bool throwExcep=( (_flagSetting&THROWEXCEP)!=0 );

    GenericType myContent=_targetMap.Content;

    if( ignoreEmptyString&&_sourceMap.is<std::string>() ){ if( _sourceMap.as<std::string>()=="" ) skip=true; }

    if( typeSecure && ( myContent.typeInfo()!=_sourceMap.Content.typeInfo() ) )
    {
      if( !throwExcep ) skip=true;
      else throw std::logic_error("GMLM::mirrorTwins(...) - error: two twin pairs do not have the same type. Mirroring aborted.");
    }

    if( !skip )_targetMap.Content=_sourceMap.Content;

    typename GenericMultiLevelMap<KEY_TYPE>::iterator it;
    for( it=_targetMap.begin(); it!=_targetMap.end(); ++it )
    {
      if( _sourceMap.count(it->first)==1 )
      {
        mirrorTwinsSub( it->second, _sourceMap.NextLevel.find(it->first)->second, _flagSetting );
      }
    }
    return;
  }



  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::print() const
  {
    std::cout<<std::endl<<"GenericMultiLevelMap - Standard print - Content: ";
    printHelper( (*this), 1 );
    std::cout<<std::endl;
  }


  TEMPT
  void GenericMultiLevelMap<KEY_TYPE>::printHelper( const GenericMultiLevelMap<KEY_TYPE>& _toPrint, unsigned int _levelIndicator ) const
  {
    _toPrint.Content.print();
    std::cout<<std::endl;

    GenericMultiLevelMap<KEY_TYPE>::const_iterator it;
    for( it=_toPrint.begin(); it!=_toPrint.end(); ++it )
    {
      for( int i=0; i<_levelIndicator; ++i ) std::cout<<"  ";
      std::cout<<it->first<<": ";
      printHelper( it->second, _levelIndicator+1 );
    }
  }


  #undef	TEMPT

}

#endif
