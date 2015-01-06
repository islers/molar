/*Copyright (c) 2014, Stefan Isler, islerstefan@bluewin.ch
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



template<class T=double>
class Average
{
public:
	Average(void);
	Average(T);
	~Average(void);

	Average& operator=( Average _el );
	Average& operator=( T _el );

	bool operator==( Average _el );
	bool operator==( T _el );

	bool operator<( Average _el );
	bool operator<( T _el );

	bool operator>( Average _el );
	bool operator>( T _el );

	bool operator<=( Average _el );
	bool operator<=( T _el );

	bool operator>=( Average _el );
	bool operator>=( T _el );

	operator T();

	void reset();
	void deleteContent();
private:
	T pValue;
	unsigned int pCount;

};



template<class T>
Average<T>::Average(void)
{
	pValue = T(0);
	pCount=0;
}

template<class T>
Average<T>::Average(T _first )
{
	pCount=1;
	pValue=_first;
}

template<class T>
Average<T>::~Average(void)
{
}

template<class T>
Average<T>& Average<T>::operator=( Average<T> _el )
{
	pValue = ((double)( pCount*pValue+_el.pCount*_el.pValue ))/( pCount+_el.pCount );
	pCount += _el.pCount;
	return *this;
}

template<class T>
Average<T>& Average<T>::operator=( T _el )
{
	pValue = ((double)(pCount*pValue + _el ))/( pCount+1 );
	pCount += 1;
	return *this;
}

template<class T>
bool Average<T>::operator==( Average<T> _el )
{
	return pValue==_el.pValue;
}

template<class T>
bool Average<T>::operator==( T _el )
{
	return pValue==_el;
}

template<class T>
bool Average<T>::operator<( Average<T> _el )
{
	return pValue<_el.pValue;
}

template<class T>
bool Average<T>::operator<( T _el )
{
	return pValue<_el;
}

template<class T>
bool Average<T>::operator>( Average<T> _el )
{
	return pValue>_el.pValue;
}

template<class T>
bool Average<T>::operator>( T _el )
{
	return pValue>_el;
}

template<class T>
bool Average<T>::operator<=( Average<T> _el )
{
	return pValue<=_el.pValue;
}

template<class T>
bool Average<T>::operator<=( T _el )
{
	return pValue<=_el;
}

template<class T>
bool Average<T>::operator>=( Average<T> _el )
{
	return pValue>=_el.pValue;
}

template<class T>
bool Average<T>::operator>=( T _el )
{
	return pValue>=_el;
}

template<class T>
Average<T>::operator T()
{
	return pValue;
}

template<class T>
void Average<T>::reset()
{
	pCount=1;
}

template<class T>
void Average<T>::deleteContent()
{
	pCount=0;
	pValue=T(0);
}
