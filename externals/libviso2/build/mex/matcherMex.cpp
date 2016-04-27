/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/


#include <iostream>
#include <string.h>
#include "matcher.h"
#include "matrix.h"


using namespace std;

static Matcher *M;

template<class T> T* transpose(T* I,const int32_t* dims) {
  T* I_ = (T*)malloc(dims[0]*dims[1]*sizeof(T));
  for (int32_t u=0; u<dims[1]; u++) {
    for (int32_t v=0; v<dims[0]; v++) {
      I_[v*dims[1]+u] = *I;
      I++;
    }
  }
  return I_;
}

void matcherMex(char prhs[],Matcher::parameters param1,Matcher::parameters param2) {
  if (!strcmp(prhs,"init")) {
	  param2=param1;
	  M = new Matcher(param2);
   // close
  } else if (!strcmp(prhs,"close")) {
    delete M;
  }
}

  void matcherMex2(char prhs[],int q){
	   if (!strcmp(prhs,"push") || !strcmp(prhs,"replace")) {
	}
  }