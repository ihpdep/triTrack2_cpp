// mex.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <string.h>
#include "matcher.h"
#include "matcherMex.hpp"

using namespace std;

static Matcher *M;

int _tmain(int argc, _TCHAR* argv[])
{
char prhs[10];
Matcher::parameters param;
matcherMex("init",param);

	return 0;
}

