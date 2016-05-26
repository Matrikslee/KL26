
#include "tools.h"

float flimit(float x, float lmt) {
	if(x>lmt) return lmt;
	if(x<-lmt) return -lmt;
	return x;
}

int limit(int x, int lmt) {
	if(x>lmt) return lmt;
	if(x<-lmt) return -lmt;
	return x;
}
