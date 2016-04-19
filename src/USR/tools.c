
float max(float a, float b){ return a>b?a:b; }

float min(float a, float b){ return a<b?a:b; }

int limit(int x, int lmt) {
	if(x>lmt) return lmt;
	if(x<-lmt) return -lmt;
	return x;
}
