/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stddef.h>
#include <iqs915x_calibration_math.h>
int main(void) {
    struct iqs915x_calibration_grid g = {0};
    _Static_assert(sizeof(g) == 884, "map payload");
    for (unsigned x=0; x<=1024; ++x) for (unsigned y=0; y<=768; ++y) {
        uint16_t a=x,b=y;
        assert(iqs915x_cal_apply(&g,1024,768,&a,&b));
        assert(a==x && b==y);
    }
    for (unsigned i=0; i<IQS915X_CAL_NODES; ++i) {
        g.dx[i]=-7; g.dy[i]=9;
    }
    uint16_t x=500,y=300;
    assert(iqs915x_cal_apply(&g,1024,768,&x,&y));
    assert(x==493 && y==309);
    x=1024;y=768;
    assert(iqs915x_cal_apply(&g,1024,768,&x,&y));
    assert(x==1017 && y==768);
    x=0;y=0;
    assert(iqs915x_cal_apply(&g,1024,768,&x,&y));
    assert(x==0 && y==9);
    x=UINT16_MAX;y=1;
    assert(!iqs915x_cal_apply(&g,1024,768,&x,&y));
    assert(x==UINT16_MAX && y==1);
    for (unsigned i=0;i<IQS915X_CAL_NODES;++i) {
        g.dx[i]= i%2 ? INT16_MAX : INT16_MIN;
        g.dy[i]= i%2 ? INT16_MIN : INT16_MAX;
    }
    for (unsigned a=0;a<=65534;a+=127) {
        x=a;y=65534-a;
        assert(iqs915x_cal_apply(&g,65534,65534,&x,&y));
        assert(x<=65534 && y<=65534);
    }
    g.dx[0]=0;g.dx[1]=1;g.dx[17]=0;g.dx[18]=1;
    assert(iqs915x_cal_interpolate(g.dx,0,2048,0)==1);
    g.dx[1]=-1;g.dx[18]=-1;
    assert(iqs915x_cal_interpolate(g.dx,0,2048,0)==-1);
    return 0;
}
