/* SPDX-License-Identifier: MIT */
#include <stdio.h>
#include <iqs915x_processing_math.h>
int main(void) {
    puts("[");
    for(unsigned mask=0;mask<64;mask++) {
        struct iqs915x_processing_settings p={.coefficient_permille=mask&1?900:-900,.x_middle=mask&2?900:-900,.x_lower=mask&4?900:-900,
            .y_upper=mask&8?900:-900,.y_lower=mask&16?900:-900,.y_center=mask&32?500:-500,.top_speed=1000};
        int16_t map[5][5][2];iqs915x_map_generate(&p,map);
        printf("{\"controls\":[%d,%d,%d,%d,%d,%d],\"nodes\":[",p.coefficient_permille,p.x_middle,p.x_lower,p.y_upper,p.y_lower,p.y_center);
        for(unsigned j=0;j<5;j++){printf("%s[",j?",":"");for(unsigned i=0;i<5;i++)printf("%s[%d,%d]",i?",":"",map[j][i][0],map[j][i][1]);printf("]");}
        printf("]}%s\n",mask<63?",":"");
    }
    puts("]");
}
