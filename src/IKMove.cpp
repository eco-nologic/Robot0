#include "IKMove.h"

String IKMove::toCommand() const {
    return "IK_MOVE:" + String(tg) + "," + String(td) + "," +
           String(vg) + "," + String(vd) + "," +
           String(dg, 2) + "," + String(dd, 2);
}

IKMove IKMove::fromParams(const String& params) {
    // Parse "45,38,80,80,0.00,0.00"
    int idx[5];
    idx[0] = params.indexOf(',');
    for (int i = 1; i < 5; i++)
        idx[i] = params.indexOf(',', idx[i-1] + 1);

    IKMove m;
    m.tg = params.substring(0,         idx[0]).toInt();
    m.td = params.substring(idx[0]+1,  idx[1]).toInt();
    m.vg = params.substring(idx[1]+1,  idx[2]).toInt();
    m.vd = params.substring(idx[2]+1,  idx[3]).toInt();
    m.dg = params.substring(idx[3]+1,  idx[4]).toFloat();
    m.dd = params.substring(idx[4]+1        ).toFloat();
    return m;
}
