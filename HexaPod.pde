final static float kTBL_LEG_SIGN[][] = {
    // x  y
    {  1, 1 }, 
    {  1, 0 }, 
    {  1, -1 }, 
    { -1, -1 }, 
    { -1, 0 }, 
    { -1, 1 }
};

final color kCOLOR_BONE  = color(40, 200, 110);
final color kCOLOR_JOINT = color(255, 255, 255);
final color kCOLOR_BODY  = color(50, 150, 220);
final color kCOLOR_HEAD  = color(255, 0, 0);
final float kLEG_WIDTH   = 5.0f;

class Leg {
    float _coxaLength;
    float _femurLength;
    float _tibiaLength;

    PVector _vOffFromCenter = new PVector();
    PVector _vInitPos       = new PVector();
    PVector _vPos           = new PVector();

    float _angleCoxa;
    float _angleFemur;
    float _angleTibia;

    int   _pos;

    float fixAngle(float angle) {
        if (angle < -180) {
            angle += 360;
        } else if (angle > 180) {
            angle -= 360;
        }
        return angle;
    }

    float getCoxaAngle() {
        float angle;

        if (_pos < 3) {
            angle = 45.0f - (_pos * 45.0f);
        } else {
            angle = -90.0f - ((_pos - 2) * 45.0f);
        }

        return fixAngle(angle);
    }

    float roundUp(float v) {
        return round(v * 100.0f) / 100.0f;
    }

    Leg(int pos, float bodyFrontWidth, float bodyHeight, float bodyMiddleWidth, 
        float coxaLength, float femurLength, float tibiaLength) {
        _pos = pos;
        _coxaLength    = coxaLength;
        _femurLength   = femurLength;
        _tibiaLength   = tibiaLength;

        float angle = getCoxaAngle();
        float offsetX;
        float offsetY;

        if (pos == 1 || pos == 4) {
            offsetX = bodyMiddleWidth / 2;
            offsetY = 0;
        } else {
            offsetX = bodyFrontWidth  / 2;
            offsetY = bodyHeight / 2;
        }
        _vOffFromCenter.x = kTBL_LEG_SIGN[pos][0] * offsetX;
        _vOffFromCenter.y = kTBL_LEG_SIGN[pos][1] * offsetY;
        _vOffFromCenter.z = tibiaLength;

        _vInitPos.x = roundUp(cos(radians(angle)) * (_coxaLength + _femurLength));
        _vInitPos.y = roundUp(sin(radians(angle)) * (_coxaLength + _femurLength));
        _vInitPos.z = roundUp(tibiaLength);

        _vPos.set(_vInitPos);
        println(String.format("init   leg:%d, angle:%6.1f, off from center (%6.1f, %6.1f)", pos, 
            angle, _vOffFromCenter.x, _vOffFromCenter.y));
        println(String.format("init   leg:%d, %6.1f, %6.1f, %6.1f", pos, 
            _vPos.x, _vPos.y, _vPos.z));
    }

    void bodyIK(PVector vMov, PVector vRot) {
        // body ik
        float totX    = _vInitPos.x + _vOffFromCenter.x + vMov.x;
        float totY    = _vInitPos.y + _vOffFromCenter.y + vMov.y;

        // yaw
        float alpha_0 = atan2(totY, totX);
        float alpha_1 = alpha_0 + radians(vRot.z);

        float dist   = sqrt(sq(totX) + sq(totY));
        float bodyIKX = roundUp(cos(alpha_1) * dist - totX);
        float bodyIKY = roundUp(sin(alpha_1) * dist - totY);

        // pitch, roll
        float rollZ  = tan(radians(vRot.y)) * totX;
        float pitchZ = tan(radians(vRot.x)) * totY;
        float bodyIKZ = roundUp(rollZ + pitchZ);

        println(String.format("totxy  leg:%d, %6.1f, %6.1f, dist:%6.1f, deg:%6.1f", _pos, totX, totY, dist, degrees(alpha_1)));
        println(String.format("bodyik leg:%d, %6.1f, %6.1f, %6.1f", _pos, bodyIKX, bodyIKY, bodyIKZ));

        // leg ik
        _vPos.set(_vInitPos.x + vMov.x + bodyIKX, _vInitPos.y + vMov.y + bodyIKY, _vInitPos.z + vMov.z - bodyIKZ);
        println(String.format("newpos leg:%d, %6.1f, %6.1f, %6.1f", _pos, _vPos.x, _vPos.y, _vPos.z));
    }

    void legIK(PVector tgt) {
        float L1      = sqrt(sq(tgt.x) + sq(tgt.y));            // coxafeetdist
        float L       = sqrt(sq(L1 - _coxaLength) + sq(tgt.z));   // iksw 
        float a1      = acos(tgt.z / L); //atan((L1 - _coxaLength) / tgt.z);
        float a2      = acos((sq(_tibiaLength) - sq(_femurLength) - sq(L)) / (-2 * _femurLength * L));
        float angleTibia = acos((sq(L) - sq(_tibiaLength) - sq(_femurLength)) / (-2 * _femurLength * _tibiaLength));

        float ac = degrees(atan2(tgt.y, tgt.x));
        float af = degrees(a1 + a2);
        float at = degrees(angleTibia);
        
        _angleCoxa  = fixAngle(ac);         // coxa location angle base
        _angleFemur = -fixAngle(90 - af);    // zero base
        _angleTibia = -fixAngle(90 - at);    // zero base
        println(String.format("angle  leg:%d, %6.1f, %6.1f, %6.1f\n", _pos, _angleCoxa, _angleFemur, _angleTibia));
    }

    void move(PVector vMov, PVector vRot) {
        bodyIK(vMov, vRot);
        legIK(_vPos);
    }


    void moveBody(PVector vMov, PVector vRot) {
        println(String.format("pos    leg:%d, %6.1f, %6.1f, %6.1f", _pos, _vPos.x, _vPos.y, _vPos.z));

        // inverted Y
        PVector tgt = new PVector(_vPos.x - vMov.x, _vPos.y + vMov.y, _vPos.z + vMov.z);
        println(String.format("distmv leg:%d, %6.1f, %6.1f, %6.1f", _pos, tgt.x, tgt.y, tgt.z));

        //float y = tgt.y * cos(a) - tgt.z * sin(a);
        //float z = tgt.y * sin(a) + tgt.z * cos(a);
        //tgt.y = y; //_vPos.y + (y - tgt.y) * 2;
        //tgt.z = tgt.z + (z - tgt.z) * 2;

        float p = tan(radians(-vRot.x)) * tgt.y;
        float r = tan(radians(-vRot.y)) * tgt.x;
        
        tgt.z = tgt.z + (p + r) * 2;
        println(String.format("newpos leg:%d, %6.1f, %6.1f, %6.1f\n", _pos, tgt.x, tgt.y, tgt.z));

        legIK(tgt);
    }

    private void jointX(float x, float y, float z, float a, color c) {
        translate(x, y, z);
        rotateX(radians(a));
        fill(c);
        sphere(kLEG_WIDTH); //> draws a sphere
    }

    private void jointZ(float x, float y, float z, float a, color c) {
        translate(x, y, z);
        rotateZ(radians(a));
        fill(c);
        sphere(kLEG_WIDTH);
    }

    void draw() {
        pushMatrix();

        //
        // y is inverted in the screen '-'
        //

        // hip
        jointZ(_vOffFromCenter.x, -_vOffFromCenter.y, _vOffFromCenter.z, -90 - _angleCoxa, 
               (_pos == 0 || _pos == 5) ? kCOLOR_HEAD : kCOLOR_JOINT);
        translate(0, _coxaLength / 2, 0);
        fill(kCOLOR_BONE);
        box(kLEG_WIDTH, _coxaLength, kLEG_WIDTH);

        // femur
        jointX(0, _coxaLength / 2, 0, _angleFemur, kCOLOR_JOINT);
        translate(0, _femurLength / 2, 0);
        fill(kCOLOR_BONE);
        box(kLEG_WIDTH, _femurLength, kLEG_WIDTH);

        // tibia
        jointX(0, _femurLength / 2, 0, _angleTibia - 90, kCOLOR_JOINT);
        translate(0, _tibiaLength / 2, 0);
        fill(kCOLOR_BONE);
        box(kLEG_WIDTH, _tibiaLength, kLEG_WIDTH);

        jointX(0, _tibiaLength / 2, 0, 0, kCOLOR_JOINT);

        popMatrix();
    }
}

class HexaPod {
    Leg       _legs[];
    PVector   _vBodyPos;
    PVector   _vMov;
    PVector   _vRot;    // x-> pitch, y->roll, z->yaw
    Gait      _gait;

    int       _debug = 0;
    int       _debug_legs = (_debug == 0) ? 6 : 1;

    HexaPod(float bodyFrontWidth, float bodyHeight, float bodyMiddleWidth, float coxaLen, float femurLen, float tibiaLen) {
        _legs      = new Leg[6];
        _vBodyPos  = new PVector(0, 0, 0);
        _vMov      = new PVector(0, 0, 0);
        _vRot      = new PVector(0, 0, 0);

        for (int i = 0; i < _legs.length; i++) {
            _legs[i] = new Leg(i, bodyFrontWidth, bodyHeight, bodyMiddleWidth, coxaLen, femurLen, tibiaLen);
            _legs[i].move(_vMov, _vRot);
        }
        _gait = new GaitTrot();
        println("-----------------------");
    }

    void draw() {
        // body
        pushMatrix();
        translate(_vBodyPos.x, _vBodyPos.y, _vBodyPos.z + _vMov.z);
        rotateX(radians(_vRot.x));
        rotateY(radians(_vRot.y));
        rotateZ(radians(_vRot.z));
        strokeWeight(15);
        stroke(kCOLOR_BODY);
        noFill();
        beginShape();
        for (int i = 0; i <= _legs.length; i++) {
            vertex(_legs[i % _legs.length]._vOffFromCenter.x, _legs[i % _legs.length]._vOffFromCenter.y, 
                _legs[i % _legs.length]._vOffFromCenter.z);
        }
        endShape();
        noStroke();

        // legs
        for (int i = 0; i < _debug_legs; i++) { //_legs.length; i++) {
            _legs[i].draw();
        }

        popMatrix();
    }

    PVector getPos() {
        return (_debug == 2) ? _vMov : _vBodyPos;
    }

    PVector getRot() {
        return _vRot;
    }

    void update() {
        println("_vBodyPos = " + _vBodyPos.toString());
        println("_vMov     = " + _vMov.toString());
        println("_vRot     = " + _vRot.toString());
        for (int i = 0; i < _debug_legs; i++) {
            if (_debug == 2)
                _legs[i].move(_vMov, _vRot);
            else
                _legs[i].moveBody(_vBodyPos, _vRot);
        }
        println("-----------------------");
    }


    //final static float kTBL_LEG_SIGN[][] = {
    //    {  1, 1 }, 
    //    {  1, 0 }, 
    //    {  1, -1 }, 
    //    { -1, -1 }, 
    //    { -1, 0 }, 
    //    { -1, 1 }
    //};    

    void walk() {
        for (int i = 0; i < 3; i++) { //_legs.length; i++) {
            PVector dir = new PVector(
                Gait.kPRECISION - _vMov.x + _vRot.z * kTBL_LEG_SIGN[i][0], 
                Gait.kPRECISION - _vMov.y + _vRot.z * kTBL_LEG_SIGN[i][0], 
                _vMov.z);

            PVector vMove = _gait.doStep(i, 60, dir, _vRot, 1); //kTBL_LEG_SIGN[i][0]);
            println(String.format("movpos leg:0, %6.1f, %6.1f, %6.1f", vMove.x, vMove.y, vMove.z));
            _legs[i].move(vMove, _vRot);
        }
    }
}
