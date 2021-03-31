final color kCOLOR_BONE  = color(40, 200, 110);
final color kCOLOR_JOINT = color(255, 255, 255);
final color kCOLOR_BODY  = color(50, 150, 220);
final color kCOLOR_HEAD  = color(255, 0, 0);
final float kLEG_WIDTH   = 5.0f;

/*
             TOP VIEW
               front
             ---------
  135deg    / 6     1 \         45 deg
 -180deg    | 5  +  2 | right    0 deg
 -135deg    \ 4     3 /        -45 deg
             ---------
*/

class Leg {
    float _coxaLength;
    float _femurLength;
    float _tibiaLength;

    Vector _vOffFromCenter = new Vector();
    Vector _vInitPos       = new Vector();
    Vector _vPos           = new Vector();
    float  _fCoxaOffset;    
    
    float _angleCoxa;
    float _angleFemur;
    float _angleTibia;

    int   _pos;
    boolean _isDebug;

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

    void enableDebug(boolean en) { 
        _isDebug = en; 
    }
    
    boolean isDebugEnabled() {
        return _isDebug;
    }

    Leg(int pos, float bodyFrontWidth, float bodyHeight, float bodyMiddleWidth, 
        float coxaLength, float femurLength, float tibiaLength) {
        _pos         = pos;
        _isDebug     = false;
        _coxaLength  = coxaLength;
        _femurLength = femurLength;
        _tibiaLength = tibiaLength;

        int   signY;
        int   signX = (pos < 3) ? 1 : -1;
        float offsetX;
        float offsetY;
        
        if (_pos < 3) {
            _fCoxaOffset = 45.0f - (_pos * 45.0f);
        } else {
            _fCoxaOffset = -90.0f - ((_pos - 2) * 45.0f);
        }
        _fCoxaOffset = fixAngle(_fCoxaOffset);
        if (_pos == 1 || _pos == 4) {
            offsetX = bodyMiddleWidth / 2;
            offsetY = 0;
            signY   = 0;
        } else {
            offsetX = bodyFrontWidth  / 2;
            offsetY = bodyHeight / 2;
            signY   = (_pos == 0 || _pos == 5) ? 1 : -1;
        }
        _vOffFromCenter.x = signX * offsetX;
        _vOffFromCenter.y = signY * offsetY;
        _vOffFromCenter.z = tibiaLength;

        _vInitPos.x = roundUp(cos(radians(_fCoxaOffset)) * (_coxaLength + _femurLength));
        _vInitPos.y = roundUp(sin(radians(_fCoxaOffset)) * (_coxaLength + _femurLength));
        _vInitPos.z = roundUp(tibiaLength);

        _vPos.set(_vInitPos);
        if (true) { //_isDebug) {
            println(String.format("init   leg:%d, angle:%6.1f, off from center (%6.1f, %6.1f)", pos, 
                _fCoxaOffset, _vOffFromCenter.x, _vOffFromCenter.y));
            println(String.format("init   leg:%d, %6.1f, %6.1f, %6.1f", pos, 
                _vPos.x, _vPos.y, _vPos.z));
        }
    }

    Vector bodyIK(Vector vMov, Rotator rRot) {
        // body ik
        float totX    = _vInitPos.x + _vOffFromCenter.x + vMov.x;
        float totY    = _vInitPos.y + _vOffFromCenter.y + vMov.y;

        // yaw
        float alpha_0 = atan2(totY, totX);
        float alpha_1 = alpha_0 + radians(rRot.yaw);

        float dist   = sqrt(sq(totX) + sq(totY));
        float bodyIKX = roundUp(cos(alpha_1) * dist - totX);
        float bodyIKY = roundUp(sin(alpha_1) * dist - totY);

        // pitch, roll
        float rollZ  = tan(radians(rRot.roll)) * totX;
        float pitchZ = tan(radians(rRot.pitch)) * totY;
        float bodyIKZ = roundUp(rollZ + pitchZ);

        //float rot = radians(yaw);
        //bodyIKX = bodyIKX * cos(rot) - bodyIKY * sin(rot);
        //bodyIKY = bodyIKX * sin(rot) + bodyIKY * cos(rot);

        //println(String.format("totxy  leg:%d, %6.1f, %6.1f, dist:%6.1f, deg:%6.1f", _pos, totX, totY, dist, degrees(alpha_1)));
        //println(String.format("bodyik leg:%d, %6.1f, %6.1f, %6.1f", _pos, bodyIKX, bodyIKY, bodyIKZ));

        _vPos.set(_vInitPos.x + vMov.x + bodyIKX, _vInitPos.y + vMov.y + bodyIKY, _vInitPos.z + vMov.z - bodyIKZ);
        
        if (_isDebug) {
            println(String.format("newpos leg:%d, %6.1f, %6.1f, %6.1f", _pos, _vPos.x, _vPos.y, _vPos.z));
        }
        
        return _vPos;
    }

    void legIK(Vector tgt) {
        float distFoot   = sqrt(sq(tgt.x) + sq(tgt.y));
        float diagonal   = sqrt(sq(distFoot - _coxaLength) + sq(tgt.z)); 
        float a1         = acos(tgt.z / diagonal); //atan((distFoot - _coxaLength) / tgt.z);
        float a2         = acos((sq(_tibiaLength) - sq(_femurLength) - sq(diagonal)) / (-2 * _femurLength * diagonal));
        float angleTibia = acos((sq(diagonal) - sq(_tibiaLength) - sq(_femurLength)) / (-2 * _femurLength * _tibiaLength));

        float ac = degrees(atan2(tgt.y, tgt.x));
        float af = degrees(a1 + a2);
        float at = degrees(angleTibia);

        if (Float.isNaN(ac) || Float.isNaN(af) || Float.isNaN(at)) {
            println(String.format("ERROR  leg:%d, %6.1f, %6.1f, %6.1f\n", _pos, ac, af, at));            
            return;
        }

        _angleCoxa  = fixAngle((ac - _fCoxaOffset));    // zero base
        //_angleCoxa  = fixAngle((ac - _fCoxaOffset) * (_pos < 3 ? -1 : 1));  // zero base for servo
        _angleFemur = -fixAngle(90 - af);    // zero base
        _angleTibia = -fixAngle(90 - at);    // zero base

        if (_isDebug) {
            println(String.format("angle  leg:%d, %6.1f, %6.1f, %6.1f\n", _pos, _angleCoxa, _angleFemur, _angleTibia));
        }
    }

    void move(Vector vMov, Rotator rRot) {
        Vector tgt = bodyIK(vMov, rRot);
        legIK(tgt);
    }

/*
    void moveBody(Vector vMov, Vector rRot) {
        println(String.format("pos    leg:%d, %6.1f, %6.1f, %6.1f", _pos, _vPos.x, _vPos.y, _vPos.z));

        // inverted Y
        Vector tgt = new Vector(_vPos.x - vMov.x, _vPos.y + vMov.y, _vPos.z + vMov.z);
        println(String.format("distmv leg:%d, %6.1f, %6.1f, %6.1f", _pos, tgt.x, tgt.y, tgt.z));

        //float y = tgt.y * cos(a) - tgt.z * sin(a);
        //float z = tgt.y * sin(a) + tgt.z * cos(a);
        //tgt.y = y; //_vPos.y + (y - tgt.y) * 2;
        //tgt.z = tgt.z + (z - tgt.z) * 2;

        float p = tan(radians(-rRot.x)) * tgt.y;
        float r = tan(radians(-rRot.y)) * tgt.x;
        
        tgt.z = tgt.z + (p + r) * 2;
        println(String.format("newpos leg:%d, %6.1f, %6.1f, %6.1f\n", _pos, tgt.x, tgt.y, tgt.z));

        legIK(tgt);
    }
*/

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
        jointZ(_vOffFromCenter.x, -_vOffFromCenter.y, _vOffFromCenter.z, -90 - (_angleCoxa + _fCoxaOffset), 
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
    Vector   _vBodyPos;
    Vector   _vMov;
    Rotator  _rRot;    // x-> pitch, y->roll, z->yaw
    Gait      _gait;
    boolean   _isWalk;

    int       _debugLegMask = 0x01;

    HexaPod(float bodyFrontWidth, float bodyHeight, float bodyMiddleWidth, float coxaLen, float femurLen, float tibiaLen) {
        _legs      = new Leg[6];
        _vBodyPos  = new Vector(0, 0, 0);
        _vMov      = new Vector(0, 0, 0);
        _rRot      = new Rotator(0, 0, 0);
        _debugLegMask = (1 << 3);

        int mask;
        for (int i = 0; i < _legs.length; i++) {
            mask = 1 << i;
            _legs[i] = new Leg(i, bodyFrontWidth, bodyHeight, bodyMiddleWidth, coxaLen, femurLen, tibiaLen);
            _legs[i].enableDebug((_debugLegMask & mask) == mask);
            _legs[i].move(_vMov, _rRot);
        }
        _gait = new GaitRipple();
        println("-----------------------");
    }

    void draw() {
        // body
        pushMatrix();
        translate(_vBodyPos.x, _vBodyPos.y, _vBodyPos.z + _vMov.z);
        rotateX(radians(_rRot.pitch));
        rotateY(radians(_rRot.roll));
        if (!_isWalk) {
            rotateZ(radians(_rRot.yaw));
        }
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
        for (int i = 0; i < _legs.length; i++) {
            _legs[i].draw();
        }
        popMatrix();
    }

    Vector getPos() {
        return _vMov;
    }

    Rotator getRot() {
        return _rRot;
    }

    void update(boolean isWalk) {
        //println("_vBodyPos = " + _vBodyPos.toString());
        _vMov.dump("_vMov");
        _rRot.dump("_rRot");

        _isWalk = isWalk;
        if (isWalk) {
            int    sign;
            float  yaw = _rRot.yaw;
            
            for (int i = 0; i < _legs.length; i++) {
                sign = (i < 3) ? 1 : -1;
                Vector dir = new Vector(
                    Gait.kPRECISION + _vMov.x + ((yaw / 8) *  sign), 
                    Gait.kPRECISION + _vMov.y + ((yaw / 8) * -sign), 
                    _vMov.z);
    
                Vector vMove = _gait.doStep(i, 60, dir, _rRot);
                if (_legs[i].isDebugEnabled()) {
                    println(String.format("dir    leg:%d, %6.1f, %6.1f, %6.1f", i, dir.x, dir.y, dir.z));
                    println(String.format("movpos leg:%d, %6.1f, %6.1f, %6.1f", i, vMove.x, vMove.y, vMove.z));
                }
                _legs[i].move(vMove, _rRot);
            }
            
            _rRot.yaw = yaw;
        } else {
            for (int i = 0; i < _legs.length; i++) {
                _legs[i].move(_vMov, _rRot);
                //    _legs[i].moveBody(_vBodyPos, _rRot);
            }
        }
        println("-----------------------");
    }
}
