boolean   _isCamLog = false;
PVector   _origin;
HexaPod   _hexapod;

class CamCtrl {
    int        _mode;
    PVector    _eye;
    PVector    _center;
    PVector    _up;
    PVector    _rot;

    PVector  _tblCamFix[][] = {
        { new PVector(   0,   0, 400), new PVector( 0, 0, 0), new PVector(0, 0, 0) },     // up
        { new PVector(   0, 400,   0), new PVector(10, 0, 0), new PVector(-90, 0,  90) }, // right
        { new PVector(   0, 400,   0), new PVector(10, 0, 0), new PVector(-90, 0, -90) }, // left
        { new PVector(-200, 400,   0), new PVector(10, 0, 0), new PVector(-90, 0,   0) }, // back
    };
    
    CamCtrl() {
        _mode   = 0;
        _eye    = new PVector();
        _center = new PVector(0, 0, 0);
        _up     = new PVector(0, 1, 0);
        _rot    = new PVector(0, 0, 0);
    }
    
    void init() {
        _eye.set(0, 0, (height / 2.0) / tan(radians(30)));
        move(1);
    }
    
    void move(int idx) {
        if (idx < _tblCamFix.length) {
            _eye.set(_tblCamFix[idx][0]);
            _center.set(_tblCamFix[idx][1]);
            _rot.set(_tblCamFix[idx][2]);
        }
    }
    
    void apply() {
        beginCamera();
        camera(_eye.x, _eye.y, _eye.z, _center.x, _center.y, _center.z, _up.x, _up.y, _up.z);
        endCamera();
        
        rotateX(radians(_rot.y));
        rotateY(radians(_rot.x));
        rotateZ(radians(_rot.z));
    }
    
    boolean process(int key) {
        boolean ret = true;
        PVector   v = _eye;
        
        //print(String.format("keyCode:%d, key:%d\n", keyCode, key));
        
        if (_mode == 1)
            v = _center;
        else if (_mode == 2)
            v = _rot;
        
        switch (keyCode) {
        case UP:
            if (_mode == 2)
                v.y = (v.y > -360) ? (v.y - 10) : -360;
            else
                v.y -= 10;
            break;
            
        case DOWN:
            if (_mode == 2)
                v.y = (v.y < 360) ? (v.y + 10) : 360;
            else
                v.y += 10;
            break;
            
        case LEFT:
            if (_mode == 2)
                v.x = (v.x > -360) ? (v.x - 10) : -360;
            else
                v.x -= 10;
            break;
            
        case RIGHT:
            if (_mode == 2)
                v.x = (v.x < 360) ? (v.x + 10) : 360;
            else
                v.x += 10;
            break;
            
        case 2:        // home
            if (_mode == 2)
                v.z = (v.z > -360) ? (v.z - 10) : -360;
            else
                v.z = (v.z > 10) ? (v.z - 10) : 0;
            break;
            
        case 3:        // end
            if (_mode == 2)
                v.z = (v.z < 360) ? (v.z + 10) : 360;
            else
                v.z += 10;        
            break;
            
        case 147:      // delete
            _mode = (_mode + 1) % 3;
            print(String.format("MODE:%d\n", _mode));
            break;
            
        case 11:        // pgdn
            break;
            
        case 26:        // insert
            break;
            
        case 16:        // pgup
            break;
            
        default:
            switch (key) {
            case '1':
            case '2':
            case '3':
            case '4':
                move(key - '1');
                break;            
                
            default:
                ret = false;
                break;
            };
            break;
        }
        
        if (ret) {
            print(String.format("CAM : eye(%5.1f %5.1f %5.1f), center(%5.1f %5.1f %5.1f), rot(%5.1f %5.1f %5.1f)\n", 
                _eye.x, _eye.y, _eye.z, _center.x, _center.y, _center.z, _rot.x, _rot.y, _rot.z));
        }
        
        return ret;
    }    
}

CamCtrl _cam = new CamCtrl();



static final float kBodyFrontWidth  = 90;
static final float kBodyHeight      = 124;
static final float kBodyMiddleWidth = 100;
static final float kCoxaLength      = 45;
static final float kFemurLength     = 35;
static final float kTibiaLength     = 70;

void setup() {
    hint(ENABLE_KEY_REPEAT);
    size(1280, 720, P3D);
    lights();
    //noCursor();
    frameRate(30);
    _cam.init();

    _origin  = new PVector(width / 2, height / 2, 0);
    _hexapod = new HexaPod(kBodyFrontWidth, kBodyHeight, kBodyMiddleWidth, kCoxaLength, kFemurLength, kTibiaLength);
    //_hexapod = new HexaPod(kBodyWidth, kCoxaLength, kFemurLength, kTibiaLength);
}

private final color kCOLOR_PLANE = color(100, 100, 100);

void drawPlane(float x, float y, float z, float w, float l) {
    pushMatrix();
    translate(x, y, z);
    fill(kCOLOR_PLANE);
    box(w, l, 1);
    popMatrix();
}

void draw() {
    _cam.apply();
    background(0);
    drawPlane(_origin.x, _origin.y, _origin.z, 5000, 5000);
    _hexapod.draw();
}



void keyPressed() {
    if (_cam.process(key))
        return;
    
    switch (key) {
    case 'j':
        _hexapod.getRot().y--;
        break;

    case 'l':
        _hexapod.getRot().y++;
        break;

    case 'i':
        _hexapod.getRot().x++;
        break;

    case 'k':
        _hexapod.getRot().x--;
        break;        

    case 'u':
        _hexapod.getRot().z--;
        break;

    case 'o':
        _hexapod.getRot().z++;
        break;

    case 'p':
        _hexapod.getRot().set(0, 0, 0);
        _hexapod.getPos().set(0, 0, 0);
        break;

    case '0':
        _isCamLog = !_isCamLog;
        break;

    case 'q':
        _hexapod.getPos().z--;
        break;

    case 'e':
        _hexapod.getPos().z++;
        break;
        
    case 'w':
        _hexapod.getPos().y--;
        break;
        
    case 's':
        _hexapod.getPos().y++;
        break;
        
    case 'a':
        _hexapod.getPos().x--;
        break;
        
    case 'd':
        _hexapod.getPos().x++;
        break;
        
    case ' ':
        _hexapod.walk();
        break;
    }
    _hexapod.update();
    //print(String.format("P%s R%s\n", _hexapod.getPos().toString(), _hexapod.getRot().toString()));
}