public abstract class Gait {
    public final static float kUNIT_MM   = 1f;
    public final static float kPRECISION = 0.1f;
    
    public Gait() {
        _vecStep   = new PVector(40, 40, 10);
        _pOption   = null;
        _stepsPerSec = 1f;
    }

    public void  setStep(PVector vecStep)                           { _vecStep = vecStep;         }
    public PVector getStep()                                        { return _vecStep;            }
    public void  setStepsPerSec(float steps)                        { _stepsPerSec  = steps;      }
    public float getStepsPerSec()                                   { return _stepsPerSec;        }
    public void  setOption(Object option)                           { _pOption = option;          }
    public void  setTiptoeOffset(PVector vOffset)                   { _vTiptoeOffset = vOffset;   }

    abstract public String getName();
    abstract public PVector doStep(int leg, float freq, PVector dir, PVector rot, float sign);

    protected PVector   _vTiptoeOffset;
    protected PVector   _vecStep;
    protected float    _stepsPerSec;

    protected float    _c[]      = new float[6];
    protected short    _c_iter[] = new short[6];
    protected short    _c_inv[]  = new short[6];
    protected Object   _pOption;
    
    protected void clock(int leg, float freq, PVector dir) {
        float w0 = _vecStep.x * kUNIT_MM / (2 / max(abs(dir.x), abs(dir.y)));
        float a0 = (w0 * 2) * (float(_c_iter[leg]) / round(freq / _stepsPerSec)) - w0;
    
        _c[leg] = a0;
        _c_iter[leg]++;
    
        if (_c_iter[leg] > round(freq / _stepsPerSec)) {
            _c[leg]      = -w0;
            _c_iter[leg] = 1;
            _c_inv[leg]++;
            if (_c_inv[leg] > 31)
                _c_inv[leg] = 0;
        }
        //println(String.format("c_iter:%3d, c_inv:%3d\n", _c_iter[leg], _c_inv[leg]));        
    }
    
    protected PVector calcDirRatio(PVector dir) {
        float maxV = max(abs(dir.x), abs(dir.y));
        return new PVector(dir.x / maxV, dir.y / maxV);
    }
};

public class GaitTrot extends Gait {
    public GaitTrot() {
        super();
    }

    public String getName() { return "Trot"; }
    
    public PVector doStep(int leg, float freq, PVector dir, PVector rot, float sign) {
        if (abs(dir.x) <= Gait.kPRECISION && abs(dir.y) <= Gait.kPRECISION) {
            return new PVector(0.0f, 0.0f, 0.0f);
        }

        clock(leg, freq, dir);
    
        PVector rDir = calcDirRatio(dir);
        PVector c = new PVector(rDir.x * _c[leg], sign * rDir.y * _c[leg]);
        float w0 = _vecStep.x * Gait.kUNIT_MM / 2 * dir.x;
        float l0 = _vecStep.y * Gait.kUNIT_MM * 4 * dir.y;
        float h0 = _vecStep.z * Gait.kUNIT_MM;
        boolean inv = boolean(leg % 2) ^ boolean(_c_inv[leg] % 2);
    
        println(String.format("gaittr leg:%d, c_iter:%3d, c_inv:%3d, inv:%d", leg, _c_iter[leg], _c_inv[leg], int(inv)));
    
        if (inv == false) {
            c.set(-c.x, -c.y);
        }
        float h1 = sqrt(abs((1 - sq(c.x / w0) - sq(c.y / l0)) * sq(h0)));
    
        return new PVector(c.x / Gait.kUNIT_MM, c.y / Gait.kUNIT_MM, dir.z + h1 / Gait.kUNIT_MM * int(inv));
    }
};
