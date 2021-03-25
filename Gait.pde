public abstract class Gait {
    public final static float kUNIT_MM   = 1f;
    public final static float kPRECISION = 0.1f;

    protected PVector   _vecStep;
    protected float     _stepsPerSec;
    protected GaitParam _paramLegs[] = new GaitParam[6];
    
    public Gait(float stanceRate) {
        _vecStep     = new PVector(20, 20, 10);
        _stepsPerSec = 1f;
        
        for (int i = 0; i < _paramLegs.length; i++) {
            _paramLegs[i] = new GaitParam(stanceRate);
        }
    }

    public void    setStep(PVector vecStep)                           { _vecStep = vecStep;         }
    public PVector getStep()                                          { return _vecStep;            }
    public void    setStepsPerSec(float steps)                        { _stepsPerSec = steps;       }
    public float   getStepsPerSec()                                   { return _stepsPerSec;        }
    
    protected void setSwingOffsets(float[] offsets) {
        for (int i = 0; i < _paramLegs.length; i++) {
            _paramLegs[i].setSwingOffset(offsets[i]);
        }
    }
    
    protected PVector calcDirRatio(PVector dir) {
        float maxV = max(abs(dir.x), abs(dir.y));
        return new PVector(dir.x / maxV, dir.y / maxV);
    }
    
    public PVector doStep(int leg, float freq, PVector dir, PVector rot) {
        if (abs(dir.x) == kPRECISION && abs(dir.y) == kPRECISION) {
            return new PVector(0.0f, 0.0f, dir.z);
        }

        _paramLegs[leg].tick(dir, _vecStep, freq, _stepsPerSec);
        PVector rDir = calcDirRatio(dir);
        PVector c = new PVector(rDir.x * _paramLegs[leg].getAmplitude(), rDir.y * _paramLegs[leg].getAmplitude(), dir.z);
        
        if (_paramLegs[leg].isSwingState()) {
            float w0 = _vecStep.x * Gait.kUNIT_MM / 2 * dir.x;
            float l0 = _vecStep.y * Gait.kUNIT_MM * 4 * dir.y;
            float h0 = _vecStep.z * Gait.kUNIT_MM;
            float h1 = sqrt(abs((1 - sq(c.x / w0) - sq(c.y / l0)) * sq(h0)));
            c.z = c.z - h1;
        } else {
            c.set(-c.x, -c.y, c.z);
        }
        
        print(String.format("tick:%3d, amplitude:%6.1f, swing:%d, (%6.1f, %6.1f)\n", _paramLegs[leg].getTick(), 
            _paramLegs[leg].getAmplitude(), int(_paramLegs[leg].isSwingState()), c.x, c.z));
    
        return new PVector(c.x / Gait.kUNIT_MM, c.y / Gait.kUNIT_MM, c.z / Gait.kUNIT_MM);
    }
    
    abstract public String getName();    
};

/*
***************************************************************************************************
* GaitParam
***************************************************************************************************
*/
public class GaitParam {
    private   float    _fAmplitude;
    private   short    _tick;

    private   boolean  _isSwing;
    private   float    _fCurMult;
    private   float    _fStanceMult;
    
    public GaitParam(float fStanceMult) {
        _fStanceMult = fStanceMult;
        setSwingState(false);
    }
    
    public void setSwingOffset(float fSwingOffset) {
        if (fSwingOffset == 0) {
            setSwingState(true);
        } else {
            _fCurMult = fSwingOffset;
        }
    }
 
    public void setSwingState(boolean en) {
        _isSwing = en;
        if (en) {
            _fCurMult = 1.0f;
        } else {
            _fCurMult = _fStanceMult;
        }
    }
    
    protected void tick(PVector dir, PVector step, float freq, float stepsPerSec) {
        float w0 = step.x * Gait.kUNIT_MM / (2 / max(abs(dir.x), abs(dir.y)));
        float a0 = (w0 * 2) * (float(_tick) / round((freq * _fCurMult) / stepsPerSec)) - w0;

        _tick++;
        _fAmplitude = a0;
        if (_tick > (round(freq * _fCurMult) / stepsPerSec)) {
            setSwingState(!isSwingState());
            _fAmplitude = -w0;
            _tick = 1;
        }
    }    
    
    public boolean isSwingState()   { return _isSwing;  }
    public float   getAmplitude()   { return _fAmplitude;  }
    public int     getTick()        { return _tick; }
}


/*
***************************************************************************************************
* GaitTripod
***************************************************************************************************
*/
public class GaitTripod extends Gait {
    public GaitTripod() {
        super(1.0f);
        setSwingOffsets(new float[] { 0, 1, 0, 1, 0, 1});
    }

    public String getName() { return "Trot"; }
};


/*
***************************************************************************************************
* GaitWave
***************************************************************************************************
*/
public class GaitWave extends Gait {
    public GaitWave() {
        super(5.0f);
        setSwingOffsets(new float[] { 2, 1, 0, 3, 4, 5});
    }

    public String getName() { return "Wave"; }

};


/*
***************************************************************************************************
* GaitRipple
***************************************************************************************************
*/
public class GaitRipple extends Gait {
    public GaitRipple() {
        super(2.0f);
        //                            0  1  2  3  4  5
        setSwingOffsets(new float[] { 2, 0, 1, 2, 1, 0});
    }

    public String getName() { return "Wave"; }

};
