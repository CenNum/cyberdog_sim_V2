/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class simulator_lcmt implements lcm.lcm.LCMEncodable
{
    public double vb[];
    public double rpy[];
    public long timesteps;
    public double time;
    public double quat[];
    public double R[];
    public double omegab[];
    public double omega[];
    public double p[];
    public double v[];
    public double vbd[];
    public double q[];
    public double qd[];
    public double qdd[];
    public double tau[];
    public double tauAct[];
    public double f_foot[];
    public double p_foot[];
 
    public simulator_lcmt()
    {
        vb = new double[3];
        rpy = new double[3];
        quat = new double[4];
        R = new double[9];
        omegab = new double[3];
        omega = new double[3];
        p = new double[3];
        v = new double[3];
        vbd = new double[3];
        q = new double[12];
        qd = new double[12];
        qdd = new double[12];
        tau = new double[12];
        tauAct = new double[12];
        f_foot = new double[12];
        p_foot = new double[12];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x7d7ef698ad9bea0fL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.simulator_lcmt.class))
            return 0L;
 
        classes.add(lcmtypes.simulator_lcmt.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.vb[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.rpy[a]); 
        }
 
        outs.writeLong(this.timesteps); 
 
        outs.writeDouble(this.time); 
 
        for (int a = 0; a < 4; a++) {
            outs.writeDouble(this.quat[a]); 
        }
 
        for (int a = 0; a < 9; a++) {
            outs.writeDouble(this.R[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.omegab[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.omega[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.p[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.v[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.vbd[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeDouble(this.q[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeDouble(this.qd[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeDouble(this.qdd[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeDouble(this.tau[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeDouble(this.tauAct[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeDouble(this.f_foot[a]); 
        }
 
        for (int a = 0; a < 12; a++) {
            outs.writeDouble(this.p_foot[a]); 
        }
 
    }
 
    public simulator_lcmt(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public simulator_lcmt(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.simulator_lcmt _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.simulator_lcmt o = new lcmtypes.simulator_lcmt();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.vb = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.vb[a] = ins.readDouble();
        }
 
        this.rpy = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.rpy[a] = ins.readDouble();
        }
 
        this.timesteps = ins.readLong();
 
        this.time = ins.readDouble();
 
        this.quat = new double[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.quat[a] = ins.readDouble();
        }
 
        this.R = new double[(int) 9];
        for (int a = 0; a < 9; a++) {
            this.R[a] = ins.readDouble();
        }
 
        this.omegab = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.omegab[a] = ins.readDouble();
        }
 
        this.omega = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.omega[a] = ins.readDouble();
        }
 
        this.p = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.p[a] = ins.readDouble();
        }
 
        this.v = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.v[a] = ins.readDouble();
        }
 
        this.vbd = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.vbd[a] = ins.readDouble();
        }
 
        this.q = new double[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.q[a] = ins.readDouble();
        }
 
        this.qd = new double[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.qd[a] = ins.readDouble();
        }
 
        this.qdd = new double[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.qdd[a] = ins.readDouble();
        }
 
        this.tau = new double[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.tau[a] = ins.readDouble();
        }
 
        this.tauAct = new double[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.tauAct[a] = ins.readDouble();
        }
 
        this.f_foot = new double[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.f_foot[a] = ins.readDouble();
        }
 
        this.p_foot = new double[(int) 12];
        for (int a = 0; a < 12; a++) {
            this.p_foot[a] = ins.readDouble();
        }
 
    }
 
    public lcmtypes.simulator_lcmt copy()
    {
        lcmtypes.simulator_lcmt outobj = new lcmtypes.simulator_lcmt();
        outobj.vb = new double[(int) 3];
        System.arraycopy(this.vb, 0, outobj.vb, 0, 3); 
        outobj.rpy = new double[(int) 3];
        System.arraycopy(this.rpy, 0, outobj.rpy, 0, 3); 
        outobj.timesteps = this.timesteps;
 
        outobj.time = this.time;
 
        outobj.quat = new double[(int) 4];
        System.arraycopy(this.quat, 0, outobj.quat, 0, 4); 
        outobj.R = new double[(int) 9];
        System.arraycopy(this.R, 0, outobj.R, 0, 9); 
        outobj.omegab = new double[(int) 3];
        System.arraycopy(this.omegab, 0, outobj.omegab, 0, 3); 
        outobj.omega = new double[(int) 3];
        System.arraycopy(this.omega, 0, outobj.omega, 0, 3); 
        outobj.p = new double[(int) 3];
        System.arraycopy(this.p, 0, outobj.p, 0, 3); 
        outobj.v = new double[(int) 3];
        System.arraycopy(this.v, 0, outobj.v, 0, 3); 
        outobj.vbd = new double[(int) 3];
        System.arraycopy(this.vbd, 0, outobj.vbd, 0, 3); 
        outobj.q = new double[(int) 12];
        System.arraycopy(this.q, 0, outobj.q, 0, 12); 
        outobj.qd = new double[(int) 12];
        System.arraycopy(this.qd, 0, outobj.qd, 0, 12); 
        outobj.qdd = new double[(int) 12];
        System.arraycopy(this.qdd, 0, outobj.qdd, 0, 12); 
        outobj.tau = new double[(int) 12];
        System.arraycopy(this.tau, 0, outobj.tau, 0, 12); 
        outobj.tauAct = new double[(int) 12];
        System.arraycopy(this.tauAct, 0, outobj.tauAct, 0, 12); 
        outobj.f_foot = new double[(int) 12];
        System.arraycopy(this.f_foot, 0, outobj.f_foot, 0, 12); 
        outobj.p_foot = new double[(int) 12];
        System.arraycopy(this.p_foot, 0, outobj.p_foot, 0, 12); 
        return outobj;
    }
 
}

