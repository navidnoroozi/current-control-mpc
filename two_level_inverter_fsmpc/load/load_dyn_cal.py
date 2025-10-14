import math

class Load:
    def __init__(self, R=10.0, L=5e-3, V_backemf=0.0, f_backemf=50.0):
        self.R=R; self.L=L; self.V_backemf=V_backemf; self.f_backemf=f_backemf

    def _vbemf(self, t, phase=0.0):
        return self.V_backemf*math.sin(2*math.pi*self.f_backemf*t + phase)

    def next_current_single(self, i, v_an, t, Ts):
        vbemf = self._vbemf(t)
        di_dt = (v_an - vbemf - self.R*i)/self.L
        return i + di_dt*Ts

    def next_current_three(self, ia,ib,ic, va,vb,vc, t, Ts):
        vbema = self._vbemf(t, 0.0)
        vbemb = self._vbemf(t, -2*math.pi/3)
        vbemc = self._vbemf(t, +2*math.pi/3)
        dia = (va - vbema - self.R*ia)/self.L
        dib = (vb - vbemb - self.R*ib)/self.L
        dic = (vc - vbemc - self.R*ic)/self.L
        return ia + dia*Ts, ib + dib*Ts, ic + dic*Ts

    def calculateLoadDynamics(self, v_inv_tarj, i_init, t_0, sampling_rate, phase_num=1):
        t=t_0
        if phase_num==1:
            i=i_init
            traj=[]
            for v_an in v_inv_tarj:
                i = self.next_current_single(i, v_an, t, sampling_rate)
                traj.append(i)
                t += sampling_rate
            return traj
        else:
            ia,ib,ic = i_init
            ia_tr=[]; ib_tr=[]; ic_tr=[]
            for (va,vb,vc) in v_inv_tarj:
                ia,ib,ic = self.next_current_three(ia,ib,ic, va,vb,vc, t, sampling_rate)
                ia_tr.append(ia); ib_tr.append(ib); ic_tr.append(ic)
                t += sampling_rate
            return (ia_tr, ib_tr, ic_tr)
