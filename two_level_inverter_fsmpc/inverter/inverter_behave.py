
class Inverter:
    def __init__(self, V_dc=400.0):
        self.V_dc = V_dc

    def generateOutputVoltage(self, s_seq, phase_num=1):
        """Return inverter pole voltages v_{xN} for each step.
        single-phase: s in {0,1}; v = Vdc*(2s-1)/2
        three-phase: s tuple (sa,sb,sc) per step -> return (va_list, vb_list, vc_list)
        """
        V = self.V_dc/2.0
        if phase_num==1:
            return [ V*(2*int(s)-1) for s in s_seq ]
        else:
            va=[]; vb=[]; vc=[]
            for sa,sb,sc in s_seq:
                va.append(V*(2*int(sa)-1))
                vb.append(V*(2*int(sb)-1))
                vc.append(V*(2*int(sc)-1))
            return (va,vb,vc)
