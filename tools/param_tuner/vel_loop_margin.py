import numpy as np
G=4.4/(0.0018*63/13)   # torque -> V
T=1e-3
def loop(p,i,d,ca=0.1004,cv=1.1,Td=1.5e-3,f=None):
    f=np.logspace(0,np.log10(450),4000) if f is None else f
    w=2*np.pi*f; z=np.exp(1j*w*T)
    C=(p+i/(1-1/z)+d*(1-1/z))*G           # V per (mm/s)
    P=1000.0/(ca*1j*w+cv)                  # (mm/s) per V
    L=C*P*np.exp(-1j*w*Td)
    return f,L
def margins(p,i,d,**k):
    f,L=loop(p,i,d,**k); mag=np.abs(L); ph=np.unwrap(np.angle(L))
    S=1/(1+L)
    # gain crossover (last crossing downward)
    idx=np.where((mag[:-1]>1)&(mag[1:]<=1))[0]
    fc=f[idx[-1]] if len(idx) else np.nan; pm=180+np.degrees(ph[idx[-1]]) if len(idx) else np.nan
    # phase crossover
    j=np.where((ph[:-1]>-np.pi)&(ph[1:]<=-np.pi))[0]
    fp=f[j[0]] if len(j) else np.nan; gm=1/mag[j[0]] if len(j) else np.nan
    k=np.argmax(np.abs(S))
    return fc,pm,fp,gm,np.abs(S[k]),f[k],f,S,L
if __name__=='__main__':
    print('plant: ca=0.1004 V/(m/s^2), cv=1.1 V/(m/s) -> tau=%.0f ms, DC gain %.0f (mm/s)/V'%(0.1004/1.1*1000,1000/1.1))
    print('%-34s %7s %6s %7s %6s %6s %6s | |S| at 9/18/36/82/110Hz'%('gains (p/i/d) , plant','fc[Hz]','PM','f180','GM','Smax','@Hz'))
    cases=[('now 7.25/2.75/8.75 new plant',7.25e-6,2.75e-6,8.75e-6,0.1004),
           ('same gains, pre-swap plant',7.25e-6,2.75e-6,8.75e-6,0.1098),
           ('09-13 6.0/2.7/8.75 pre-swap',6.0e-6,2.7e-6,8.75e-6,0.1098),
           ('old 8.25/3.75/8.75 pre-swap',8.25e-6,3.75e-6,8.75e-6,0.1098)]
    for lab,p,i,d,ca in cases:
        fc,pm,fp,gm,sm,fs,f,S,L=margins(p,i,d,ca=ca)
        sv=[np.abs(S[np.argmin(abs(f-x))]) for x in (9,18,36,82,110)]
        print('%-34s %7.1f %6.1f %7.1f %6.2f %6.2f %6.1f | %s'%(lab,fc,pm,fp,gm,sm,fs,' '.join('%.2f'%x for x in sv)))
    print('\n-- delay sensitivity (now gains, new plant)')
    for Td in (1.0e-3,1.5e-3,2.0e-3,2.5e-3):
        fc,pm,fp,gm,sm,fs,*_=margins(7.25e-6,2.75e-6,8.75e-6,Td=Td)
        print('Td=%.1fms fc=%.1f PM=%.1f f180=%.1f GM=%.2f Smax=%.2f@%.0fHz'%(Td*1e3,fc,pm,fp,gm,sm,fs))
