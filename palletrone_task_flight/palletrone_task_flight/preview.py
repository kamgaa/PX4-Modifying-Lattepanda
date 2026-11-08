"""Offline preview of the same Plan. No ROS imports or hardware access."""
import csv
import json
from pathlib import Path
from math import degrees
from .trajectory import Plan


def write_preview(plan, destination):
    destination=Path(destination)
    destination.mkdir(parents=True,exist_ok=True)
    rows=[]
    for t,segment,target in plan.samples(100):
        m,e=plan.geometry.points(target.position,target.yaw,target.theta)
        rows.append({'t':t,'phase':segment.name,'p':target.position,'m':m,'e':e,
                     'yaw':degrees(target.yaw),'theta':degrees(target.theta),'d':target.distance})
    summary=plan.summary()
    summary['timing_note']='Nominal reference motion only; live feedback gating and settling extend execution.'
    summary['bounds_note']='Extents sampled at 400 subdivisions/segment; command speed bounds are analytic.'
    (destination/'plan.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    with (destination/'trajectory.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['t','phase','px','py','pz','mx','my','mz','ex','ey','ez','yaw_deg','j1_deg','d_m'])
        for r in rows:w.writerow([r['t'],r['phase'],*r['p'],*r['m'],*r['e'],r['yaw'],r['theta'],r['d']])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(13,9))
    c=plan.contact;start=plan.start
    for ax,indices,title in [(axes[0,0],(0,1),'Top view / NED XY'),(axes[0,1],(0,2),'Side view / NED X and height')]:
        i,j=indices
        for key,label,color in [('p','PX4 reference P','#2463b4'),('m','J1 mount M','#996b00'),('e','Rod endpoint E','#159279')]:
            xs=[r[key][i]-start[i] for r in rows]
            ys=[r[key][j]-start[j] for r in rows]
            if j==2:ys=[-v for v in ys]
            ax.plot(xs,ys,label=label,color=color,linewidth=1.1)
        ax.scatter([c[i]-start[i]],[start[j]-c[j] if j==2 else c[j]-start[j]],marker='*',s=180,c='crimson',label='Fixed C')
        for seg in plan.segments:
            if seg.name.startswith('APPROACH'):
                target=seg.start;m,e=plan.geometry.points(target.position,target.yaw,target.theta)
                ax.plot([m[i]-start[i],e[i]-start[i]],[start[j]-m[j],start[j]-e[j]] if j==2 else [m[j]-start[j],e[j]-start[j]],color='gray',alpha=.5)
        ax.set_title(title);ax.set_aspect('equal',adjustable='datalim');ax.grid(True,alpha=.3)
        ax.set_xlabel('X relative to start (m)');ax.set_ylabel('Height above start (m)' if j==2 else 'Y relative to start (m)')
        ax.legend(fontsize=8)
    ts=[r['t'] for r in rows]
    for k,label in enumerate(('X','Y','Z (down)')):
        axes[1,0].plot(ts,[r['p'][k]-start[k] for r in rows],label=label)
    axes[1,0].set_title('PX4 position target relative to start');axes[1,0].set_ylabel('m')
    axes[1,1].plot(ts,[r['yaw']-degrees(plan.yaw0) for r in rows],label='yaw relative to start')
    axes[1,1].plot(ts,[r['theta'] for r in rows],label='physical J1 angle')
    axes[1,1].set_title('Yaw and J1 target');axes[1,1].set_ylabel('degrees')
    for ax in axes[1]:
        ax.set_xlabel('Nominal reference time (s)');ax.grid(True,alpha=.3);ax.legend()
        for seg in plan.segments:
            if seg.name.startswith('APPROACH'):
                a=next(r['t'] for r in rows if r['phase']==seg.name)
                ax.axvspan(a,a+seg.duration,color='orange',alpha=.10)
    fig.suptitle(f'Fixed-contact task preview | initial rise {plan.p.hover_height_m:.2f} m | max rise {summary["max_rise_from_start_m"]:.3f} m\nLevel body assumption; target arrival is not contact detection',fontsize=13)
    fig.tight_layout();fig.savefig(destination/'path_preview.png',dpi=140);plt.close(fig)
    data=json.dumps({'rows':rows,'contact':c,'summary':summary})
    html='''<!doctype html><meta charset="utf-8"><title>Palletrone task preview</title>
<style>body{font:16px system-ui;max-width:1100px;margin:25px auto;background:#f8fafc;color:#18304b}canvas{background:white;border:1px solid #ccc;width:49%;height:auto}input{width:85%}pre{white-space:pre-wrap}img{width:100%}</style>
<h1>Fixed-contact task preview</h1><p>Offline only. Level roll/pitch assumed. C is calculated from initial placement, not sensor detection. P blue, M amber, E green, C red. Rod M–E is black.</p>
<canvas id="top" width="520" height="360"></canvas><canvas id="side" width="520" height="360"></canvas>
<p><button id="play">Play / pause</button> <input id="slider" type="range" min="0" value="0"></p><pre id="info"></pre>
<img src="path_preview.png" alt="Plan projections and time histories"><details><summary>Plan details</summary><pre id="summary"></pre></details>
<script>const data=__DATA__;const rows=data.rows;let running=false;const slider=document.getElementById('slider');slider.max=rows.length-1;
document.getElementById('summary').textContent=JSON.stringify(data.summary,null,2);
function draw(id,a,b,neg){const cv=document.getElementById(id),ctx=cv.getContext('2d');ctx.clearRect(0,0,520,360);const all=rows.flatMap(r=>[r.p,r.m,r.e]).concat([data.contact]);
const xs=all.map(v=>v[a]),ys=all.map(v=>neg?-v[b]:v[b]);const loX=Math.min(...xs),hiX=Math.max(...xs),loY=Math.min(...ys),hiY=Math.max(...ys);const s=Math.min(440/Math.max(.1,hiX-loX),270/Math.max(.1,hiY-loY));
const pt=v=>[40+(v[a]-loX)*s,320-((neg?-v[b]:v[b])-loY)*s];function path(points,color){ctx.strokeStyle=color;ctx.beginPath();points.forEach((v,i)=>{const p=pt(v);i?ctx.lineTo(...p):ctx.moveTo(...p)});ctx.stroke()}
path(rows.map(r=>r.p),'#b5c8e1');const r=rows[+slider.value];path([r.m,r.e],'black');path([r.e,data.contact],'#999');
[[r.p,'#2463b4','P'],[r.m,'#996b00','M'],[r.e,'#159279','E'],[data.contact,'crimson','C']].forEach(([v,col,label])=>{const p=pt(v);ctx.fillStyle=col;ctx.beginPath();ctx.arc(...p,5,0,7);ctx.fill();ctx.fillText(label,p[0]+8,p[1]-8)});ctx.fillStyle='#18304b';ctx.fillText(id==='top'?'Top: local NED X/Y':'Side: local X / height',20,20)}
function update(){const r=rows[+slider.value];draw('top',0,1,false);draw('side',0,2,true);document.getElementById('info').textContent=`${r.phase} | nominal t=${r.t.toFixed(2)} s\nP=${r.p.map(x=>x.toFixed(4))} m | yaw=${r.yaw.toFixed(2)} deg | J1=${r.theta.toFixed(2)} deg | d=${r.d.toFixed(3)} m`}
slider.oninput=update;document.getElementById('play').onclick=()=>running=!running;setInterval(()=>{if(running){slider.value=(+slider.value+1)%rows.length;update()}},30);update();</script>'''
    (destination/'preview.html').write_text(html.replace('__DATA__',data),encoding='utf-8')
    return summary
