// Deterministic simulation: no DOM, network, or clock dependency.
export const WORLD = { w: 900, h: 540, radius: 9, start: {x:65,y:270}, goal: {x:835,y:270,r:24}, maxSteps: 580 };
export const INITIAL = [{x:245,y:255,r:55},{x:425,y:145,r:58},{x:455,y:365,r:64},{x:635,y:260,r:60},{x:705,y:445,r:35}];
export const RANGES = [[85,235],[.65,2.8],[.8,3.4],[.6,2.6],[.38,1.18],[-.65,.65],[1.5,3.5]];
export const LABELS = ['近い距離','ゴール誘導','回避強度','回避優先','センサー角','左右バイアス','巡航速度'];
export const clamp = (v,a=0,b=1) => Math.max(a,Math.min(b,v));
export function rng(seed) { let h=2166136261; for (const c of String(seed)) {h^=c.charCodeAt(0);h=Math.imul(h,16777619);} return () => {h+=0x6D2B79F5;let t=h;t=Math.imul(t^(t>>>15),t|1);t^=t+Math.imul(t^(t>>>7),t|61);return ((t^(t>>>14))>>>0)/4294967296;}; }
export const near = (distance, threshold) => clamp(1-distance/threshold);
export const angle = a => Math.atan2(Math.sin(a),Math.cos(a));
export function sensors(s, obstacles, spread) {
  return [-spread,0,spread].map(offset => {
    const a=s.heading+offset, dx=Math.cos(a),dy=Math.sin(a); let best=300;
    for (const [p,d,max] of [[s.x,dx,WORLD.w],[s.y,dy,WORLD.h]]) {
      if(Math.abs(d)>1e-9) best=Math.min(best,((d>0?max-WORLD.radius:WORLD.radius)-p)/d);
    }
    for(const o of obstacles){
      const ox=s.x-o.x,oy=s.y-o.y, rr=o.r+WORLD.radius;
      if(ox*ox+oy*oy<=rr*rr) {best=0;continue;}
      const b=ox*dx+oy*dy,c=ox*ox+oy*oy-rr*rr,disc=b*b-c;
      if(disc>=0){const hit=-b-Math.sqrt(disc); if(hit>=0)best=Math.min(best,hit);}
    }
    return clamp(best,0,300);
  });
}
export function control(s, obstacles, g) {
  const distance=sensors(s,obstacles,g[4]);
  const memberships=distance.map(d=>near(d,g[0]));
  const [left,front,right]=memberships;
  const error=angle(Math.atan2(WORLD.goal.y-s.y,WORLD.goal.x-s.x)-s.heading);
  const goalLeft=clamp(-error/1.2),goalRight=clamp(error/1.2),aligned=clamp(1-Math.abs(error)/1.2);
  // Small fuzzy modules -> avoidance + attraction -> fuzzy arbitration.
  // Singleton consequents: left=-1, straight=0, right=+1 (screen coordinates).
  const attraction=(goalRight-goalLeft)*g[1];
  const avoidance=clamp((left-right+front*(left-right+g[5]))*g[2],-1,1);
  const danger=Math.max(front,Math.max(left,right)*.65);
  const weight=clamp(danger*g[3]);
  const turn=clamp((1-weight)*attraction+weight*avoidance,-1,1);
  const speed=g[6]*(1-.68*danger);
  return {distance,memberships,error,goalLeft,goalRight,aligned,attraction,avoidance,danger,weight,turn,speed};
}
export function initialState(heading=0){return {...WORLD.start,heading,steps:0,status:'running'};}
export function collision(s,obstacles){
  const r=WORLD.radius;
  return s.x<r||s.x>WORLD.w-r||s.y<r||s.y>WORLD.h-r||obstacles.some(o=>Math.hypot(s.x-o.x,s.y-o.y)<=o.r+r);
}
export function step(s,obstacles,g){
  if(s.status!=='running')return s;
  const c=control(s,obstacles,g),heading=angle(s.heading+c.turn*.115);
  const next={x:s.x+Math.cos(heading)*c.speed,y:s.y+Math.sin(heading)*c.speed,heading,steps:s.steps+1,status:'running'};
  if(collision(next,obstacles))next.status='collision';
  else if(Math.hypot(next.x-WORLD.goal.x,next.y-WORLD.goal.y)<=WORLD.goal.r)next.status='goal';
  else if(next.steps>=WORLD.maxSteps)next.status='timeout';
  return next;
}
export function episode(g,obstacles,heading=0,trace=false){
  let s=initialState(heading);const path=trace?[{...s}]:null;
  while(s.status==='running'){s=step(s,obstacles,g);if(trace)path.push({...s});}
  const progress=1-Math.min(1,Math.hypot(s.x-WORLD.goal.x,s.y-WORLD.goal.y)/770);
  const score=progress*180+(s.status==='goal'?420:0)-(s.status==='collision'?100:0)-s.steps*.06;
  return {score,status:s.status,steps:s.steps,path};
}
export function evaluate(g,obstacles){const runs=[-.18,0,.18].map(h=>episode(g,obstacles,h));return {score:runs.reduce((a,r)=>a+r.score,0)/runs.length,goals:runs.filter(r=>r.status==='goal').length,collisions:runs.filter(r=>r.status==='collision').length,runs};}
export const randomGenome = random => RANGES.map(([lo,hi])=>lo+random()*(hi-lo));
export function tournament(ranked,random){let best=null;for(let i=0;i<3;i++){const p=ranked[Math.floor(random()*ranked.length)];if(!best||p.fitness>best.fitness)best=p;}return best;}
export function crossover(a,b,random){return a.map((v,i)=>{const t=random();return v*t+b[i]*(1-t);});}
export function mutate(g,random,rate=.23){return g.map((v,i)=>{const [lo,hi]=RANGES[i];return random()<rate?clamp(v+(random()*2-1)*(hi-lo)*.22,lo,hi):v;});}
export class Evolution {
  constructor(seed='ALPHA-2016',size=24){this.random=rng(seed);this.size=size;this.generation=0;this.population=Array.from({length:size},()=>randomGenome(this.random));this.history=[];this.assess();}
  assess(){this.ranked=this.population.map(genome=>({genome,fitness:evaluate(genome,INITIAL).score})).sort((a,b)=>b.fitness-a.fitness);this.best=this.ranked[0].genome.slice();const result=evaluate(this.best,INITIAL);this.history.push({generation:this.generation,score:result.score,goals:result.goals});return result;}
  advance(){const next=this.ranked.slice(0,2).map(p=>p.genome.slice());while(next.length<this.size){next.push(mutate(crossover(tournament(this.ranked,this.random).genome,tournament(this.ranked,this.random).genome,this.random),this.random));}this.population=next;this.generation++;return this.assess();}
}
