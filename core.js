// Deterministic simulation: no DOM, network, or clock dependency.
export const WORLD = { w: 900, h: 540, radius: 9, start: {x:65,y:270}, goal: {x:835,y:270,r:24}, maxSteps: 270 };
export const INITIAL = [{x:245,y:255,r:55},{x:425,y:145,r:58},{x:455,y:365,r:64},{x:635,y:260,r:60},{x:705,y:445,r:35}];
export const RANGES = [[85,235],[.65,2.8],[.8,3.4],[.6,2.6],[.38,1.18],[-.65,.65],[1.5,3.5]];
export const HEADINGS = [-.18,0,.18];
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
export const NN_RANGES = [...Array.from({length:44},()=>[-2,2]), RANGES[4]];
export const sensorSpread = (g,method='gft') => method==='nn'?g[44]:g[4];
export function nnControl(s,obstacles,g){
  const distance=sensors(s,obstacles,sensorSpread(g,'nn'));
  const error=angle(Math.atan2(WORLD.goal.y-s.y,WORLD.goal.x-s.x)-s.heading);
  const inputs=[...distance.map(d=>d/300),error/Math.PI];
  let k=0;
  const hidden=Array.from({length:6},()=>{let sum=0;for(const x of inputs)sum+=x*g[k++];return Math.tanh(sum+g[k++]);});
  const sums=Array.from({length:2},()=>{let sum=0;for(const x of hidden)sum+=x*g[k++];return sum+g[k++];});
  const outputs=[Math.tanh(sums[0]),1/(1+Math.exp(-sums[1]))];
  return {distance,error,inputs,hidden,outputs,turn:outputs[0],speed:.48+3.02*outputs[1]};
}
export function control(s, obstacles, g, method='gft') {
  if(method==='nn')return nnControl(s,obstacles,g);
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
export function step(s,obstacles,g,method='gft'){
  if(s.status!=='running')return s;
  const c=control(s,obstacles,g,method),heading=angle(s.heading+c.turn*.115);
  const next={x:s.x+Math.cos(heading)*c.speed,y:s.y+Math.sin(heading)*c.speed,heading,steps:s.steps+1,status:'running'};
  if(collision(next,obstacles))next.status='collision';
  else if(Math.hypot(next.x-WORLD.goal.x,next.y-WORLD.goal.y)<=WORLD.goal.r)next.status='goal';
  else if(next.steps>=WORLD.maxSteps)next.status='timeout';
  return next;
}
export function episode(g,obstacles,heading=0,trace=false,method='gft'){
  let s=initialState(heading);const path=trace?[{...s}]:null;
  while(s.status==='running'){s=step(s,obstacles,g,method);if(trace)path.push({...s});}
  const progress=1-Math.min(1,Math.hypot(s.x-WORLD.goal.x,s.y-WORLD.goal.y)/770);
  const score=progress*180+(s.status==='goal'?420:0)-(s.status==='collision'?100:0)-s.steps*.06;
  return {score,status:s.status,steps:s.steps,path};
}
export function evaluate(g,obstacles,method='gft'){const runs=HEADINGS.map(h=>episode(g,obstacles,h,false,method));return {score:runs.reduce((a,r)=>a+r.score,0)/runs.length,goals:runs.filter(r=>r.status==='goal').length,collisions:runs.filter(r=>r.status==='collision').length,runs};}
export const randomGenome = (random,method='gft') => (method==='nn'?NN_RANGES:RANGES).map(([lo,hi])=>lo+random()*(hi-lo));
export function tournament(ranked,random){let best=null;for(let i=0;i<3;i++){const p=ranked[Math.floor(random()*ranked.length)];if(!best||p.fitness>best.fitness)best=p;}return best;}
export function crossover(a,b,random){return a.map((v,i)=>{const t=random();return v*t+b[i]*(1-t);});}
export function mutate(g,random,rate=.23,method='gft'){return g.map((v,i)=>{const [lo,hi]=(method==='nn'?NN_RANGES:RANGES)[i];return random()<rate?clamp(v+(random()*2-1)*(hi-lo)*(method==='nn'?.10:.22),lo,hi):v;});}
export class Evolution {
  constructor(seed='ALPHA-2016',size=24,method='gft'){this.seed=seed;this.method=method;this.random=rng(seed);this.size=size;this.generation=0;this.population=Array.from({length:size},()=>randomGenome(this.random,this.method));this.history=[];this.assess();this.initialBest=this.best.slice();}
  assess(){this.ranked=this.population.map(genome=>({genome,fitness:evaluate(genome,INITIAL,this.method).score})).sort((a,b)=>b.fitness-a.fitness);this.best=this.ranked[0].genome.slice();const result=evaluate(this.best,INITIAL,this.method);this.history.push({generation:this.generation,score:result.score,goals:result.goals});return result;}
  advance(){const next=this.ranked.slice(0,2).map(p=>p.genome.slice());while(next.length<this.size){next.push(mutate(crossover(tournament(this.ranked,this.random).genome,tournament(this.ranked,this.random).genome,this.random),this.random,this.method==='nn'?.1:.23,this.method));}this.population=next;this.generation++;return this.assess();}
}

// Each method owns its genome, seed, RNG closure, history and initial best.
export class Lab {
  constructor(seed='ALPHA-2016'){this.method='gft';this.states={gft:new Evolution(seed)};}
  get current(){return this.states[this.method];}
  select(method){if(!['gft','nn'].includes(method))throw Error('unknown method');this.method=method;return this.states[method]??(this.states[method]=new Evolution('ALPHA-2016',24,method));}
  reset(seed){return this.states[this.method]=new Evolution(seed,24,this.method);}
}
