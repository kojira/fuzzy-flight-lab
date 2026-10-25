import test from 'node:test';
import assert from 'node:assert/strict';
import {WORLD,INITIAL,RANGES,rng,near,control,initialState,step,collision,episode,evaluate,randomGenome,tournament,crossover,mutate,Evolution} from '../core.js';
test('近い所属度の境界・単調性・確率分布ではない',()=>{assert.equal(near(0,100),1);assert.equal(near(50,100),.5);assert.equal(near(100,100),0);assert.equal(near(200,100),0);assert.equal(near(-1,100),1);});
test('seed再現とseedの分離',()=>{const a=rng('abc'),b=rng('abc'),c=rng('def');const x=Array.from({length:20},a);assert.deepEqual(x,Array.from({length:20},b));assert.notDeepEqual(x,Array.from({length:20},c));});
test('各制御値が有限、旋回と混合比の範囲',()=>{const random=rng(7);for(let i=0;i<200;i++){const g=randomGenome(random),s={x:20+random()*860,y:20+random()*500,heading:random()*Math.PI*2};const c=control(s,INITIAL,g);for(const v of Object.values(c).flat())assert.ok(Number.isFinite(v));assert.ok(Math.abs(c.turn)<=1);assert.ok(c.weight>=0&&c.weight<=1);assert.ok(c.speed>0);}});
test('選択はトーナメントの最良を選ぶ',()=>{const ranked=[{fitness:1},{fitness:8},{fitness:3}],values=[0,.4,.9];assert.equal(tournament(ranked,()=>values.shift()).fitness,8);});
test('交叉は両親を実際に混ぜる',()=>{assert.deepEqual(crossover([0,2],[4,6],()=>.25),[3,5]);});
test('突然変異は発生し、範囲内。rate=0なら不変',()=>{const g=RANGES.map(([a,b])=>(a+b)/2);assert.deepEqual(mutate(g,rng(1),0),g);const m=mutate(g,rng(1),1);assert.notDeepEqual(m,g);m.forEach((v,i)=>assert.ok(v>=RANGES[i][0]&&v<=RANGES[i][1]));});
test('衝突、外壁、到達、時間切れを判定',()=>{const g=[150,1,1,1,.8,0,2];assert.ok(collision({x:245,y:255},INITIAL));assert.ok(collision({x:0,y:270},[]));assert.ok(!collision(initialState(),INITIAL));const goal=step({...initialState(),x:WORLD.goal.x-2,y:WORLD.goal.y},[],g);assert.equal(goal.status,'goal');const wall=step({...initialState(),x:WORLD.w-8,y:20},[],g);assert.equal(wall.status,'collision');const timeout=step({...initialState(),steps:WORLD.maxSteps-1},[],g);assert.equal(timeout.status,'timeout');assert.equal(step(goal,[],g),goal);});
test('エピソードと集計は実測結果に一致',()=>{const g=[150,1,1,1,.8,0,2],r=episode(g,INITIAL,0,true),e=evaluate(g,INITIAL);assert.equal(r.path.length,r.steps+1);assert.ok(r.steps<=WORLD.maxSteps);assert.ok(Number.isFinite(r.score));assert.equal(e.goals,e.runs.filter(x=>x.status==='goal').length);assert.equal(e.score,e.runs.reduce((a,r)=>a+r.score,0)/3);});
test('実GAは同じseedで同じ進化、エリート保持、世代を更新',()=>{const a=new Evolution('test',12),b=new Evolution('test',12),before=a.history[0].score,pop=a.population.map(g=>g.slice());for(let i=0;i<4;i++){a.advance();b.advance();}assert.deepEqual(a.history,b.history);assert.deepEqual(a.best,b.best);assert.equal(a.generation,4);assert.ok(a.history.at(-1).score>=before);assert.notDeepEqual(a.population,pop);assert.equal(a.population.length,12);});

test('時間制約は全世代で270、初期集団を壊さずデフォルトseedで到達改善',()=>{assert.equal(WORLD.maxSteps,270);const e=new Evolution();const initial=e.initialBest.slice();const random=rng('ALPHA-2016');assert.deepEqual(e.population,Array.from({length:24},()=>randomGenome(random)));assert.equal(evaluate(initial,INITIAL).goals,0);for(let i=0;i<20;i++)e.advance();assert.deepEqual(e.initialBest,initial);assert.equal(evaluate(e.best,INITIAL).goals,3);assert.ok(e.history.at(-1).score>e.history[0].score);});

// Neural controller: real weighted sums, no heuristic controller fallback.
const {NN_RANGES,nnControl,sensors,sensorSpread,HEADINGS,Lab}=await import('../core.js');
test('NN zero/explicit weights compute actual tanh and sigmoid',()=>{
  const g=Array(45).fill(0);g[44]=.8;
  let c=nnControl(initialState(),INITIAL,g);
  assert.deepEqual(c.hidden,Array(6).fill(0));assert.deepEqual(c.outputs,[0,.5]);assert.equal(c.speed,1.99);
  g[0]=1;g[4]=.2;g[30]=2;g[36]=-.1;g[37]=-.5;g[43]=.4;
  c=control(initialState(),INITIAL,g,'nn');const h=Math.tanh(c.inputs[0]+.2);
  assert.equal(c.hidden[0],h);assert.equal(c.turn,Math.tanh(2*h-.1));assert.equal(c.outputs[1],1/(1+Math.exp(-(-.5*h+.4))));assert.equal(c.speed,.48+3.02*c.outputs[1]);
});
test('NN finite bounded real outputs, same raw sensors and goal error as GFT',()=>{
  const random=rng('nn-finite');for(let i=0;i<250;i++){
    const g=randomGenome(random,'nn'),gf=randomGenome(random);gf[4]=g[44];const s={x:10+random()*880,y:10+random()*520,heading:random()*6.28};
    const c=control(s,INITIAL,g,'nn'),f=control(s,INITIAL,gf);
    assert.equal(g.length,45);assert.deepEqual(c.distance,f.distance);assert.equal(c.error,f.error);
    assert.deepEqual(c.inputs,[...c.distance.map(v=>v/300),c.error/Math.PI]);
    for(const x of Object.values(c).flat())assert.ok(Number.isFinite(x));assert.ok(Math.abs(c.turn)<=1);assert.ok(c.speed>=.48&&c.speed<=3.5);
  }
});
test('NN GA: seed reproducible, real mutation bounds, population changes and initial copy',()=>{
  const a=new Evolution('nn-repeat',24,'nn'),b=new Evolution('nn-repeat',24,'nn'),initial=a.initialBest.slice(),pop=a.population.map(g=>g.slice());
  const r=rng('nn-repeat');assert.deepEqual(pop,Array.from({length:24},()=>randomGenome(r,'nn')));
  for(let i=0;i<5;i++){a.advance();b.advance();}
  assert.deepEqual(a.history,b.history);assert.deepEqual(a.best,b.best);assert.deepEqual(a.initialBest,initial);assert.notDeepEqual(a.population,pop);assert.ok(a.history.at(-1).score>=a.history[0].score);
  const m=mutate(initial,rng('mutation'),1,'nn');assert.notDeepEqual(m,initial);m.forEach((v,i)=>assert.ok(v>=NN_RANGES[i][0]&&v<=NN_RANGES[i][1]));
  assert.deepEqual(mutate(initial,rng('mutation'),0,'nn'),initial);
});
test('method switch preserves independent genes, seed, RNG, histories; reset isolates current method',()=>{
  const lab=new Lab('gft-seed'),reference=new Evolution('gft-seed');lab.current.advance();reference.advance();const gf=lab.current;
  lab.select('nn');const nn=lab.reset('nn-seed'),nr=new Evolution('nn-seed',24,'nn');nn.advance();nr.advance();assert.equal(nn.seed,'nn-seed');
  assert.equal(lab.select('gft'),gf);lab.current.advance();reference.advance();assert.deepEqual(lab.current.best,reference.best);assert.deepEqual(lab.current.history,reference.history);
  assert.equal(lab.select('nn'),nn);nn.advance();nr.advance();assert.deepEqual(nn.history,nr.history);assert.deepEqual(nn.best,nr.best);
  lab.reset('nn-seed');assert.equal(lab.current.generation,0);assert.notEqual(lab.current,nn);assert.equal(lab.select('gft'),gf);assert.equal(gf.generation,2);
});
test('NN physical steps, traces and scoring use same 270 deadline and layouts; no learned values hardcoded',()=>{
  const e=new Evolution('nn-layout',24,'nn'),g=e.best;assert.equal(WORLD.maxSteps,270);
  const initial=initialState(),c=nnControl(initial,INITIAL,g),next=step(initial,INITIAL,g,'nn'),heading=initial.heading+c.turn*.115;assert.equal(next.heading,Math.atan2(Math.sin(heading),Math.cos(heading)));assert.equal(next.x,initial.x+Math.cos(next.heading)*c.speed);assert.equal(next.y,initial.y+Math.sin(next.heading)*c.speed);
  for(const h of HEADINGS){const r=episode(g,INITIAL,h,true,'nn');assert.equal(r.path.length,r.steps+1);assert.ok(r.steps<=270);let s=initialState(h);for(let i=1;i<r.path.length;i++){s=step(s,INITIAL,g,'nn');assert.deepEqual(s,r.path[i]);}}
  const score=evaluate(g,INITIAL,'nn');assert.equal(score.score,HEADINGS.map(h=>episode(g,INITIAL,h,false,'nn').score).reduce((a,b)=>a+b)/3);
  const changed=INITIAL.map(o=>({...o}));changed[0]={x:80,y:270,r:55};assert.equal(evaluate(g,changed,'nn').collisions,3);assert.notDeepEqual(evaluate(g,changed,'nn'),score);
});
